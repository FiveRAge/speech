# -*- coding: utf-8 -*-
import os
from threading import Thread, Semaphore

from chatterbot.conversation import Response
from chatterbot.trainers import ListTrainer
from openpyxl import load_workbook


class SpeechTrainer(ListTrainer):

    def __init__(self, storage, **kwargs):
        super().__init__(storage, **kwargs)

        self.speech_storage = storage

        self.text_to_speech = kwargs.get('text_to_speech')
        self.offline = kwargs.get('offline', False)
        self.threads_num = kwargs['threads_num']

    def download_speeches(self, statements):

        if self.offline:

            self.logger.warning('specified offline mode for training')
            return

        assert self.speech_storage is not None, 'speech_storage is none'

        self.logger.info('starting downloading {} speeches'.format(len(statements)))

        semaphore = Semaphore(self.threads_num)
        threads = []

        def thread_runner(stat, file, storage, logger):

            with semaphore:

                logger.info('downloading {}'.format(file))
                self.text_to_speech.save_to_file(stat, file)

                storage.insert_speech_file(stat, file)
                os.remove(file)

                logger.info('{} done'.format(file))

        for i, statement in enumerate(statements):

            if self.speech_storage.is_statement_exists(statement):
                self.logger.warning('statement {} already exists in speech storage'.format(statement))

                continue

            thread_runner_args = (statement, '{}_{}'.format(i, 'voice.wav'),
                                  self.speech_storage,
                                  self.logger)

            thread = Thread(target=thread_runner, args=thread_runner_args)
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

        self.logger.info('speeches has been downloaded')


class DialogFileTrainer(SpeechTrainer):

    def __init__(self, storage, **kwargs):
        super().__init__(storage, **kwargs)

    def train(self, filename):

        statements = []
        self.logger.info('beginning training')

        with open(filename, 'r', encoding='utf-8') as f:

            statements.extend(f.read().split('\n'))

            if statements.count(''):
                statements.remove('')

            super().train(statements)

        try:
            self.download_speeches(statements)

        except Exception as e:
            self.logger.error('failed to load speeches')
            raise e

        self.logger.info('training complete')


class ExcelTrainer(SpeechTrainer):

    def __init__(self, storage, **kwargs):
        super().__init__(storage, **kwargs)

    def train(self, filename):

        self.logger.info('beginning training')

        wb = load_workbook(filename, read_only=True)
        ws = wb.active

        statements = []

        count = 0

        for row in ws.rows:
            if row[0].value is None:
                continue

            requests = [str(r).strip() for r in str(row[0].value).strip('; ').split(';')]
            responses = [str(r).strip() for r in str(row[1].value).strip('; ').split(';')]

            count += len(responses)
            statements.extend(responses)

            for request in requests:
                for response in responses:

                    statement = self.get_or_create(response.strip())
                    statement.add_response(Response(text=request.strip()))

                    self.storage.update(statement)

        self.logger.info('training complete {} statements'.format(count))

        try:
            self.download_speeches(statements)

        except Exception as e:
            self.logger.error('failed to load speeches')
            print(e)
