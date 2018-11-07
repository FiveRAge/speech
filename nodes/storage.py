# -*- coding: utf-8 -*-
import logging

import bson
from chatterbot.storage import MongoDatabaseAdapter
from pymongo import MongoClient


class SpeechStorageAdapter(MongoDatabaseAdapter):

    collection_name = 'speeches'

    @property
    def __name__(self):
        return 'SpeechStorage'

    def __init__(self, **kwargs):

        super(SpeechStorageAdapter,self).__init__(**kwargs)

        self.file_name = kwargs.get('filename')
        self.logger = logging.getLogger(__name__)

        if not self.file_name:
            self.url = kwargs['database_uri']
        
        self.logger.info('connected to database')
        print "test"

    def insert_speech_file(self, statement, audio_filename):
        raise NotImplementedError()

    def get_speech(self, statement):
        raise NotImplementedError()


class MongoSpeechStorageAdapter(SpeechStorageAdapter):

    def __init__(self, **kwargs):
        super(MongoSpeechStorageAdapter, self).__init__(**kwargs)

        self.client = MongoClient(self.url or 'mongodb://localhost:27017/')
        self.db = self.client['chatterbot-database']
        self.speech_collection = self.db[self.collection_name]

    def is_statement_exists(self, statement):
        return self.speech_collection.find_one({'statement': statement})

    def insert_speech_file(self, statement, audio_filename):

        self.speech_collection.insert_one({
            'statement': statement,
            'speech': bson.Binary(open(audio_filename, 'rb').read())
        })

        self.logger.info('inserted speech for statement \'{}\' to speech storage'
                         .format(statement))

    def get_speech(self, statement):

        self.logger.info('getting speech.')
        result = self.speech_collection.find_one({'statement': statement})

        if result is None:
            self.logger.error('speech not found for statement {}'.format(statement))
            return result

        return result['speech']
