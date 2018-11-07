# -*- coding: utf-8 -*-
import copy
import logging
import os
import socket
from threading import Thread

import speech_recognition as sr
from yandex_speech import TTS


class Recognizer:

    def __init__(self, **kwargs):

        self.logger = kwargs.get('logger', logging.getLogger(__name__))

        if kwargs.get('list_mics'):
            print(''.join(['{} {}\n'.format(i, name) for i, name in enumerate(sr.Microphone.list_microphone_names())]))

            exit(0)

        self.rec = sr.Recognizer()
        self.rec.phrase_threshold = 0.1

        self.mic = sr.Microphone(device_index=kwargs.get('device_index'),
                                 sample_rate=kwargs.get('sample_rate'),
                                 chunk_size=kwargs.get('chunk_size') or 1024)

        adjust_mic_device_index = kwargs.get('adjust_device_index')

        if adjust_mic_device_index:
            self.adjust_mic = sr.Microphone(device_index=adjust_mic_device_index,
                                            sample_rate=kwargs.get('sample_rate'),
                                            chunk_size=kwargs.get('chunk_size') or 1024)
        else:
            self.adjust_mic = self.mic

        self.rec.operation_timeout = kwargs.get('operation_timeout')

        self.listen_timeout = kwargs.get('listen_timeout')
        self.phrase_time_limit = kwargs.get('phrase_time_limit')

        self.error_limit = kwargs.get('error_limit')
        self.error_count = self.error_limit + 1

    def adjust_recognizer(self, source=None):

        self.logger.info('adjusting microphone...')

        if source:
            self.rec.adjust_for_ambient_noise(source)
        else:
            with self.adjust_mic as source:
                self.rec.adjust_for_ambient_noise(source)

        self.logger.info('adjusting microphone done: {}'
                         .format(self.rec.energy_threshold))

    def listen(self):

        with self.mic as source:

            if self.adjust_mic != self.mic:
                Thread(target=self.adjust_recognizer).start()

            else:
                if self.error_limit and self.error_count > self.error_limit:
                    self.adjust_recognizer(source)
                    self.error_count = 0

            try:
                self.logger.info('listening...')

                return self.rec.listen(source=source,
                                       timeout=self.listen_timeout,
                                       phrase_time_limit=self.phrase_time_limit)
            except sr.WaitTimeoutError:

                self.error_count += 1
                self.logger.warning('waiting timeout ({} sec) expired'.format(self.listen_timeout))

    def recognize(self, audio_data, language):

        self.logger.info('sending data for recognition...')

        try:
            return self.rec.recognize_google(audio_data=audio_data,
                                             language=language)

        except sr.UnknownValueError:

            self.logger.warning('recognition failed')
            self.error_count += 1

        except (sr.RequestError, socket.timeout) as e:

            self.logger.error('request error: {}'.format(e))


class Player:

    @staticmethod
    def play_file(filename):

        name, extention = os.path.splitext(filename)

        if extention == '.wav':
            Player.play_wav(filename)
        else:
            raise Exception('Unknown file format!')

    @staticmethod
    def play_wav(filename):
        chunk = 1024

        import wave
        wf = wave.open(filename, 'rb')

        import pyaudio
        p = pyaudio.PyAudio()
        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        output=True)

        data = wf.readframes(chunk)

        while len(data) > 0:
            stream.write(data)
            data = wf.readframes(chunk)

        stream.stop_stream()
        stream.close()

        p.terminate()


class YandexTTS:

    def __init__(self, **kwargs):

        self.logger = kwargs.get('logger', logging.getLogger(__name__))
        self.yandex_tts = TTS(speaker=kwargs.get('speaker', 'jane'),
                              audio_format='wav',
                              emotion=kwargs.get('emotion'),
                              key=kwargs['key'])

    def save_to_file(self, text, filename):

        yandex_tts = copy.copy(self.yandex_tts)
        yandex_tts.generate(text)
        yandex_tts.save(filename)

        self.logger.info('saved speech to file {}'.format(filename))
