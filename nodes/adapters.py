# -*- coding: utf-8 -*-
from logging import getLogger

import os

from chatterbot.conversation import Statement
from chatterbot.input import InputAdapter
from chatterbot.output import OutputAdapter
from speech import Player
import roslib
from std_msgs.msg import String
import rospy
class SpeechOutputAdapter(OutputAdapter):

    audio_filename = 'voice.wav'

    def __init__(self, **kwargs):
        super(SpeechOutputAdapter, self).__init__(**kwargs)
        self.pub = rospy.Publisher('speech', String)
        self.logger = kwargs.get('logger', getLogger(self.__class__.__name__))

        self.speech_storage = kwargs['chatbot'].storage
        self.tts = kwargs['text_to_speech']
        self.locker = kwargs['locker']

        self.offline = kwargs.get('out_offline', False)

    def process_response(self, statement, session_id=None):

        self.logger.info('process response for statement: \'{}\''.format(statement))

        text = statement.text
        print text
        self.pub.publish(String(text))
        data = self.speech_storage.get_speech(text)

        if data:
            with open(self.audio_filename, 'wb') as file:
                file.write(data)

        elif self.offline:
            self.logger.error('speech not found in offline mode - output failed')

            return

        else:
            self.logger.warning('speech not found in storage. downloading...')

            self.tts.save_to_file(text, self.audio_filename)
            self.speech_storage.insert_speech_file(text, self.audio_filename)

        self.logger.info('saying {}'.format(text))
        Player.play_file(self.audio_filename)

        self.logger.info('removing speech file {}'.format(self.audio_filename))
        os.remove(self.audio_filename)


class SpeechInputAdapter(InputAdapter):

    def __init__(self, **kwargs):
        super(SpeechInputAdapter, self).__init__(**kwargs)

        self.rec = kwargs['recognizer']
        self.locker = kwargs['locker']
        self.input_language = kwargs.get('input_language', 'ru_RU')
        self.request_timeout = kwargs.get('request_timeout')

    def process_input(self, *args, **kwargs):

        text = None

        with self.locker:
            while text is None:
                input_audio = self.rec.listen()

                if input_audio:
                    text = self.rec.recognize(input_audio,
                                              self.input_language)

            return Statement(text.lower())
