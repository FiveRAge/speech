# -*- coding: utf-8 -*-
import datetime
from random import choice

from chatterbot.conversation import Statement
from chatterbot.logic import LogicAdapter
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

class PleaseRepeatLogicAdapter(LogicAdapter):

    please_repeat_words = ['повтори']

    def can_process(self, statement):
        for please_repeat in self.please_repeat_words:

            if statement.text.startswith(please_repeat):
                return True
        return False

    def process(self, statement):

        session = self.chatbot.default_session
        statement, response_statement = session.conversation[-1]

        response_statement.confidence = 1

        return response_statement


class TimeLogicAdapter(LogicAdapter):

    whats_time_words = ['сколько время', 'который час']
    response_words = ['сейчас', 'время', 'на часах']

    def can_process(self, statement):
        for whats_time in self.whats_time_words:

            if statement.text.startswith(whats_time):
                return True
        return False

    def process(self, statement):

        time = datetime.datetime.now().strftime('%H %M')

        statement = Statement(text='{} {}'.format(choice(self.response_words), time))
        statement.confidence = 1

        return statement
