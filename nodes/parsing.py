# -*- coding: utf-8 -*-
import argparse
import logging
from threading import Lock

from chatterbot import ChatBot
from trainers import DialogFileTrainer, ExcelTrainer


input_adapters = {
    'speech': 'adapters.SpeechInputAdapter',
    'terminal': 'chatterbot.input.TerminalAdapter',
}

output_adapters = {
    'speech': 'adapters.SpeechOutputAdapter',
    'terminal': 'chatterbot.output.TerminalAdapter'
}

file_trainers = {
    '.txt': DialogFileTrainer,
    '.xlsx': ExcelTrainer
}

yandex_key = 'bca0b653-ff55-40d5-b69f-6374436dac23'
yandex_speakers = ['jane', 'oksana', 'alyss', 'omazh', 'zahar', 'ermil']
yandex_emotions = ['neutral', 'good', 'evil']


def parse_args():

    parser = argparse.ArgumentParser()

    parser.add_argument('-q', '--quiet',
                        action='store_true',
                        help='silent mode.',
                        dest='quiet')

    bot_settings_group = parser.add_argument_group('chatterbot')

    bot_settings_group.add_argument('-w', '--auto-learn',
                                    action='store_false',
                                    help='if specified bot will remember phrases.',
                                    dest='auto_learn')

    bot_settings_group.add_argument('-i', '--input',
                                    help='specify input adapter ({}).'.format(', '.join(input_adapters.keys())),
                                    type=str,
                                    choices=list(input_adapters.keys()),
                                    dest='input_adapter')

    bot_settings_group.add_argument('-o', '--output',
                                    help='specify output adapter ({}).'.format(', '.join(output_adapters.keys())),
                                    type=str,
                                    choices=list(output_adapters.keys()),
                                    dest='output_adapter')

    bot_settings_group.add_argument('--out-offline',
                                    help='specify for prevent auto-downloading speech in runtime.',
                                    dest='out_offline',
                                    action='store_true')

    storage_settings = parser.add_argument_group('storage')

    storage_settings.add_argument('-u', '--db-uri',
                                  metavar='uri',
                                  type=str,
                                  dest='database_uri',
                                  help='specify database uri (default mongodb://localhost:27017/).')

    storage_settings.add_argument('-t', '--train',
                                  help='specify text files for training chat bot.',
                                  type=str,
                                  nargs='+',
                                  metavar='file',
                                  dest='train_files')

    bot_settings_group.add_argument('--train-offline',
                                    help='specify for prevent downloading speech on train.',
                                    dest='train_offline',
                                    action='store_true')

    storage_settings.add_argument('--drop-db',
                                  help='specify if you need to clear storage.',
                                  action='store_true',
                                  dest='drop_database')

    speech_settings_group = parser.add_argument_group('microphone')

    speech_settings_group.add_argument('--list-mics',
                                       help='list of available microphones.',
                                       action='store_true',
                                       dest='list_mics')

    speech_settings_group.add_argument('--device-index',
                                       help='select microphone by num. ',
                                       type=int,
                                       metavar='index',
                                       dest='device_index')

    speech_settings_group.add_argument('--adjust-device-index',
                                       help='select microphone by num. ',
                                       type=int,
                                       metavar='index',
                                       dest='adjust_device_index')

    speech_settings_group.add_argument('-l', '--listen-timeout',
                                       help='parameter is the maximum number of seconds \
                                       that this will wait for a phrase to start.',
                                       dest='listen_timeout',
                                       metavar='timeout',
                                       default=None,
                                       type=int)

    speech_settings_group.add_argument('-e', '--error-limit',
                                       help='limit error count before beginning re-adjusting microphone.',
                                       type=int,
                                       metavar='error_count',
                                       dest='error_limit',
                                       default=None)

    yandex_settings_group = parser.add_argument_group('yandex speech')

    yandex_settings_group.add_argument('-k', '--yandex-key',
                                       default=yandex_key,
                                       type=str,
                                       metavar='key',
                                       help='yandex speech key.',
                                       dest='yandex_key')

    yandex_settings_group.add_argument('--speaker',
                                       choices=yandex_speakers,
                                       type=str,
                                       help='specify voice speaker.',
                                       dest='speaker')

    yandex_settings_group.add_argument('--emotion',
                                       choices=yandex_emotions,
                                       type=str,
                                       help='specify speaker emotion.',
                                       dest='emotion')

    args = parser.parse_args()

    args.logger = logging.getLogger(ChatBot.__name__)
    args.bot_name = ''
    args.locker = Lock()

    # limits and timeouts

    args.error_limit = args.error_limit or 3
    args.listen_timeout = args.listen_timeout or 5

    args.downloading_threads_num = 10
    args.operation_timeout = 4
    args.phrase_time_limit = 5

    # adapters

    args.storage_adapter = 'storage.MongoSpeechStorageAdapter'
    args.logic_adapters = \
        [
            {
                'import_path': 'logic.TimeLogicAdapter'
            },
            {
                'import_path': 'logic.PleaseRepeatLogicAdapter'
            },
            {
                'import_path': 'chatterbot.logic.BestMatch',
                'statement_comparison_function': 'chatterbot.comparisons.levenshtein_distance',
                'response_selection_method': 'chatterbot.response_selection.get_random_response',
            }
        ]

    args.input_adapter = 'speech'
    args.output_adapter = 'speech'

    args.speaker = 'jane'
    args.emotion = 'normal'

    # microphone

    # args.sample_rate = 48000
    args.sample_rate = None
    args.chunk_size = 512

    if not args.quiet:
        logging.basicConfig(level=logging.INFO)

    return args
