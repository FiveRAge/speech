#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
import rospy

import os

from parsing import *
from std_msgs.msg import String
from speech import YandexTTS, Recognizer
def main():
    rospy.init_node('speech')
    args = parse_args()
    recognizer = Recognizer(listen_timeout=args.listen_timeout,
                            phrase_time_limit=args.phrase_time_limit,
                            error_limit=args.error_limit,
                            sample_rate=args.sample_rate,
                            chunk_size=args.chunk_size,
                            operation_timeout=args.operation_timeout,
                            list_mics=args.list_mics,
                            device_index=args.device_index,
                            adjust_device_index=args.adjust_device_index)

    tts = YandexTTS(speaker=args.speaker,
                    emotion=args.emotion,
                    key=args.yandex_key)

    chatbot = ChatBot(name=args.bot_name,
                      text_to_speech=tts,
                      recognizer=recognizer,
                      locker=args.locker,
                      out_offline=args.out_offline,
                      input_adapter=input_adapters[args.input_adapter],
                      output_adapter=output_adapters[args.output_adapter],
                      read_only=args.auto_learn,
                      database_uri=args.database_uri,
                      storage_adapter=args.storage_adapter,
                      logic_adapters=args.logic_adapters,
                      logger=args.logger)

    if args.train_files:

        for filename in args.train_files:

            name, extension = os.path.splitext(filename)
            trainer = file_trainers.get(extension)

            assert trainer, 'unknown file extension \'{}\''.format(extension)

            chatbot.set_trainer(trainer,
                                text_to_speech=tts,
                                offline=args.train_offline,
                                threads_num=args.downloading_threads_num)
            chatbot.train(filename)

        exit(0)

    if args.drop_database:

        chatbot.storage.drop()
        chatbot.logger.info('storage is dropped')

        exit(0)

    while True:
        chatbot.get_response(None)

if __name__ == "__main__":
    main()
