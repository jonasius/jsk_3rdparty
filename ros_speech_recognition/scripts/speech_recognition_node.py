#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import actionlib
import rospy
import speech_recognition as SR
import json
import wave
import time
import os
from threading import Lock
from six.moves import queue
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Bool, ColorRGBA, String
from std_srvs.srv import SetBool
from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from speech_recognition_msgs.srv import SpeechRecognition
from speech_recognition_msgs.srv import SpeechRecognitionResponse

from dynamic_reconfigure.server import Server
from ros_speech_recognition.cfg import SpeechRecognitionConfig as Config
from google.cloud import speech_v1
from google.cloud.speech_v1 import enums
from google.cloud.speech_v1 import types

class ROSAudio(SR.AudioSource):
    # def __init__(self, topic_name="audio", depth=16, sample_rate=16000, chunk_size=1024, buffer_size=10240):
    def __init__(self, topic_name="audio", depth=16, sample_rate=16000, chunk_size=1600, buffer_size=1600):
        # assert buffer_size > chunk_size

        self.topic_name = topic_name
        self.buffer_size = buffer_size

        if depth == 8:
            self.SAMPLE_WIDTH = 1L
        elif depth == 16:
            self.SAMPLE_WIDTH = 2L
        elif depth == 32:
            self.SAMPLE_WIDTH = 4L
        else:
            raise ValueError("depth must be 8, 16 or 32")

        self.SAMPLE_RATE = sample_rate
        self.CHUNK = chunk_size

        self.stream = None


    def open(self):
        if self.stream is not None:
            self.stream.close()
            self.stream = None
        self.stream = ROSAudio.AudioStream(self.topic_name, self.buffer_size)
        return self

    def close(self):
        self.stream.close()
        self.stream = None

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    class AudioStream(object):
        def __init__(self, topic_name, buffer_size=1600):
            self.buffer_size = buffer_size
            # self.buffer_size = 1600
            self.lock = Lock()
            self.buffer = bytes()
            # self.sub_audio = rospy.Subscriber(
            #     topic_name, AudioData, self.audio_cb)
            self.sub_audio = rospy.Subscriber(
                topic_name, AudioData, self._fill_buffer)
            self._buff = queue.Queue()
            # self.closed = True
            self.closed = False

        def __enter__(self):
            # rospy.loginfo("__enter__ AudioStream called!")
            self.closed = False
            return self

        def __exit__(self, exc_type, exc_value, traceback):
            self.closed = True
            # Signal the generator to terminate so that the client's
            # streaming_recognize method will not block the process termination.
            self._buff.put(None)
        
        def generator(self):
            while not self.closed:
                chunk = self._buff.get()
                if chunk is None:
                    return
                data = [chunk]
                # while not queue.empty():
                while True:
                    try:
                        chunk = self._buff.get(block=False)
                        if chunk is None:
                            return
                        data.append(chunk)
                    except queue.Empty:
                        break
                # print(data)
                yield b''.join(data)

        # def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        def _fill_buffer(self, msg):
            # print("filling buffer")
            """Continuously collect data from the audio stream, into the buffer."""
            self._buff.put(bytes(msg.data))
            # self._buff.put(msg.data)
            # return None

        def close(self):
            self.closed = True
            self._buff.put(None)


class ROSSpeechRecognition(object):
    def __init__(self):
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = rospy.get_param(
            '/google_credentials_path')
        self.default_duration = rospy.get_param("~duration", 10.0)
        self.engine = None
        self.record_wave = False
        self.wavefile = None
        self.recognizer = SR.Recognizer()
        self.audio = ROSAudio(topic_name="audio",
                              depth=rospy.get_param("~depth", 16),
                              sample_rate=rospy.get_param("~sample_rate", 16000))

        # initialize sound play client
        self.act_sound = actionlib.SimpleActionClient("sound_play", SoundRequestAction)
        if not self.act_sound.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("Failed to find sound_play action. Disabled audio alert")
            self.act_sound = None
        self.signals = {
            "start": rospy.get_param("~start_signal",
                                     "/usr/share/sounds/ubuntu/stereo/bell.ogg"),
            "recognized": rospy.get_param("~recognized_signal",
                                          "/usr/share/sounds/ubuntu/stereo/button-toggle-on.ogg"),
            "success": rospy.get_param("~success_signal",
                                       "/usr/share/sounds/ubuntu/stereo/message-new-instant.ogg"),
            "timeout": rospy.get_param("~timeout_signal",
                                       "/usr/share/sounds/ubuntu/stereo/window-slide.ogg"),
        }
        self.status_led = rospy.Publisher('/status_led', ColorRGBA, queue_size=10)
        self.status_led_think = rospy.Publisher('/status_led_think', Bool, queue_size=10)

        self.dyn_srv = Server(Config, self.config_callback)
        self.stop_fn = None
        self.continuous = rospy.get_param("~continuous", False)
        if self.continuous:
            rospy.loginfo("Enabled continuous mode")
            self.pub = rospy.Publisher("/Tablet/voice",
                                       SpeechRecognitionCandidates,
                                       queue_size=1)
        else:
            # self.srv = rospy.Service("speech_recognition",
            #                          SpeechRecognition,
            #                          self.speech_recognition_srv_cb)
            self.srv = rospy.Service("speech_recognition",
                                     SpeechRecognition,
                                     self.speech_recognition_stream_srv_test)
            # self.record_service = rospy.Service("speech_recognition",
            #                                     SpeechRecognition,
            #                                     self.speech_recognition_wave_record)

        # Service to call ambient noise adjust manually
        self.ambient_noise_adjust_service = rospy.Service(
            "speech_recognition/ambient_noise_adjust", SetBool, self.ambient_noise_adjust)
        self.stt_pub = rospy.Publisher('/stt', String, queue_size=1)
        self.tts_pub = rospy.Publisher('/tts', String, queue_size=1)

    def config_callback(self, config, level):
        # config for engine
        self.language = config.language
        if self.engine != config.engine:
            self.args = {}
            self.engine = config.engine

        # config for adaptive thresholding
        self.dynamic_energy_threshold = config.dynamic_energy_threshold
        if self.dynamic_energy_threshold:
            config.energy_threshold = self.recognizer.energy_threshold
        else:
            self.recognizer.energy_threshold = config.energy_threshold
        self.recognizer.dynamic_energy_adjustment_damping = config.dynamic_energy_adjustment_damping
        self.recognizer.dynamic_energy_ratio = config.dynamic_energy_ratio

        # config for timeout
        if config.listen_timeout > 0.0:
            self.listen_timeout = config.listen_timeout
        else:
            self.listen_timeout = None
        if config.phrase_time_limit > 0.0:
            self.phrase_time_limit = config.phrase_time_limit
        else:
            self.phrase_time_limit = None
        if config.operation_timeout > 0.0:
            self.recognizer.operation_timeout = config.operation_timeout
        else:
            self.recognizer.operation_timeout = None

        if config.debug_record and self.wavefile == None:
            fname = str(rospy.get_rostime())[0:10]
            rospy.logwarn(
                "Enabled debug_record fname: testrecording{}.wav".format(fname))
            self.wavefile = self._prepare_file(
                "testrecording{}.wav".format(fname))
            self.record_wave = True
        elif not config.debug_record:
            if self.wavefile != None:
                rospy.logwarn("debug_record disabled")
                self.wavefile.close()
                self.wavefile = None
                self.record_wave = False

        # config for VAD
        if config.pause_threshold < config.non_speaking_duration:
            config.pause_threshold = config.non_speaking_duration
        self.recognizer.pause_threshold = config.pause_threshold
        self.recognizer.non_speaking_duration = config.non_speaking_duration
        self.recognizer.phrase_threshold = config.phrase_threshold

        return config

    def ambient_noise_adjust(self, param):
        if param.data == True:
            rospy.loginfo("Adjusting speech recognition for ambient noise")
            with self.audio as src:
                self.recognizer.adjust_for_ambient_noise(src)
                rospy.loginfo("Set minimum energy threshold to {}".format(self.recognizer.energy_threshold))

    def play_sound(self, key, timeout=5.0):
        if self.act_sound is None:
            return
        req = SoundRequest()
        req.sound = SoundRequest.PLAY_FILE
        req.command = SoundRequest.PLAY_ONCE
        req.arg = self.signals[key]
        goal = SoundRequestGoal(sound_request=req)
        self.act_sound.send_goal_and_wait(goal, rospy.Duration(timeout))

    def recognize(self, audio):
        recog_func = None
        if self.engine == Config.SpeechRecognition_Google:
            if not self.args:
                self.args = {'key': rospy.get_param("~google_key", None)}
            recog_func = self.recognizer.recognize_google
        elif self.engine == Config.SpeechRecognition_GoogleCloud:
            if not self.args:
                credentials_path = rospy.get_param("~google_cloud_credentials_json", None)
                if credentials_path is not None:
                    with open(credentials_path) as j:
                        credentials_json = j.read()
                else:
                    credentials_json = None
                self.args = {'credentials_json': credentials_json,
                             'preferred_phrases': rospy.get_param('~google_cloud_preferred_phrases', None)}
            recog_func = self.recognizer.recognize_google_cloud
        elif self.engine == Config.SpeechRecognition_Sphinx:
            recog_func = self.recognizer.recognize_sphinx
        elif self.engine == Config.SpeechRecognition_Wit:
            recog_func = self.recognizer.recognize_wit
        elif self.engine == Config.SpeechRecognition_Bing:
            if not self.args:
                self.args = {'key': rospy.get_param("~bing_key")}
            recog_func = self.recognizer.recognize_bing
        elif self.engine == Config.SpeechRecognition_Houndify:
            recog_func = self.recognizer.recognize_houndify
        elif self.engine == Config.SpeechRecognition_IBM:
            recog_func = self.recognizer.recognize_ibm

        return recog_func(audio_data=audio, language=self.language, **self.args)

    def audio_cb(self, _, audio):
        rospy.loginfo("audio_cb betreten")
        try:
            rospy.logdebug("Waiting for result... (Sent %d bytes)" % len(audio.get_raw_data()))
            result = self.recognize(audio)
            rospy.loginfo("Result: %s" % result.encode('utf-8'))
            msg = SpeechRecognitionCandidates(transcript=[result])
            self.pub.publish(msg)
        except SR.UnknownValueError as e:
            if self.dynamic_energy_threshold:
                self.recognizer.adjust_for_ambient_noise(self.audio)
                rospy.loginfo("Updated energy threshold to %f" % self.recognizer.energy_threshold)
        except SR.RequestError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))

    def start_speech_recognition(self):
        if self.dynamic_energy_threshold:
            with self.audio as src:
                self.recognizer.adjust_for_ambient_noise(src)
                rospy.loginfo("Set minimum energy threshold to {}".format(
                    self.recognizer.energy_threshold))
        rospy.loginfo("Started start_speech_recognition continuous")
        self.stop_fn = self.recognizer.listen_in_background(
            self.audio, self.audio_cb, phrase_time_limit=self.phrase_time_limit)
        rospy.on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        if self.record_wave:
            self.wavefile.close()
        if self.stop_fn is not None:
            self.stop_fn()

    def _prepare_file(self, fname, mode='wb'):
        wavefile = wave.open(fname, mode)
        # wavefile.setnchannels(self.channels)
        wavefile.setnchannels(1)
        # wavefile.setsampwidth(self.pyaudio.get_sample_size(pyaudio.paInt16))
        wavefile.setsampwidth(2)
        # wavefile.setframerate(self.rate)
        wavefile.setframerate(16000)
        return wavefile

    def listen_print_loop(self, responses):
        num_chars_printed = 0
        for response in responses:
            if not response.results:
                continue

            # The `results` list is consecutive. For streaming, we only care about
            # the first result being considered, since once it's `is_final`, it
            # moves on to considering the next utterance.
            result = response.results[0]
            print(result)
            if not result.alternatives:
                continue

            # Display the transcription of the top alternative.
            transcript = result.alternatives[0].transcript

            # Display interim results, but with a carriage return at the end of the
            # line, so subsequent lines will overwrite them.
            #
            # If the previous result was longer than this one, we need to print
            # some extra spaces to overwrite the previous result
            # overwrite_chars = ' ' * (num_chars_printed - len(transcript))
            
            if not result.is_final:
                # sys.stdout.write(transcript + overwrite_chars + '\r')
                # sys.stdout.flush()
                print(transcript)
                num_chars_printed = len(transcript)

            else:
                print(transcript + overwrite_chars)

                # Exit recognition if any of the transcribed phrases could be
                # one of our keywords.
                # if re.search(r'\b(exit|quit)\b', transcript, re.I):
                #     print('Exiting..')
                #     break
                num_chars_printed = 0
                return(transcript + overwrite_chars)


    ##################
    ##################
    ##################
    ##################
    def speech_recognition_stream_srv_test(self, req):
        rospy.loginfo("started speech recognition stream")
        self.stt_pub.publish("")
        self.tts_pub.publish("")
        res = SpeechRecognitionResponse()
        transcript = None
        result_is_final = False
        language_code = 'de-DE'  # a BCP-47 language tag
        custom_phrases = rospy.get_param(
            '~google_cloud_preferred_phrases', None)
        client = speech_v1.SpeechClient()
        config = types.RecognitionConfig(
            encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=self.audio.SAMPLE_RATE,
            language_code=language_code,
            speech_contexts=[speech_v1.types.SpeechContext(
                phrases=custom_phrases,)],
            model='command_and_search'
        )
        streaming_config = types.StreamingRecognitionConfig(
            config=config, single_utterance=True, interim_results=True)
        with self.audio as saudio:
            with saudio.stream as audio:
                transcript = None
                # Set LED Ring green
                audio_generator = audio.generator()
                self.status_led.publish(0.0, 1.0, 0.0, 0.5)
                requests = (types.StreamingRecognizeRequest(audio_content=content)
                            for content in audio_generator)
                responses = client.streaming_recognize(streaming_config, requests)
                for response in responses:
                    if not response.results:
                        # self.status_led.publish(1.0, 0.0, 1.0, 0.5)
                        audio.close()
                        # return res
                    for result in response.results:
                        self.status_led.publish(0.0, 1.0, 0.0, 0.5)
                        if not result.alternatives:
                            self.status_led.publish(0.0, 0.0, 1.0, 0.5)
                            rospy.logerr(result)
                            continue
                            # break
                        transcript = result.alternatives[0].transcript
                        self.status_led.publish(0.0, 1.0, 0.0, 0.5)
                        self.stt_pub.publish(transcript)
                        rospy.loginfo('Text:{}; Finished: {}; Stability: {}'.format(
                            result.alternatives[0].transcript.encode('utf-8'), result.is_final, result.stability))
                        res.result = SpeechRecognitionCandidates(transcript=[transcript])
                        if result.is_final:
                            result_is_final = result.is_final
                            # Set LED Ring to "think"
                            self.status_led_think.publish(True)
                            res.result = SpeechRecognitionCandidates(transcript=[transcript])
                            return res
                    rospy.logwarn("DEBUG ENDE inner FOR")
                    rospy.logwarn(response)
                    rospy.logwarn(res.result)
                    if response.speech_event_type and transcript is None:
                        # No speech input
                        self.status_led.publish(1.0, 0.0, 0.0, 1.0)
                        result_is_final = True
                        return res
                    else:
                        rospy.logwarn("ELSE clause")
                        rospy.logwarn(response)
                    # return res
                return res


    def speech_recognition_wave_record(self, req):
        res = SpeechRecognitionResponse()

        # duration = req.duration
        # if duration <= 0.0:
        #     duration = self.default_duration

        with self.audio as src:
            # if self.dynamic_energy_threshold:
            #     self.recognizer.adjust_for_ambient_noise(src)
            #     rospy.loginfo("Set minimum energy threshold to %f" % self.recognizer.energy_threshold)

            # if not req.quiet:
            #     self.play_sound("start", 0.1)

            # start_time = rospy.Time.now()
            # while (rospy.Time.now() - start_time).to_sec() < duration:
            #     self.status_led.publish(0.0, 1.0, 0.0, 0.5)
                # rospy.loginfo("Waiting for speech...")
            try:
                # audio = self.recognizer.listen(
                #     src, timeout=self.listen_timeout, phrase_time_limit=self.phrase_time_limit)
                audio = self.recognizer.record(
                    src)
                # start = time.time()
                rospy.logwarn("recording wavefile")    
                self.wavefile.writeframes(audio.get_wav_data())
                # rospy.logwarn("time for wave write: {}".format(time.time()-start))
            except SR.WaitTimeoutError as e:
                rospy.logwarn(e)
                break
                # if not req.quiet:
                #     self.play_sound("recognized", 0.05)
                # self.status_led_think.publish(True)
                # rospy.loginfo("Waiting for result... (Sent %d bytes)" % len(audio.get_raw_data()))

                # try:
                #     result = self.recognize(audio)
                #     rospy.loginfo("Result: %s" % result.encode('utf-8'))
                #     if not req.quiet:
                #         self.play_sound("success", 0.1)
                #     res.result = SpeechRecognitionCandidates(transcript=[result])
                #     return res
                # except SR.UnknownValueError:
                #     if self.dynamic_energy_threshold:
                #         self.recognizer.adjust_for_ambient_noise(src)
                #         rospy.loginfo("Set minimum energy threshold to %f" % self.recognizer.energy_threshold)
                # except SR.RequestError as e:
                #     rospy.logerr("Failed to recognize: %s" % str(e))
                # rospy.sleep(0.1)
                if rospy.is_shutdown():
                    break

            # Timeout
            # if not req.quiet:
            #     self.play_sound("timeout", 0.1)
            return res

    def spin(self):
        if self.continuous:
            self.start_speech_recognition()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node("speech_recognition")
    rec = ROSSpeechRecognition()
    rec.spin()
