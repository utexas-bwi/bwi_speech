#!/usr/bin/python
# Copyright (C) 2016 Google Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Sample that streams audio to the Google Cloud Speech API via GRPC."""

from __future__ import division

import contextlib
import functools
import re
import signal
import sys

import google.auth
import google.auth.transport.grpc
import google.auth.transport.requests
from google.cloud.grpc.speech.v1beta1 import cloud_speech_pb2
from google.rpc import code_pb2
import grpc
import pyaudio
from six.moves import queue
import rospy

#TODO Change if necessary. 
#Adds nlu_pipeline src folder in order to import modules from it. 
#nlu_pipeline_path = '/home/rcorona/catkin_ws/src/bwi_speech/NLL/CkyParser/src/'
nlu_pipeline_path = '/home/rcorona/catkin_ws/src/bwi_speech/NLL/CkyParser/src'
sys.path.append(nlu_pipeline_path)

#TODO Change if necessary. 
#Path to CKYParser
#parser_path = '/home/rcorona/catkin_ws/src/bwi_speech/src/parser.cky'
parser_path = '/home/rcorona/catkin_ws/src/bwi_speech/src/parser.cky'

#Nlu pipeline modules.
try:
    import CKYParser
    from CKYParser import count_ccg_productions
    from utils import *
    from Action import Action
    from ActionSender import ActionSender

    from Ontology import Ontology

    #grounder_path = '/home/justin/UT/Research/Code/dialog_active_learning/src'
    #sys.path.append(grounder_path)
    from Grounder import Grounder

except ImportError, e:
    print 'ERROR: Unable to load nlu_pipeline_modules! Verify that nlu_pipeline_path is set correctly!'
    print 'Error details: ' + str(e)
    sys.exit()

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

# The Speech API has a streaming limit of 60 seconds of audio*, so keep the
# connection alive for that long, plus some more to give the API time to figure
# out the transcription.
# * https://g.co/cloud/speech/limits#content
DEADLINE_SECS = 60 * 3 + 5
SPEECH_SCOPE = 'https://www.googleapis.com/auth/cloud-platform'

def replace_numbers_with_text(phrase): 
    phrase = phrase.replace('0', 'zero ')
    phrase = phrase.replace('1', 'one ')
    phrase = phrase.replace('2', 'two ')
    phrase = phrase.replace('3', 'three ')
    phrase = phrase.replace('4', 'four ')
    phrase = phrase.replace('5', 'five ')
    phrase = phrase.replace('6', 'six ')
    phrase = phrase.replace('7', 'seven ')
    phrase = phrase.replace('8', 'eight ')
    phrase = phrase.replace('9', 'nine ')

    return phrase

def tokenize_for_parser(phrase):
    
    """
    This function takes in a string
    and returns a tokenized version for 
    it that is compatible with the format
    the CKYParser expects. 
    """
    phrase = replace_numbers_with_text(phrase)


    #Currently only splits possessive. 
    return phrase.lower().replace("'", " '")


def make_channel(host, port):
    """Creates a secure channel with auth credentials from the environment."""
    # Grab application default credentials from the environment
    credentials, _ = google.auth.default(scopes=[SPEECH_SCOPE])

    # Create a secure channel using the credentials.
    http_request = google.auth.transport.requests.Request()
    target = '{}:{}'.format(host, port)

    return google.auth.transport.grpc.secure_authorized_channel(
        credentials, http_request, target)


def _audio_data_generator(buff):
    """A generator that yields all available data in the given buffer.

    Args:
        buff - a Queue object, where each element is a chunk of data.
    Yields:
        A chunk of data that is the aggregate of all chunks of data in `buff`.
        The function will block until at least one data chunk is available.
    """
    stop = False
    while not stop:
        # Use a blocking get() to ensure there's at least one chunk of data.
        data = [buff.get()]

        # Now consume whatever other data's still buffered.
        while True:
            try:
                data.append(buff.get(block=False))
            except queue.Empty:
                break

        # `None` in the buffer signals that the audio stream is closed. Yield
        # the final bit of the buffer and exit the loop.
        if None in data:
            stop = True
            data.remove(None)

        yield b''.join(data)


def _fill_buffer(buff, in_data, frame_count, time_info, status_flags):
    """Continuously collect data from the audio stream, into the buffer."""
    buff.put(in_data)
    return None, pyaudio.paContinue


# [START audio_stream]
@contextlib.contextmanager
def record_audio(rate, chunk):
    """Opens a recording stream in a context manager."""
    # Create a thread-safe buffer of audio data
    buff = queue.Queue()

    audio_interface = pyaudio.PyAudio()
    audio_stream = audio_interface.open(
        format=pyaudio.paInt16,
        # The API currently only supports 1-channel (mono) audio
        # https://goo.gl/z757pE
        channels=1, rate=rate,
        input=True, frames_per_buffer=chunk,
        # Run the audio stream asynchronously to fill the buffer object.
        # This is necessary so that the input device's buffer doesn't overflow
        # while the calling thread makes network requests, etc.
        stream_callback=functools.partial(_fill_buffer, buff),
    )

    yield _audio_data_generator(buff)

    audio_stream.stop_stream()
    audio_stream.close()
    # Signal the _audio_data_generator to finish
    buff.put(None)
    audio_interface.terminate()
# [END audio_stream]


def request_stream(data_stream, rate, interim_results=True):
    """Yields `StreamingRecognizeRequest`s constructed from a recording audio
    stream.

    Args:
        data_stream: A generator that yields raw audio data to send.
        rate: The sampling rate in hertz.
        interim_results: Whether to return intermediate results, before the
            transcription is finalized.
    """
    # The initial request must contain metadata about the stream, so the
    # server knows how to interpret it.
    recognition_config = cloud_speech_pb2.RecognitionConfig(
        # There are a bunch of config options you can specify. See
        # https://goo.gl/KPZn97 for the full list.
        encoding='LINEAR16',  # raw 16-bit signed LE samples
        sample_rate=rate,  # the rate in hertz
        # See http://g.co/cloud/speech/docs/languages
        # for a list of supported languages.
        language_code='en-US',  # a BCP-47 language tag
    )
    streaming_config = cloud_speech_pb2.StreamingRecognitionConfig(
        interim_results=interim_results,
        config=recognition_config,
    )

    yield cloud_speech_pb2.StreamingRecognizeRequest(
        streaming_config=streaming_config)

    for data in data_stream:
        # Subsequent requests can all just have the content
        yield cloud_speech_pb2.StreamingRecognizeRequest(audio_content=data)


def listen_print_loop(recognize_stream):
    """Iterates through server responses and prints them.

    The recognize_stream passed is a generator that will block until a response
    is provided by the server. When the transcription response comes, print it.

    In this case, responses are provided for interim results as well. If the
    response is an interim one, print a line feed at the end of it, to allow
    the next result to overwrite it, until the response is a final one. For the
    final one, print a newline to preserve the finalized transcription.
    """

    #TODO will hold alternative transcripts (i.e. n-best list). 
    responses = []

    num_chars_printed = 0
    for resp in recognize_stream:
        if resp.error.code != code_pb2.OK:
            raise RuntimeError('Server error: ' + resp.error.message)

        if not resp.results:
            continue

        # Display the top transcription
        result = resp.results[0]

        transcript = result.alternatives[0].transcript

        # Display interim results, but with a carriage return at the end of the
        # line, so subsequent lines will overwrite them.
        #
        # If the previous result was longer than this one, we need to print
        # some extra spaces to overwrite the previous result
        overwrite_chars = ' ' * max(0, num_chars_printed - len(transcript))

        if not result.is_final:
            pass
            #Commenting out to make output less noisy. 
            #sys.stdout.write(transcript + overwrite_chars + '\r')
            #sys.stdout.flush()


        else:
            #Get final transcript for this utterance and store it. 
            response = transcript + overwrite_chars
            responses.append(response)

            #Return response instead of looping so we may feed it to parser. 
            return response

            #Moved this functionality to the main loop, delete if wanted. 
            """
            # Exit recognition if any of the transcribed phrases could be
            # one of our keywords.
            if re.search(r'\b(exit|quit)\b', transcript, re.I):
                print('Exiting..')
                break

            num_chars_printed = 0
            """

def ground_parse_to_action(parse):
    #TODO integrate actual grounder.
    #TODO Use parse nodes, right now we're only using parse strings. 

    #If too complicated a semantic form, just throw away. 
    if 'lambda' in parse or not 'walk' in parse: 
        return None

    #Simple commands will have form of action(param1, param2,...) 
    #Therefore parse as follows:
    action, params = parse.split('(')

    #Post process action (ontologies don't necessarily match between our parser and the planner's). 
    #TODO Add more mappings for actions if needed. 
    if action == 'walk':
        #Format parameter to fit door description. 
        params = params.strip(')')
        
        if params == 'l3_414b':
            action = 'at'
            params = [params]
        else:
            action = 'beside'
            params = params.replace('l', 'd')
            params = [params]
            print params
    else:
        #Only allow the walk action for now. 
        return None

    return Action(action, params)

def main():
    service = cloud_speech_pb2.SpeechStub(
        make_channel('speech.googleapis.com', 443))

    #Instantiate ROS node. 
    #rospy.init_node('speech_language_acquisition')

    #Load parser from given path. 
    parser = load_obj_general(parser_path)

    #Will hold user speech transcript. 
    response = None

    #Action sender for sending actions to Segbot. #TODO None's are ont, lex, and out, do we need to add this later? 
    #action_sender = ActionSender(None, None, None)

    taking_input = True

    #Add grounder

    ontology = Ontology('/home/rcorona/catkin_ws/src/bwi_speech/src/ont.txt')
    #lexicon = Lexicon(ontology, lex_file)
    
    kb_predicates = dict()
    kb_predicates['person'] = [('stacy'), ('scott'), ('jesse'), ('shiqi'), ('jivko'), ('rodolfo'), ('aishwarya'), ('peter'), ('dana'), ('ray'), ('justin')]
    kb_predicates['item'] = ['chips', 'coffee', 'hamburger', 'juice', 'muffin']
    kb_predicates['office'] = [('l3_404'), ('l3_402'), ('l3_512'), ('l3_510'), ('l3_508'), ('l3_432'), ('l3_420'), ('l3_502'), ('l3_414b')]
    kb_predicates['possesses'] = [('justin', 'l3_402'), ('scott', 'l3_404'), ('l3_512', 'ray'), ('dana', 'l3_510'), ('peter', 'l3_508'), ('shiqi', 'l3_432'), ('jivko', 'l3_420'), ('stacy', 'l3_502'), ('jesse', 'l3_414b'), ('aishwarya', 'l3_414b'), ('rodolfo', 'l3_414b')]

    grounder = Grounder(ontology, perception_module=None, kb_predicates=kb_predicates, classifier_predicates=None)

    parse = parser.most_likely_cky_parse("walk to ray 's office").next()[0]
    semantic_node = parse.node

    print parser.print_parse(semantic_node)

    print grounder.ground_semantic_node(semantic_node)[0][0][0]

    sys.exit()


    while taking_input: 
        # For streaming audio from the microphone, there are three threads.
        # First, a thread that collects audio data as it comes in
        with record_audio(RATE, CHUNK) as buffered_audio_data:
            # Second, a thread that sends requests with that data
            requests = request_stream(buffered_audio_data, RATE)
            # Third, a thread that listens for transcription responses
            recognize_stream = service.StreamingRecognize(
                requests, DEADLINE_SECS)

            # Exit things cleanly on interrupt
            signal.signal(signal.SIGINT, lambda *_: recognize_stream.cancel())

            # Now, put the transcription responses to use.
            try:
                print "Please speak command: " 
                response = listen_print_loop(recognize_stream).strip()

                print "TRANSCRIPT: " + response

                if response == 'exit' or response == 'quit':
                    #Stop listening and exit loop. 
                    taking_input = False
                else:
                    #Parse it using parser.
                    response = tokenize_for_parser(response)
                    parse_node = parser.most_likely_cky_parse(response).next()[0] 
                    parse = parser.print_parse(parse_node.node)

                    print "PARSE: " + parse + '\n'

                    #Now ground.
                    action = ground_parse_to_action(parse)

                    #Not a valid action. 
                    if type(action) == type(None):
                        print 'Sorry, action not valid or misunderstood...'
                    else: 
                        #Simple confirmation, so that we don't perform unwanted action. 
                        print "I understood" 
                        print "ACTION: " + str(action.name)
                        print "PARAMS: " + str(action.params)
                        print "Is this correct? (yes/no): " 

                        response = listen_print_loop(recognize_stream).strip()

                        if response == 'yes':
                            print 'OK, performing action...\n'
                            #Send action to Segbot. 
                            #action_sender.execute_plan_action_client(action)
                        else:
                            print 'Ok, cancelling action...\n'

                recognize_stream.cancel()

            except grpc.RpcError as e:
                code = e.code()
                # CANCELLED is caused by the interrupt handler, which is expected.
                if code is not code.CANCELLED:
                    raise

            #If people take too long, this error comes up, ignore it. 
            except RuntimeError as e: 
                pass


if __name__ == '__main__':
    main()
