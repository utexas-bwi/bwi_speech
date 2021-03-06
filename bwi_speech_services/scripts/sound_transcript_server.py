#!/usr/bin/env python

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

DEADLINE_SECS = 60 * 3 + 5
SPEECH_SCOPE = 'https://www.googleapis.com/auth/cloud-platform'

NAME = 'sound_transcript_server'

import contextlib
import functools
import re
import signal
import sys

import google.auth
import google.auth.transport.grpc
import google.auth.transport.requests
from google.cloud.grpc.speech.v1beta1 import cloud_speech_pb2
from google.cloud.grpc.speech.v1beta1.cloud_speech_pb2 import SpeechContext
from google.rpc import code_pb2
import grpc
import pyaudio
from six.moves import queue

from bwi_speech_services.srv import *
from std_msgs.msg import String
import rospy

class SpeechTranscript:
    isGood = True
    responses = []

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

        #You can limit the lexicon here.
        #speech_context=SpeechContext(phrases=['bevo', 'longhorn', ..]),
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
    returnVal = SpeechTranscript()
    returnVal.responses = []
    returnVal.isGood = True

    num_chars_printed = 0
    for resp in recognize_stream:
        if resp.error.code != code_pb2.OK:
            returnVal.isGood = False
            return returnVal
            #raise RuntimeError('Server error: ' + resp.error.message)

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
            returnVal.responses.append(response)
            return returnVal

            #Return response instead of looping so we may feed it to parser. 

            #Moved this functionality to the main loop, delete if wanted. 
            """
            # Exit recognition if any of the transcribed phrases could be
            # one of our keywords.
            if re.search(r'\b(exit|quit)\b', transcript, re.I):
                print('Exiting..')
                break

            num_chars_printed = 0
            """
    return returnVal

def getAudioText():
    # For streaming audio from the microphone, there are three threads.
    # First, a thread that collects audio data as it comes in
    with record_audio(RATE, CHUNK) as buffered_audio_data:
        # Second, a thread that sends requests with that data
        requests = request_stream(buffered_audio_data, RATE)
        # Third, a thread that listens for transcription responses
        recognize_stream = googleSpeech.StreamingRecognize(
            requests, DEADLINE_SECS)
        retVal = listen_print_loop(recognize_stream)
    return retVal

def make_channel(host, port):
    """Creates a secure channel with auth credentials from the environment."""
    # Grab application default credentials from the environment
    credentials, _ = google.auth.default(scopes=[SPEECH_SCOPE])

    # Create a secure channel using the credentials.
    http_request = google.auth.transport.requests.Request()
    target = '{}:{}'.format(host, port)

    return google.auth.transport.grpc.secure_authorized_channel(
        credentials, http_request, target)

def handle_request_sound_transcript(req):
    requestSoundTranscriptResponse = RequestSoundTranscriptResponse()
    try:
        resp = getAudioText()
        requestSoundTranscriptResponse.utterance = resp.responses[0]
        requestSoundTranscriptResponse.isGood = resp.isGood
    except RuntimeError:
        global googleSpeech
        googleSpeech = cloud_speech_pb2.SpeechStub(
            make_channel('speech.googleapis.com', 443))
        requestSoundTranscriptResponse.isGood = False

    
    return requestSoundTranscriptResponse

def sound_transcript_server():
    rospy.init_node(NAME)
    while(True):
        try:
            global googleSpeech
            googleSpeech = cloud_speech_pb2.SpeechStub(
                make_channel('speech.googleapis.com', 443))
            serv = rospy.Service('sound_transcript_server', RequestSoundTranscript,
                handle_request_sound_transcript)

            rospy.spin()
            break
        except RuntimeError:
            pass

if __name__ == "__main__":
    sound_transcript_server()
