import numpy as np
import tensorflow as tf

from parameters import *


class SpeechRecognition:

    # audio parameters
    frames_per_buffer : int = FRAMES_PER_BUFFER
    channels : int = CHANNELS
    format = FORMAT
    rate : int = RATE
    duration : float  = duration
    audio : np.array = None

    # CNN model
    name : str = model_name
    model_path : str = None

    # classification variables
    text : str = ''
    labels : list = labels

    def __init__(self,model_path):

        self.model = tf.keras.models.load_model(model_path)

        self.p = pyaudio.PyAudio()


    def record_audio(self):
        """Record real time audio file from microphone
        The duration of the audio file needs to be the same as the one to train the model
        """
        stream = self.p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.frames_per_buffer
        )

        frames = []
        seconds = self.duration
        for _ in range(int(self.rate / self.frames_per_buffer * seconds)):
            data = stream.read(self.frames_per_buffer)
            frames.append(data)

        stream.stop_stream()
        stream.close()

        self.audio =  np.frombuffer(b''.join(frames), dtype=np.int16)

    def terminate(self):
        """Close the microphone
        """
        self.p.terminate()

    def get_spectrogram(self):
        """Compute the spectrogram of the wave function.
        Add a 'channels to the stft (Short Time Fourier Transform) so that the spectrogram 
        as image-like input data with convolution layer(which expect shape (`batch_size`, `height`, `width`, `channels`). 
        """
        spectrogram = tf.signal.stft(
            self.audio_tensor, frame_length=255, frame_step=128)
        spectrogram = tf.abs(spectrogram)
        spectrogram = spectrogram[..., tf.newaxis]
        self.spectrogram =  spectrogram

    def preprocess_audiobuffer(self):
        """
        waveform: ndarray of size (16000, )
        output: Spectogram Tensor of size: (1, `height`, `width`, `channels`)
        """
        self.audio =  self.audio / 32768
        self.audio_tensor = tf.convert_to_tensor(self.audio, dtype=tf.float32)
        self.get_spectrogram()
        self.spectrogram =  tf.expand_dims(self.spectrogram, 0)

    def predict_mic(self):
        """Predict text from audio file
        """
        self.record_audio()
        self.preprocess_audiobuffer()
        prediction = self.model(self.spectrogram)
        label_pred = np.argmax(prediction, axis=1)
        self.text = self.labels[label_pred[0]]
