# need scikits audiolab for reading audio files
import librosa
import numpy as np
# need to import btrack, our beat tracker
import btrackpy

import pydub 

def read(f, normalized=False):
    """MP3 to numpy array"""
    a = pydub.AudioSegment.from_mp3(f)
    y = np.array(a.get_array_of_samples())
    if a.channels == 2:
        y = y.reshape((-1, 2))
    if normalized:
        return a.frame_rate, np.float32(y) / 2**15
    else:
        return a.frame_rate, y

audioFilePath = '/Users/oscarostlund/Programmation/projects/pico/jupyter/test-audio-single.wav'
audioFilePath = '/Users/oscarostlund/Programmation/projects/pico/jupyter/track.mp3'

print(f"Loadingfile {audioFilePath}")

# set the path to an audio file on your machine
# audioData, sr = librosa.load(audioFilePath, sr=44100)
sr, audioData = read(audioFilePath)
print(f"Loaded file, audioData shape {audioData.shape} : {audioData}, frameRate: {sr}")

# # convert to mono if need be
if (audioData.shape[1] == 2):
    print("converting to mono")
    audioData = np.average(audioData,axis=1)

# ==========================================    
# Usage A: track beats from audio            
beats = btrackpy.trackBeats(audioData)    
print(beats)

# ==========================================
# Usage B: extract the onset detection function
onsetDF = btrackpy.calculateOnsetDF(audioData)         
print(onsetDF)

# ==========================================
# Usage C: track beats from the onset detection function (calculated in Usage B)
ODFbeats = btrackpy.trackBeatsFromOnsetDF(onsetDF)
print(ODFbeats)
