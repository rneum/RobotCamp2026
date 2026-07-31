# if you wanna run this you need pygame
# pyserial, librosa, and numpy
# James Harper UT Summer CS Academy for robotics July 30, 2026

import pygame
import time
import serial
import struct
import numpy as np
import librosa
import math


# if you want to control the mechanism with a controller's left joystick
# instead of with music
# on linux you need to be apart of the input group, which you can't do on the UT boxes
joystick = False

# path of audio file if for audio
path = "tiki.wav"

# arduino serial bus port, will be different on Mac/Windows or if you have another serial device plugged in on linux
port = "/dev/ttyUSB0"

# This is basically time step in number of audio samples used in the fourier transforms
# The default is 512, its bascially how accurate you want it to be but even fast fourier transforms can take time so you don't really need to change it
hopLength = 512


pygame.init()

if (joystick):
    pygame.joystick.init()

if (not joystick):

    # load the song so we can play it with pygame
    pygame.mixer.init()
    song = pygame.mixer.Sound(path)

    print("loaded song")

    # the load function tkaes in our path
    # sr is the sampling rate, setting it to none will do the "default"
    # im pretty sure that is just the samppling rate of the audio file you provide
    audio, samplingRate = librosa.load(path, sr=None)

    print("parsed song")

    # This is not needed, but another (less effective) method of audio visualization
    # Is to find the 1st centroid, which is basically the average frequency, for a given time, and render the ampltiude of that frequency
    # centroids = librosa.feature.spectral_centroid(y=audio, sr=samplingRate)[0]
    # print("computed centoroids")

    # avgFreq = np.mean(centroids)
    # print("Got mean")
    # stft = librosa.stft(audio)
    # print("DID STFT")
    # stft = np.abs(stft)
    # print("got the magnitude")

# convert to decibel then normalize, but this shouldn't really work
# A lot of songs try to keep the same decibal range and becuase decibel is a
# logarithmic scale it makes it so there is little variation and doesn't move that much
# audio = librosa.amplitude_to_db(audio, ref=np.max)
# min = np.min(audio)
# max = np.max(audio)
#
# audio = 2 * (audio - min) / (max - min) - 1
# More jargon for if you want to do it frequency based
# fftFreqs = librosa.fft_frequencies(sr=samplingRate)
# print("Did the FFT")
# binNumber = np.argmin(np.abs(fftFreqs-avgFreq))

# newAudio = stft[binNumber, :] #calm smiley face over here
# This is what I found gives the best results
# We do this operation called a root mean square root on the data which is like an average but better
# We seperate the audio by time bins of the hoplength
# THen compute this in each bin for a roughly instantaneous RMS
if (not joystick):
    newAudio = librosa.feature.rms(y=audio, hop_length=hopLength)[0]
    # compute mins and max for normalization
    min = np.min(newAudio)
    # why does python ternary have to be different bro
    max = (0.001 if math.isnan(newAudio[0])
           or newAudio[0] == 0 else newAudio[0])

    print("max: ", max)
if (joystick):
    print("program will pause till you've plugged in your controller")
    while pygame.joystick.get_count() == 0:
        pygame.event.pump()
        time.sleep(0.1)
    print("controller connected")
    controller = pygame.joystick.Joystick(0)
    controller.init()

# set up arudiono serial port
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=1)
time.sleep(2)

if (not joystick):
    song.play()
    startTime = time.time()

try:
    while True:
        pygame.event.pump()

        if (joystick):
            # Set the controller position
            x = controller.get_axis(0)
            print(x)
        else:
            # Check if song over
            if (int((time.time()-startTime) *
                    samplingRate/hopLength) >= len(newAudio)):
                print("Song Over")
                arduino.close()
                break

            # This was rishi's suggestion and it makes a big difference, we dynamically increase the max as the song plays, and it makes it way better
            if (newAudio[int((time.time()-startTime) *
                             samplingRate/hopLength)] > max):
                max = newAudio[int((time.time()-startTime) *
                                   samplingRate/hopLength)]

            # We find the elapse time, multiply that b the sample rate, and divide that by the portion we break the song into for the rms, and that gives us
            # The array index we need to visualize the amplitude
            amplitude = (newAudio[int((time.time()-startTime) *
                         samplingRate/hopLength)] - min) / (max - min) * 2.0
            print(amplitude)
            x = amplitude

        time.sleep(0.005)

        # Credit to some random guy on the arduino stack exchange
        # I was transmitting hte numbers to the arduino as strings and decoding them with UTF-8 but it was so buggy for some reason
        # What we do here is really smart, we essentially take a float, which both in arduino c++  and python are 32 bits, 4 bytes
        # We just take the 4 bytes, pack them into what I think is hexidecimal, and sends them over the serial bus
        # Then the arduino does something really cool called a Union
        # Essentially, it takes two objects, an array of bytes with length 4, and a float
        # We then go through, and take the 4 bytes python sent the arduino, and put them in the array
        # THe cool thing about union, is that it is 2 different data strucutres asigned to the same position in memory
        # By putting the bytes in the array, we can read those bytes, but interpret them as a float, and we have just transmitted our number
        # That is pretty awesome if you ask me
        arduino.write(struct.pack(
            "<f", x))
        # This is reading the code for debugging, I disable it because it slows the serial transmission a lot
        # arduino.write(struct.pack("<f", 0.5))
        # try:
        #    if (arduino.in_waiting > 0):
        #        print(arduino.readline().decode(
        #            'utf-8', errors='ignore').strip())
        #        # print("\n")
        # except serial.SerialException as e:
        #    print(f"Error: {e}")
except KeyboardInterrupt:
    arduino.close()
    print("exiting")
