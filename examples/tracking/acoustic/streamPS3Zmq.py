#!/usr/bin/env python

import sounddevice as sd


devs = sd.query_devices()

fs = 16000
duration =5



sd.default.samplerate = fs
sd.default.channels = 4

myrecording = sd.rec(int(duration * fs))
# myrecording = sd.rec(int(duration * fs), samplerate=fs, channels=4, blocking=true)
# sd.wait()



#
