#!/usr/bin/env python

import sys
import speechd

client = speechd.SSIPClient('test')
client.set_output_module('festival')
client.set_language('en')
client.set_punctuation(speechd.PunctuationMode.SOME)
#client.speak("Hello World!")
client.speak(sys.argv[1])
client.close()
