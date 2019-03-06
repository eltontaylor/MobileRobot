#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 22 12:06:23 2018

@author: guest
"""
import time
import os

frequency = 1100 # Hertz

"""
for x in range(0, 5):
   
    os.system('play -n synth %s sin %s' % (0.3, frequency))
   
    time.sleep(.3)
    
    
"""

   
os.system('play -n synth %s sin %s' % (0.5, frequency))