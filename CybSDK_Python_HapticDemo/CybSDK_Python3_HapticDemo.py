# coding: utf-8
 
import time
import os

from distutils.dir_util import copy_tree
# Copy the module and dlls next to this script
copy_tree("../CybSDK", ".")

from cybsdk import *

device = Virt.FindDevice()
if device == None:
    print("No Device Found!")
    input()
    exit(-2)

if device.Open() == False:
    print("Unable to connect to Virtualizer!")
    input()
    exit(-3)

isActive = False

while True:
    os.system('cls')

    selection = int(input("""Choose function:
0: Toggle active
1: Set Gain
2: Set Frequency
3: Set Volume
4: Quit
Function: """))

    if selection == 0:
        if isActive == True:
            device.HapticStop()
        else:
            device.HapticPlay()
        isActive = not isActive
    elif selection == 1:
        selection = int(input("Set Gain (0-3): "))
        device.HapticSetGain(selection);
    elif selection == 2:
        selection = int(input("Set Frequency (10-80): "))
        device.HapticSetFrequency(selection);
    elif selection == 3:
        selection = int(input("Set Volume (0-100): "))
        device.HapticSetVolume(selection);
    else:
        device.HapticStop()
        exit(0)
