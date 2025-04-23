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

print("Position player upright in Virtualizer!")
print("Press any key to continue . . .")
input()

JUMP_THRESHOLD = 10
CROUCH_THRESHOLD = -15
device.ResetPlayerHeight();

try:
    while True:
        os.system('cls')
        height = device.GetPlayerHeight()

        if height > JUMP_THRESHOLD:
            print("Player jumping!")
        elif height < CROUCH_THRESHOLD:
            print("Player crouching!")
        else:
            print("Player standing!")
        time.sleep(0.05)
except KeyboardInterrupt:
    pass