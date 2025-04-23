# coding: utf-8

import sys 
from distutils.dir_util import copy_tree
# Copy the module and dlls next to this script
copy_tree("../CybSDK", ".")

try:
    from cybsdk import *
    version = Virt.GetNativeSDKVersion()
except OSError:
    print("[Fatal] Couldn't load CybSDK.dll!", file=sys.stderr)
    input()
    exit(-1)

device = Virt.FindDevice()
if device == None:
    print("[Fatal] No Virtualizer connected!", file=sys.stderr)
    input()
    exit(-2)

info = device.GetDeviceInfo()
product_name = info.ProductName

if device.Open() == False:
    print("[Fatal] Unable to connect to Virtualizer!", file=sys.stderr)
    input()
    exit(-3)

ring_height = device.GetPlayerHeight()
ring_angle = device.GetPlayerOrientation() * 360
movement_direction = device.GetMovementDirection() * 180
movement_speed = device.GetMovementSpeed()

print("Connection successful!")
input()
