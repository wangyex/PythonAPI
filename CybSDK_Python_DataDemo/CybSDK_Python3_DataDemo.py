# coding: utf-8
 
import time
import os

from distutils.dir_util import copy_tree
# Copy the module and dlls next to this script
copy_tree("../CybSDK", ".")

from cybsdk import *

print("========================================")
print("C++ API Version: {0}.{1}".format(Virt.GetNativeSDKVersion() >> 8, Virt.GetNativeSDKVersion() & 0xff))
print("Py  API Version: {0}.{1}".format(Virt.GetSDKVersion() >> 8, Virt.GetSDKVersion() & 0xff))
print("========================================")

device = Virt.FindDevice()
if device == None:
    print("No Device Found!")
    print("========================================")
    input()
    exit(-2)


info = device.GetDeviceInfo()

print("Device Found: {0}".format(info.ProductName))
print("Firmware Version: {0}.{1}".format(info.MajorVersion, info.MinorVersion))
print("========================================")


if device.Open() == False:
    print("Unable to connect to Virtualizer!")
    print("========================================")
    input()
    exit(-3)

print("\r\nPress any key to continue . . .")
input()

try:
    while True:
        os.system('cls')
        print("========================================")
        print("Ring Height:        {0:10.2f}cm".format(device.GetPlayerHeight()))
        print("Player Orientation: {0:10.2f}°".format(device.GetPlayerOrientation() * 360))
        print("Movement Speed:     {0:10.2f}m/s".format(device.GetMovementSpeed()))
        print("Movement Direction: {0:10.2f}°".format(device.GetMovementDirection() * 180))
        print("========================================")
        time.sleep(0.05)
except KeyboardInterrupt:
    pass

