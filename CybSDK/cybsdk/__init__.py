# coding: utf-8

from ctypes import *

is64 = (sizeof(c_voidp) == 8)
if is64:
    CybSDK_dll = cdll.LoadLibrary("x64/CybSDK_Native.dll")
else:
    CybSDK_dll = cdll.LoadLibrary("x86/CybSDK_Native.dll")

__all__ = ['Virt','IVirtDevice','VirtDeviceInfo']
from .VirtDeviceInfo import VirtDeviceInfo
from .IVirtDevice import IVirtDevice
from .Virt import Virt

""" Set up ctypes invokation """
CybSDK_dll.CybSDK_Virt_GetSDKVersion.restype = c_ushort
CybSDK_dll.CybSDK_Virt_FindDevice.restype = c_void_p
CybSDK_dll.CybSDK_Virt_GetDevice.argtypes = [POINTER(VirtDeviceInfo)]
CybSDK_dll.CybSDK_Virt_GetDevice.restype = c_void_p
CybSDK_dll.CybSDK_Virt_FindDevices.restype = c_void_p
CybSDK_dll.CybSDK_Virt_CreateDeviceMockupXInput.restype = c_void_p
CybSDK_dll.CybSDK_Virt_CreateDeviceMockupKeyboard.restype = c_void_p

CybSDK_dll.CybSDK_Virt_DeleteDevice.argtypes = [c_void_p]
CybSDK_dll.CybSDK_Virt_DeleteDevice.restype = None
CybSDK_dll.CybSDK_Virt_DeleteFoundDevices.argtypes = [POINTER(VirtDeviceInfo)]
CybSDK_dll.CybSDK_Virt_DeleteFoundDevices.restype = None

CybSDK_dll.CybSDK_VirtDevice_GetDeviceInfo.restype = POINTER(VirtDeviceInfo)

CybSDK_dll.CybSDK_VirtDevice_Open.restype = c_bool
CybSDK_dll.CybSDK_VirtDevice_IsOpen.restype = c_bool
CybSDK_dll.CybSDK_VirtDevice_Close.restype = c_bool

CybSDK_dll.CybSDK_VirtDevice_GetPlayerHeight.restype = c_float
CybSDK_dll.CybSDK_VirtDevice_ResetPlayerHeight.restype = None
CybSDK_dll.CybSDK_VirtDevice_GetPlayerOrientation.restype = c_float
CybSDK_dll.CybSDK_VirtDevice_ResetPlayerOrientation_V40.restype = None
CybSDK_dll.CybSDK_VirtDevice_GetMovementSpeed.restype = c_float
CybSDK_dll.CybSDK_VirtDevice_GetMovementDirection.restype = c_float

CybSDK_dll.CybSDK_VirtDevice_HasHaptic.restype = c_bool
CybSDK_dll.CybSDK_VirtDevice_HapticPlay.restype = None
CybSDK_dll.CybSDK_VirtDevice_HapticStop.restype = None
CybSDK_dll.CybSDK_VirtDevice_HapticSetGain.restype = None
CybSDK_dll.CybSDK_VirtDevice_HapticSetFrequency.restype = None
CybSDK_dll.CybSDK_VirtDevice_HapticSetVolume.restype = None

        