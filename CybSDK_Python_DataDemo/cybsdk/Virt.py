# coding: utf-8

from ctypes import *
from . import CybSDK_dll
from .IVirtDevice import IVirtDevice
from ._VirtDevice import VirtDevice
from .VirtDeviceInfo import VirtDeviceInfo

MajorVersion = 4
MinorVersion = 4

## @brief Entry class of the Cyberith Virtualizer SDK
class Virt(object):
    """ Entry class of the Cyberith Virtualizer SDK """

    @staticmethod
    ## @brief Returns the version number of the Virtualizer SDK.
    #  @returns (MajorVersion << 8) + MinorVersion.
    def GetSDKVersion() -> int:
        """ Returns the version number of the Virtualizer SDK. """
        return int(MajorVersion << 8 | MinorVersion)

    @staticmethod
    ## @brief Returns the version number of the native C++ Virtualizer SDK.
    #  @returns (MajorVersion << 8) + MinorVersion.
    def GetNativeSDKVersion() -> int:        
        """ Returns the version number of the native C++ Virtualizer SDK. """        
        return int(CybSDK_dll.CybSDK_Virt_GetSDKVersion())

    @staticmethod
    ## @brief Finds a standard Virtualizer device object.
    #
    #  Factory methods for a Virtualizer device object.
    #  @return A valid IVirtDevice, or None if no Virtualizer device was found.
    def FindDevice() -> IVirtDevice:    
        """ Finds a standard Virtualizer device object. """ 
        native_device = c_void_p(CybSDK_dll.CybSDK_Virt_FindDevice())

        if native_device:
            return VirtDevice(native_device)

        return None

    @staticmethod
    ## @brief Returns a standard Virtualizer device described by the given device info.
    #  @param device A device info struct returned by Virt.FindDevices.
    #  @return A valid IVirtDevice, or None if the Virtualizer device was not found.
    def GetDevice(device : VirtDeviceInfo) -> IVirtDevice:
        """ Returns a standard Virtualizer device described by the given device info. """           
        native_device = c_void_p(CybSDK_dll.CybSDK_Virt_GetDevice(pointer(device)))

        if native_device:
            return VirtDevice(native_device)

        return None

    @staticmethod
    ## @brief Finds all Virtualizer devices and returns their info.
    #  @return A VirtDeviceInfo array, or None if no Virtualizer device was found.
    def FindDevices() -> Array:    
        """ Finds all Virtualizer devices and returns their info. """
        numDevices = c_uint()
        native_devices = c_void_p(CybSDK_dll.CybSDK_Virt_FindDevices(byref(numDevices)))
        numDevices = numDevices.value

        if numDevices == 0:
            return None

        native_devices = cast(native_devices, POINTER(VirtDeviceInfo))
        devices = (VirtDeviceInfo * numDevices)()

        for i in range(numDevices):
            devices[i] = native_devices[i]

        # We have copied all DeviceInfos to a python managed array so we can delete the native C++ memory
        CybSDK_dll.CybSDK_Virt_DeleteFoundDevices(native_devices);

        return devices

    @staticmethod
    ## @brief Creates a virtualizer mockup for xInput controller.
    #  @return A virtual IVirtDevice, driven by DirectX xInput.
    def CreateDeviceMockupXInput() -> IVirtDevice:        
        """ Creates a virtualizer mockup for xInput controller. """    
        native_device =  c_void_p(CybSDK_dll.CybSDK_Virt_CreateDeviceMockupXInput())

        if native_device:
            return VirtDevice(native_device)

        return None

    @staticmethod
    ## @brief Creates a virtualizer mockup for Keyboard.
    #  @return A virtual IVirtDevice, driven by Keyboard input.
    def CreateDeviceMockupKeyboard() -> IVirtDevice:      
        """ Creates a virtualizer mockup for Keyboard. """   
        native_device =  c_void_p(CybSDK_dll.CybSDK_Virt_CreateDeviceMockupKeyboard())

        if native_device:
            return VirtDevice(native_device)

        return None


