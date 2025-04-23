# coding: utf-8

from ctypes import *
from . import CybSDK_dll
from .IVirtDevice import IVirtDevice
from .VirtDeviceInfo import VirtDeviceInfo

## @brief Native Implementation invoking the underlying C++ SDK
class VirtDevice(IVirtDevice):
    """ Native Implementation invoking the underlying C++ SDK """

    ## @brief Unmanaged pointer to the native device.
    def __init__(self, native_device : c_void_p):
        self._native_device = native_device

    ## @brief Free the unmanaged pointer to the native device.
    def __del__(self):
        CybSDK_dll.CybSDK_Virt_DeleteDevice(self._native_device)
        self._native_device = c_void_p(None)


    ## @inheritdoc  
    def Open(self) -> bool:  
        return CybSDK_dll.CybSDK_VirtDevice_Open(self._native_device)
        
    ## @inheritdoc  
    def IsOpen(self) -> bool:
        return CybSDK_dll.CybSDK_VirtDevice_IsOpen(self._native_device)

    ## @inheritdoc  
    def Close(self) -> bool:
        return CybSDK_dll.CybSDK_VirtDevice_Close(self._native_device)


    ## @inheritdoc  
    def GetDeviceInfo(self) -> VirtDeviceInfo:
        return CybSDK_dll.CybSDK_VirtDevice_GetDeviceInfo(self._native_device).contents


    ## @inheritdoc  
    def GetPlayerHeight(self) -> float:        
        return CybSDK_dll.CybSDK_VirtDevice_GetPlayerHeight(self._native_device)

    ## @inheritdoc  
    def ResetPlayerHeight(self):       
        CybSDK_dll.CybSDK_VirtDevice_ResetPlayerHeight(self._native_device)

    ## @inheritdoc  
    def GetPlayerOrientation(self) -> float:        
        return CybSDK_dll.CybSDK_VirtDevice_GetPlayerOrientation(self._native_device)

    ## @inheritdoc  
    def ResetPlayerOrientation(self):                
        CybSDK_dll.CybSDK_VirtDevice_ResetPlayerOrientation_V40(self._native_device)

    ## @inheritdoc  
    def GetMovementSpeed(self) -> float:              
        return CybSDK_dll.CybSDK_VirtDevice_GetMovementSpeed(self._native_device)

    ## @inheritdoc  
    def GetMovementDirection(self) -> float: 
        return CybSDK_dll.CybSDK_VirtDevice_GetMovementDirection(self._native_device)


    ## @inheritdoc  
    def HasHaptic(self) -> bool:       
        return CybSDK_dll.CybSDK_VirtDevice_HasHaptic(self._native_device) 
    
    ## @inheritdoc  
    def HapticPlay(self):  
      CybSDK_dll.CybSDK_VirtDevice_HapticPlay(self._native_device) 
        
    ## @inheritdoc  
    def HapticStop(self):  
        CybSDK_dll.CybSDK_VirtDevice_HapticStop(self._native_device) 
        
    ## @inheritdoc  
    def HapticSetGain(self, gain : int):       
        CybSDK_dll.CybSDK_VirtDevice_HapticSetGain(self._native_device, c_int(gain)) 
        
    ## @inheritdoc  
    def HapticSetFrequency(self, frequency : int):    
        CybSDK_dll.CybSDK_VirtDevice_HapticSetFrequency(self._native_device, c_int(frequency)) 
        
    ## @inheritdoc  
    def HapticSetVolume(self, volume : int):     
        CybSDK_dll.CybSDK_VirtDevice_HapticSetVolume(self._native_device, c_int(volume)) 


