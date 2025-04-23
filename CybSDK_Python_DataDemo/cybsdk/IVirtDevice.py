# coding: utf-8

from abc import ABC, abstractmethod

from ctypes import *
from . import CybSDK_dll
from .VirtDeviceInfo import VirtDeviceInfo

## @brief Main interface all Virtualizer Devices have to implement.
class IVirtDevice(ABC):
    """ Main interface all Virtualizer Devices have to implement. """

    ## @brief Opens the connection to the Virtualizer device.
    #  @remark No other application can open this device at the same time.
    #  @return true if the connection was successfully opened, otherwise false.
    @abstractmethod
    def Open(self) -> bool:    
        """ Opens the connection to the Virtualizer device. """
        pass

    ## @brief Checks if the connection was opened before. 
    @abstractmethod
    def IsOpen(self) -> bool:    
        """ Checks if the connection was opened before. """
        pass

    ## @brief Closes the connection to the Virtualizer device. 
    @abstractmethod
    def Close(self) -> bool:    
        """ Closes the connection to the Virtualizer device. """
        pass


    ## @brief Returns the USB infos of this device. 
    @abstractmethod
    def GetDeviceInfo(self) -> VirtDeviceInfo:    
        """ Returns the USB infos of this device. """  
        pass


    ## @brief Returns the current player height relative to the default height.
    #  @remark The default height is set by the ResetPlayerHeight method.
    #  @remark height < -threshold: crouching
    #  @remark height >  threshold: jumping
    #  @return 1.00f = 1cm.
    @abstractmethod
    def GetPlayerHeight(self) -> float:   
        """ Returns the current player height relative to the default height. """          
        pass

    ## @brief Assigns the current height to the default height.
    #  @remark This method should be called while the player is asked to stand upright.
    @abstractmethod
    def ResetPlayerHeight(self):   
        """ Assigns the current height to the default height. """         
        pass

    ## @brief Returns the orientation of the player as an absolute value.
    #  @remark The origin is set by the ResetPlayerOrientation method and increases clockwise.
    #  @return logical: 0.00f to 1.00f (= physical: 0° to 360°).
    @abstractmethod
    def GetPlayerOrientation(self) -> float:            
        """ Returns the orientation of the player as an absolute value. """ 
        pass

    ## @brief Assigns the current orientation to the default vector.
    #  @remark This method should be called while the player is asked to look forward.
    #  @remark This orientation should be used to calibrate the HMD.
	#  @deprecated Unused as new Virtualizers orient themselves automatically.
    @abstractmethod
    def ResetPlayerOrientation(self):            
        """ Assigns the current orientation to the default vector. """  
        pass

    ## @brief Returns the current movement speed in meters per second.
    #  @return 1.00f = 1m/s
    @abstractmethod
    def GetMovementSpeed(self) -> float:   
        """ Returns the current movement speed in meters per second. """          
        pass

    ## @brief Returns the movement direction relative to the current player orientation.
    #  @remark The origin is the GetPlayerOrientation method and increases clockwise.
    #  @return logical: -1.00f to 1.00f (= physical: -180° to 180°).
    @abstractmethod
    def GetMovementDirection(self) -> float:      
        """ Returns the movement direction relative to the current player orientation. """         
        pass


    ## @brief Checks if the Virtualizer device supports haptic feedback.   
    @abstractmethod
    def HasHaptic(self) -> bool:       
        """ Checks if the Virtualizer device supports haptic feedback. """   
        pass
    
    ## @brief Play a signal on the haptic unit. 
    @abstractmethod
    def HapticPlay(self):    
        """ Play a signal on the haptic unit.  """
        pass
        
    ## @brief Stop the haptic unit. 
    @abstractmethod
    def HapticStop(self):    
        """ Stop the haptic unit.  """
        pass
        
    ## @brief Set the gain (dB) level of the haptic unit.
    #  @param gain The value can be 0, 1, 2 or 3.
    @abstractmethod
    def HapticSetGain(self, gain : int):            
        """ Set the gain (dB) level of the haptic unit. """
        pass
        
    ## @brief Set the frequency (Hz) of a sine wave on the haptic unit.
    #  @param frequency The value is valid between 0Hz and 80Hz.
    @abstractmethod
    def HapticSetFrequency(self, frequency : int):   
        """ Set the frequency (Hz) of a sine wave on the haptic unit. """         
        pass
        
    ## @brief Sets the haptic feedback (change of amplitude) in the baseplate.
    # @param volume The value is valid between 0 (no feedback) and 100 (full feedback).
    @abstractmethod
    def HapticSetVolume(self, volume : int):          
        """ Sets the haptic feedback (change of amplitude) in the baseplate. """          
        pass


