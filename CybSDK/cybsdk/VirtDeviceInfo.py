# coding: utf-8

from ctypes import *

## @brief Struct containing all infos about a Virtualizer USB device.
class VirtDeviceInfo(Structure):
    """ Struct containing all infos about a Virtualizer USB device. """
    _fields_ = [("MajorVersion", c_ubyte),
                ("MinorVersion", c_ubyte),
                ("VendorId", c_ushort),
                ("VendorName", c_wchar_p),
                ("ProductId", c_ushort),
                ("ProductName", c_wchar_p),
                ("__devicePath", c_char_p)]