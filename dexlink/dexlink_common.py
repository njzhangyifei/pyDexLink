"""
File: dexlink_common.py
Author: Yifei Zhang
Email: njzhangyifei@gmail.com
Github: https://github.com/njzhangyifei
Description: Contains the definitions for common stuff used in dexlink module
"""


class DexLinkException(Exception):
    pass


class DexLinkSerialException(DexLinkException):
    pass


class DexLinkTimeoutException(DexLinkException):
    pass


class DexLinkInvalidOperationException(DexLinkException):
    pass
