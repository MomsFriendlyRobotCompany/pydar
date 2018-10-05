# https://stackoverflow.com/questions/5081875/ctypes-beginner
# https://docs.python-guide.org/scenarios/clibs/
# https://cvstuff.wordpress.com/2014/11/27/wraping-c-code-with-python-ctypes-memory-and-pointers/
# https://stackoverflow.com/questions/42585210/extending-setuptools-extension-to-use-cmake-in-setup-py
"""
https://en.wikibooks.org/wiki/Python_Programming/Extending_with_ctypes
None    void    None    the None object
c_bool    C99 _Bool    bool
c_byte    signed char    int
c_char    signed char    str    length of one
c_char_p    char *    str
c_double    double    float
c_float    float    float
c_int    signed int    int
c_long    signed long    long
c_longlong    signed long long    long
c_short    signed short    long
c_ubyte    unsigned char    int
c_uint    unsigned int    int
c_ulong    unsigned long    long
c_ulonglong    unsigned long long    long
c_ushort    unsigned short    int
c_void_p    void *    int
c_wchar    wchar_t    unicode    length of one
c_wchar_p    wchar_t *    unicode
"""

import os.path
import ctypes
import numpy.ctypeslib as ctl
import numpy as np

from ctypes import Structure

class ScanData(Structure):
    _fields_ = [
        ("len", c_int),
        ("data", (c_double * 2) * 360)
    ]

me = os.path.abspath(os.path.dirname(__file__))
libpath = os.path.join(me, "..", "libtest.so")

libname = 'pydarlib.so'
libdir = './'
pydarlib = ctypes.CDLL(libdir + libname)
# lib=ctl.load_library(libname, libdir)

# who owns the array and clears it?
# better to pass an array to the function, then python owns it
pydarlib = ctypes.CDLL(libdir + libname).function
pydarlib.restype = ctypes.POINTER(ctypes.c_float*2*360)


# http://www.ifnamemain.com/posts/2013/Dec/10/c_structs_python/
pydarlib = ctypes.cdll.LoadLibrary(libdir + libname)
pydarlib.get.restype = None
# scan =  np.array([(0,0) for _ in range(360)])
# data = ScanData(len(scan), np.cytpeslib.as_ctypes(scan))

class YDLidar(class):
    def __init__(self):
        self.scan =  np.array([(0,0) for _ in range(360)])
        self.data = ScanData(len(self.scan), np.cytpeslib.as_ctypes(self.scan))

    def get(self):
        pydarlib.get(ctypes.byref(self.data))
        return self.data
