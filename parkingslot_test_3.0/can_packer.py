'''
Copyright (C) 2020 Yirami.
All rights reserved.

@Brief: Wrapper for library `can_packer`
@Author: [yirami.tang](https://yirami.xyz)
@Email: i@yirami.xyz
@Since: 2020-11-14 19:56:27
@LastEditors: Yirami
@LastEditTime: 2020-11-20 13:59:40
@Version: 
'''
import os
import ctypes
from enum import Enum, unique

@unique
class PackStatus(Enum):
    SUCCESS = 0
    INCOMPLETE = 1
    DBC_ERROR = 2
    MESSAGE_NOT_EXIST = 3
    SIGNAL_NOT_EXIST = 4

class CSignalsPackage(ctypes.Structure):
    _fields_ = [("num", ctypes.c_int),
                ("name", ctypes.c_char_p),
                ("value", ctypes.c_double)]

class CanPacker:
    def __init__(self, lib_path:str, dbc_path:str=None):
        self._lib = ctypes.cdll.LoadLibrary(os.path.join(lib_path, 'libcan_packer.so'))
        self._dbc_stat = None
        if dbc_path:
            self.load_dbc(dbc_path)

    def load_dbc(self, dbc_path:str):
        self._dbc_stat = PackStatus(self._lib.Init(bytes(dbc_path, encoding='utf8')))

    def dbc_version(self):
        if self._dbc_stat is None:
            print("[YLog/Error]pycanpacker: DBC not loaded!")
            return
        elif self._dbc_stat != PackStatus.SUCCESS:
            print("[YLog/Error]pycanpacker: DBC load failure!")
            return
        self._lib.DBCVersion.restype = ctypes.c_char_p
        return self._lib.DBCVersion().decode('utf-8')

    def pack(self, id:int, sigs:dict, size:int, ignore_err=True):
        csigs = (CSignalsPackage*len(sigs))()
        for idx, kk in enumerate(sigs):
            csigs[idx].num = len(sigs)
            csigs[idx].name = bytes(kk, encoding='utf8')
            csigs[idx].value = ctypes.c_double(sigs[kk])
        csigs_p = ctypes.POINTER(CSignalsPackage)(csigs)
        raw = bytes(size)
        craw = (ctypes.c_ubyte*size).from_buffer(bytearray(raw))
        craw_p = ctypes.POINTER(ctypes.c_ubyte)(craw)
        if ignore_err:
            err_cnt = 0
            cerr_cnt = ctypes.c_int(err_cnt)
            cerr_cnt_p = ctypes.POINTER(ctypes.c_int)(cerr_cnt)
            stat = PackStatus(self._lib.Pack(ctypes.c_int(id), csigs_p, craw_p, ctypes.c_bool(True), cerr_cnt_p))
            return stat, ctypes.string_at(craw, size).hex()
        else:
            stat = PackStatus(self._lib.Pack(ctypes.c_int(id), csigs_p, craw_p, ctypes.c_bool(False)))
            return stat, ctypes.string_at(craw, size).hex()

    def unpack(self, id:int, raw:bytes, name:str=None):
        if isinstance(raw, bytes):
            craw = (ctypes.c_ubyte*len(raw)).from_buffer(bytearray(raw))
            craw_p = ctypes.POINTER(ctypes.c_ubyte)(craw)
            if name:
                csig = ctypes.c_double(0)
                csig_p = ctypes.POINTER(ctypes.c_double)(csig)
                stat = PackStatus(self._lib.UnPackSignal(ctypes.c_int(id), craw_p, bytes(name, encoding='utf8'), csig_p))
                return stat, csig.value
            else:
                num = self._lib.SignalsNum(ctypes.c_int(id))
                csigs = (CSignalsPackage*num)()
                for ii in range(num):
                    csigs[ii].num = 0
                    csigs[ii].name = ctypes.cast(ctypes.create_string_buffer(200), ctypes.c_char_p)
                    csigs[ii].value = ctypes.c_double(0)
                csigs_p = ctypes.POINTER(CSignalsPackage)(csigs)
                stat = PackStatus(self._lib.UnPack(ctypes.c_int(id), craw_p, csigs_p))
                sigs = {}
                for ii in range(num):
                    sigs[csigs[ii].name.decode('utf-8')] = float(csigs[ii].value)
                return stat, sigs
        elif isinstance(raw, str):
            return self.unpack(id, bytes(bytearray.fromhex(raw)), name)
        else:
            raise NotImplementedError
