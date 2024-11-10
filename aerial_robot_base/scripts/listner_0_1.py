#
# Dynamixel 2 Library v3.0
#                                       Last Edit '23 11/06
#   Copyright (c) 2005, 2023 BestTechnology CO.,LTD.
#

#---------------------------------------------
# dx2lib wrapper
#---------------------------------------------
from ctypes import *
from sys import maxsize
from sysconfig import get_platform

__version__ = "3.0.0.0"

if ('mingw' in get_platform()) or ('win' in get_platform()):
  if (maxsize > 2**32) == True:
    _dx2lib = WinDLL('./dx2lib_x64.dll')
  else:
    _dx2lib = WinDLL('./dx2lib_x32.dll')
else:
  _dx2lib = CDLL('./dx2lib.so.3.0')

_dx2lib.DX2_OpenPort.argtypes = [c_char_p, c_uint32]
_dx2lib.DX2_OpenPort.restype  = c_void_p
def DX2_OpenPort(name, baud):
  return _dx2lib.DX2_OpenPort(name.encode('utf-8'),baud)

_dx2lib.DX2_ClosePort.argtypes = [c_void_p]
_dx2lib.DX2_ClosePort.restype   = c_bool
def DX2_ClosePort(dvid):
  return _dx2lib.DX2_ClosePort(dvid)

_dx2lib.DX2_SetBaudrate.argtypes = [c_void_p, c_ulong]
_dx2lib.DX2_SetBaudrate.restype   = c_bool
def DX2_SetBaudrate(dvid,baud):
  return _dx2lib.DX2_SetBaudrate(dvid,baud)

_dx2lib.DX2_Active.argtypes = [c_void_p]
_dx2lib.DX2_Active.restype   = c_bool
def DX2_Active(dvid):
  return _dx2lib.DX2_Active(dvid)

_dx2lib.DX2_SetTimeOutOffset.argtypes = [c_void_p, c_uint32]
_dx2lib.DX2_SetTimeOutOffset.restype   =  (None)
def DX2_SetTimeOutOffset(dvid,offsettime):
  return _dx2lib.DX2_SetTimeOutOffset(dvid,offsettime)

_dx2lib.GetQueryPerformanceCounter.argtypes = (None)
_dx2lib.GetQueryPerformanceCounter.restype = c_double
def GetQueryPerformanceCounter():
  return _dx2lib.GetQueryPerformanceCounter()

_dx2lib.DX2_TxPacket.argtypes = [c_void_p, c_uint8, c_uint8, POINTER(c_uint8), c_uint32, POINTER(c_uint16)]
_dx2lib.DX2_TxPacket.restype  = c_bool
def DX2_TxPacket(dvid,id,inst,param,len,err):
  return _dx2lib.DX2_TxPacket(dvid,id,inst,param,len,err)

_dx2lib.DX2_RxPacket.argtypes = [c_void_p, POINTER(c_uint8), c_uint32, POINTER(c_uint32), c_uint32, POINTER(c_uint16)]
_dx2lib.DX2_RxPacket.restype  = c_bool
def DX2_RxPacket(dvid,rdata,rdatasize,rlen,timeout,err):
  return _dx2lib.DX2_RxPacket(dvid,rdata,rdatasize,rlen,timeout,err)

_dx2lib.DX2_ReadByteData.argtypes = [c_void_p, c_uint8, c_uint16, POINTER(c_uint8), POINTER(c_uint16)]
_dx2lib.DX2_ReadByteData.restype  = c_bool
def DX2_ReadByteData(dvid,id,addr,rdat,err):
  return _dx2lib.DX2_ReadByteData(dvid,id,addr,rdat,err)

_dx2lib.DX2_WriteByteData.argtypes = [c_void_p, c_uint8, c_uint16, c_uint8, POINTER(c_uint16)]
_dx2lib.DX2_WriteByteData.restype  = c_bool
def DX2_WriteByteData(dvid,id,addr,wdat,err):
  return _dx2lib.DX2_WriteByteData(dvid,id,addr,wdat,err)

_dx2lib.DX2_ReadWordData.argtypes = [c_void_p, c_uint8, c_uint16, POINTER(c_uint16), POINTER(c_uint16)]
_dx2lib.DX2_ReadWordData.restype  = c_bool
def DX2_ReadWordData(dvid,id,addr,rdat,err):
  return _dx2lib.DX2_ReadWordData(dvid,id,addr,rdat,err)

_dx2lib.DX2_WriteWordData.argtypes = [c_void_p, c_uint8, c_uint16, c_uint16, POINTER(c_uint16)]
_dx2lib.DX2_WriteWordData.restype  = c_bool
def DX2_WriteWordData(dvid,id,addr,wdat,err):
  return _dx2lib.DX2_WriteWordData(dvid,id,addr,wdat,err)

_dx2lib.DX2_ReadLongData.argtypes = [c_void_p, c_uint8, c_uint16, POINTER(c_uint32), POINTER(c_uint16)]
_dx2lib.DX2_ReadLongData.restype  = c_bool
def DX2_ReadLongData(dvid,id,addr,rdat,err):
  return _dx2lib.DX2_ReadLongData(dvid,id,addr,rdat,err)

_dx2lib.DX2_WriteLongData.argtypes = [c_void_p, c_uint8, c_uint16, c_uint32, POINTER(c_uint16)]
_dx2lib.DX2_WriteLongData.restype  = c_bool
def DX2_WriteLongData(dvid,id,addr,wdat,err):
  return _dx2lib.DX2_WriteLongData(dvid,id,addr,wdat,err)

_dx2lib.DX2_ReadBlockData.argtypes = [c_void_p, c_uint8, c_ushort, POINTER(c_uint8), c_uint32, POINTER(c_uint16)]
_dx2lib.DX2_ReadBlockData.restype  = c_bool
def DX2_ReadBlockData(dvid,id,addr,rdat,len,err):
  return _dx2lib.DX2_ReadBlockData(dvid,id,addr,rdat,len,err)

_dx2lib.DX2_WriteBlockData.argtypes = [c_void_p, c_uint8, c_uint16, POINTER(c_uint8), c_uint32, POINTER(c_uint16)]
_dx2lib.DX2_WriteBlockData.restype  = c_bool
def DX2_WriteBlockData(dvid,id,addr,dat,len,err):
  return _dx2lib.DX2_WriteBlockData(dvid,id,addr,dat,len,err)

_dx2lib.DX2_Ping.argtypes = [c_void_p, c_uint8, POINTER(c_uint16)]
_dx2lib.DX2_Ping.restype  = c_bool
def DX2_Ping(dvid,id,err):
  return _dx2lib.DX2_Ping(dvid,id,err)

class TDx2AlarmStatus(Structure):
  _pack_ = 1
  _fields_ = [("id", c_uint8),("Status", c_uint16)]

_dx2lib.DX2_Ping2.argtypes = [c_void_p, POINTER(c_int32), POINTER(TDx2AlarmStatus), POINTER(c_uint16)]
_dx2lib.DX2_Ping2.restype  = c_bool
def DX2_Ping2(dvid,num,AlarmStatus,err):
  return _dx2lib.DX2_Ping2(dvid,num,AlarmStatus,err)

class TSyncReadParam(Structure):
  _pack_ = 1
  _fields_ = [("addr", c_uint16),("length", c_uint16),("ids", c_uint8 * 256)]

_dx2lib.DX2_ReadSyncData.argtypes = [c_void_p, TSyncReadParam, POINTER(c_uint32), POINTER(c_uint8), POINTER(c_uint16)]
_dx2lib.DX2_ReadSyncData.restype  = c_bool
def DX2_ReadSyncData(dvid,param,num,dat,err):
  return _dx2lib.DX2_ReadSyncData(dvid,param,num,dat,err)

_dx2lib.DX2_WriteSyncData.argtypes = [c_void_p, POINTER(c_uint8), c_uint32, POINTER(c_uint16)]
_dx2lib.DX2_WriteSyncData.restype  = c_bool
def DX2_WriteSyncData(dvid,dat,size,err):
  return _dx2lib.DX2_WriteSyncData(dvid,dat,size,err)

class TBulkReadParam(Structure):
  _pack_ = 1
  _fields_ = [("id", c_uint8),("addr", c_uint16),("length", c_uint16)]

class TBulkReadResult(Structure):
  _pack_ = 1
  _fields_ = [("size", c_uint16),("id", c_uint8),("err", c_uint16),("dat", POINTER(c_uint8))]

_dx2lib.DX2_ReadBulkData.argtypes = [c_void_p, POINTER(TBulkReadParam), POINTER(c_uint32), POINTER(c_uint8), POINTER(c_uint16)]
_dx2lib.DX2_ReadBulkData.restype  = c_bool
def DX2_ReadBulkData(dvid,param,num,dat,err):
  return _dx2lib.DX2_ReadBulkData(dvid,param,num,dat,err)

_dx2lib.DX2_WriteBulkData.argtypes = [c_void_p, POINTER(c_uint8), c_uint32, POINTER(c_uint16)]
_dx2lib.DX2_WriteBulkData.restype  = c_bool
def DX2_WriteBulkData(dvid,dat,size,err):
  return _dx2lib.DX2_WriteBulkData(dvid,dat,size,err)

_dx2lib.DX2_Reset.argtypes = [c_void_p, c_uint8, c_void_p]
_dx2lib.DX2_Reset.restype  = c_bool
def DX2_Reset(dvid,id,err):
  return _dx2lib.DX2_Reset(dvid,id,err)

_dx2lib.DX2_Reboot.argtypes = [c_void_p, c_uint8, c_void_p]
_dx2lib.DX2_Reboot.restype  = c_bool
def DX2_Reboot(dvid,id,err):
  return _dx2lib.DX2_Reboot(dvid,id,err)

_dx2lib.DX2_EnableSuffixRemoval.argtypes = [c_bool]
_dx2lib.DX2_EnableSuffixRemoval.restype  = None
def DX2_EnableSuffixRemoval(en):
  _dx2lib.DX2_EnableSuffixRemoval(en)

(devtNONE, devtDX, devtAX, devtRX, devtEX, devtMX, devtXL320, devtPRO, devtX) = range(9)

class TMaxMin_int32(Structure):
  _pack_ = 1
  _fields_ = [("max", c_int32),("min", c_int32)]
class TMaxMin_dbl(Structure):
  _pack_ = 1
  _fields_ = [("max", c_double),("min", c_double)]

class TDXL_ModelInfo(Structure):
  _pack_ = 1
  _fields_ = [
    ("modelno",       c_uint16),
    ("name",          c_char * 16),
    ("devtype",       c_uint32),
    ("positionlimit", TMaxMin_int32),
    ("anglelimit",    TMaxMin_dbl),
    ("velocitylimit", TMaxMin_int32),
    ("pwmlimit",      TMaxMin_int32),
    ("velocityratio",      c_double),
    ("currentratio",  c_double),
    ("pwmratio",      c_double)]

class TAngleVelocity(Structure):
  _fields_ = [
    ("angle", c_double),
    ("velocity", c_double)]

# DXL LED
#----------------------------
_dx2lib.DXL_SetLED.argtypes = [c_void_p, c_uint8, c_bool]
_dx2lib.DXL_SetLED.restype  = c_bool
def DXL_SetLED(dvid,id,en):
  return _dx2lib.DXL_SetLED(dvid,id,en)

# DXL TorqueEnable
#----------------------------
_dx2lib.DXL_SetTorqueEnable.argtypes = [c_void_p, c_uint8, c_bool]
_dx2lib.DXL_SetTorqueEnable.restype  = c_bool
def DXL_SetTorqueEnable(dvid,id,en):
  return _dx2lib.DXL_SetTorqueEnable(dvid,id,en)

_dx2lib.DXL_SetTorqueEnables.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_bool), c_int]
_dx2lib.DXL_SetTorqueEnables.restype  = c_bool
def DXL_SetTorqueEnables(dvid,ids,en,num):
  return _dx2lib.DXL_SetTorqueEnables(dvid,ids,en,num)

_dx2lib.DXL_SetTorqueEnablesEquival.argtypes = [c_void_p, POINTER(c_uint8), c_int, c_bool]
_dx2lib.DXL_SetTorqueEnablesEquival.restype  = c_bool
def DXL_SetTorqueEnablesEquival(dvid,ids,num,en):
  return _dx2lib.DXL_SetTorqueEnablesEquival(dvid,ids,num,en)

_dx2lib.DXL_GetTorqueEnable.argtypes = [c_void_p, c_uint8, POINTER(c_bool)]
_dx2lib.DXL_GetTorqueEnable.restype  = c_bool
def DXL_GetPresentAngle(dvid,id,en):
  return _dx2lib.DXL_GetTorqueEnable(dvid,id,en)

_dx2lib.DXL_GetTorqueEnables.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_bool), c_int]
_dx2lib.DXL_GetTorqueEnables.restype  = c_bool
def DXL_GetTorqueEnables(dvid,ids,en,num):
  return _dx2lib.DXL_GetTorqueEnables(dvid,ids,en,num)

# DXL Angle
#----------------------------
_dx2lib.DXL_SetGoalAngle.argtypes = [c_void_p, c_uint8, c_double]
_dx2lib.DXL_SetGoalAngle.restype  = c_bool
def DXL_SetGoalAngle(dvid,id,angle):
  return _dx2lib.DXL_SetGoalAngle(dvid,id,angle)

_dx2lib.DXL_SetGoalAngles.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dx2lib.DXL_SetGoalAngles.restype  = c_bool
def DXL_SetGoalAngles(dvid,ids,angles,num):
  return _dx2lib.DXL_SetGoalAngles(dvid,ids,angles,num)

_dx2lib.DXL_GetPresentAngle.argtypes = [c_void_p, c_uint8, POINTER(c_double)]
_dx2lib.DXL_GetPresentAngle.restype  = c_bool
def DXL_GetPresentAngle(dvid,id,angle):
  return _dx2lib.DXL_GetPresentAngle(dvid,id,angle)

_dx2lib.DXL_GetPresentAngles.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dx2lib.DXL_GetPresentAngles.restype  = c_bool
def DXL_GetPresentAngles(dvid,ids,angles,num):
  return _dx2lib.DXL_GetPresentAngles(dvid,ids,angles,num)

# DXL Standstill
#----------------------------
_dx2lib.DXL_StandStillAngle.argtypes = [c_void_p, c_uint8]
_dx2lib.DXL_StandStillAngle.restype  = c_bool
def DXL_StandStillAngle(dvid,id):
  return _dx2lib.DXL_StandStillAngle(dvid,id)

_dx2lib.DXL_StandStillAngles.argtypes = [c_void_p, POINTER(c_uint8), c_int]
_dx2lib.DXL_StandStillAngles.restype  = c_bool
def DXL_StandStillAngles(dvid,ids,num):
  return _dx2lib.DXL_StandStillAngles(dvid,ids,num)

# DXL Velocity
#----------------------------
_dx2lib.DXL_SetGoalVelocity.argtypes = [c_void_p, c_uint8, c_double]
_dx2lib.DXL_SetGoalVelocity.restype  = c_bool
def DXL_SetGoalVelocity(dvid,id,velocity):
  return _dx2lib.DXL_SetGoalVelocity(dvid,id,velocity)

_dx2lib.DXL_SetGoalVelocities.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dx2lib.DXL_SetGoalVelocities.restype  = c_bool
def DXL_SetGoalVelocities(dvid,ids,velocities,num):
  return _dx2lib.DXL_SetGoalVelocities(dvid,ids,velocities,num)

_dx2lib.DXL_GetPresentVelocity.argtypes = [c_void_p, c_uint8, POINTER(c_double)]
_dx2lib.DXL_GetPresentVelocity.restype  = c_bool
def DXL_GetPresentVelocity(dvid,id,velocity):
  return _dx2lib.DXL_GetPresentVelocity(dvid,id,velocity)

_dx2lib.DXL_GetPresentVelocities.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dx2lib.DXL_GetPresentVelocities.restype  = c_bool
def DXL_GetPresentVelocities(dvid,ids,velocities,num):
  return _dx2lib.DXL_GetPresentVelocities(dvid,ids,velocities,num)

# DXL Angle & Velocity
#----------------------------
_dx2lib.DXL_SetGoalAngleAndVelocity.argtypes = [c_void_p, c_uint8, c_double, c_double]
_dx2lib.DXL_SetGoalAngleAndVelocity.restype  = c_bool
def DXL_SetGoalAngleAndVelocity(dvid,id,angle,velocity):
  return _dx2lib.DXL_SetGoalAngleAndVelocity(dvid,id,angle,velocity)

_dx2lib.DXL_SetGoalAnglesAndVelocities.argtypes = [c_void_p, POINTER(c_uint8), POINTER(TAngleVelocity), c_int]
_dx2lib.DXL_SetGoalAnglesAndVelocities.restype  = c_bool
def DXL_SetGoalAnglesAndVelocities(dvid,ids,anglevelocity,num):
  return _dx2lib.DXL_SetGoalAnglesAndVelocities(dvid,ids,anglevelocity,num)

# DXL Angle & Time
#----------------------------
_dx2lib.DXL_SetGoalAngleAndTime.argtypes = [c_void_p, c_uint8, c_double, c_double]
_dx2lib.DXL_SetGoalAngleAndTime.restype  = c_bool
def DXL_SetGoalAngleAndTime(dvid,id,angle,sec):
  return _dx2lib.DXL_SetGoalAngleAndTime(dvid,id,angle,sec)

_dx2lib.DXL_SetGoalAnglesAndTime.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int, c_double]
_dx2lib.DXL_SetGoalAnglesAndTime.restype  = c_bool
def DXL_SetGoalAnglesAndTime(dvid,ids,angles,num,sec):
  return _dx2lib.DXL_SetGoalAnglesAndTime(dvid,ids,angles,num,sec)

_dx2lib.DXL_SetGoalAngleAndTime2.argtypes = [c_void_p, c_uint8, c_double, c_double]
_dx2lib.DXL_SetGoalAngleAndTime2.restype  = c_bool
def DXL_SetGoalAngleAndTime2(dvid,id,angle,sec):
  return _dx2lib.DXL_SetGoalAngleAndTime2(dvid,id,angle,sec)

_dx2lib.DXL_SetGoalAnglesAndTime2.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int, c_double]
_dx2lib.DXL_SetGoalAnglesAndTime2.restype  = c_bool
def DXL_SetGoalAnglesAndTime2(dvid,ids,angles,num,sec):
  return _dx2lib.DXL_SetGoalAnglesAndTime2(dvid,ids,angles,num,sec)

# DXL Current
#----------------------------
_dx2lib.DXL_SetGoalCurrent.argtypes = [c_void_p, c_uint8, c_double]
_dx2lib.DXL_SetGoalCurrent.restype  = c_bool
def DXL_SetGoalCurrent(dvid,id,current):
  return _dx2lib.DXL_SetGoalCurrent(dvid,id,current)

_dx2lib.DXL_SetGoalCurrents.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dx2lib.DXL_SetGoalCurrents.restype  = c_bool
def DXL_SetGoalCurrents(dvid,ids,currents,num):
  return _dx2lib.DXL_SetGoalCurrents(dvid,ids,currents,num)

_dx2lib.DXL_GetPresentCurrent.argtypes = [c_void_p, c_uint8, POINTER(c_double)]
_dx2lib.DXL_GetPresentCurrent.restype  = c_bool
def DXL_GetPresentCurrent(dvid,id,current):
  return _dx2lib.DXL_GetPresentCurrent(dvid,id,current)

_dx2lib.DXL_GetPresentCurrents.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dx2lib.DXL_GetPresentCurrents.restype  = c_bool
def DXL_GetPresentCurrents(dvid,ids,currents,num):
  return _dx2lib.DXL_GetPresentCurrents(dvid,ids,currents,num)

# DXL PWM
#----------------------------
_dx2lib.DXL_SetGoalPWM.argtypes = [c_void_p, c_uint8, c_double]
_dx2lib.DXL_SetGoalPWM.restype  = c_bool
def DXL_SetGoalPWM(dvid,id,pwm):
  return _dx2lib.DXL_SetGoalPWM(dvid,id,pwm)

_dx2lib.DXL_SetGoalPWMs.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dx2lib.DXL_SetGoalPWMs.restype  = c_bool
def DXL_SetGoalPWMs(dvid,ids,pwms,num):
  return _dx2lib.DXL_SetGoalPWMs(dvid,ids,pwms,num)

_dx2lib.DXL_GetPresentPWM.argtypes = [c_void_p, c_uint8, POINTER(c_double)]
_dx2lib.DXL_GetPresentPWM.restype  = c_bool
def DXL_GetPresentPWM(dvid,id,pwm):
  return _dx2lib.DXL_GetPresentPWM(dvid,id,pwm)

_dx2lib.DXL_GetPresentPWMs.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dx2lib.DXL_GetPresentPWMs.restype  = c_bool
def DXL_GetPresentPWMs(dvid,ids,pwms,num):
  return _dx2lib.DXL_GetPresentPWMs(dvid,ids,pwms,num)

# DXL Drive Mode
#----------------------------
_dx2lib.DXL_SetDriveMode.argtypes = [c_void_p, c_uint8, c_uint8]
_dx2lib.DXL_SetDriveMode.restype  = c_bool
def DXL_SetDriveMode(dvid,id,mode):
  return _dx2lib.DXL_SetDriveMode(dvid,id,mode)

_dx2lib.DXL_SetDriveModesEquival.argtypes = [c_void_p, POINTER(c_uint8), c_int, c_uint8]
_dx2lib.DXL_SetDriveModesEquival.restype  = c_bool
def DXL_SetDriveModesEquival(dvid,ids,num,mode):
  return _dx2lib.DXL_SetDriveModesEquival(dvid,ids,num,mode)

# DXL OP Mode
#----------------------------
_dx2lib.DXL_SetOperatingMode.argtypes = [c_void_p, c_uint8, c_uint8]
_dx2lib.DXL_SetOperatingMode.restype  = c_bool
def DXL_SetOperatingMode(dvid,id,mode):
  return _dx2lib.DXL_SetOperatingMode(dvid,id,mode)

_dx2lib.DXL_SetOperatingModesEquival.argtypes = [c_void_p, POINTER(c_uint8), c_int, c_uint8]
_dx2lib.DXL_SetOperatingModesEquival.restype  = c_bool
def DXL_SetOperatingModesEquival(dvid,ids,num,mode):
  return _dx2lib.DXL_SetOperatingModesEquival(dvid,ids,num,mode)

_dx2lib.DXL_GetOperatingMode.argtypes = [c_void_p, c_uint8, POINTER(c_uint8)]
_dx2lib.DXL_GetOperatingMode.restype  = c_bool
def DXL_GetOperatingMode(dvid,id,mode):
  return _dx2lib.DXL_GetOperatingMode(dvid,id,mode)

# DXL Error
#----------------------------
_dx2lib.DXL_GetErrorCode.argtypes = [c_void_p, c_uint8]
_dx2lib.DXL_GetErrorCode.restype  = c_uint16
def DXL_GetErrorCode(dvid,id):
  return _dx2lib.DXL_GetErrorCode(dvid,id)

_dx2lib.DXL_GetHWErrorCode.argtypes = [c_void_p, c_uint8, POINTER(c_uint8)]
_dx2lib.DXL_GetHWErrorCode.restype  = c_bool
def DXL_GetHWErrorCode(dvid,id,hwerr):
  return _dx2lib.DXL_GetHWErrorCode(dvid,id,hwerr)

# DXL Model Info
#----------------------------
_dx2lib.DXL_GetModelInfo.argtypes = [c_void_p, c_uint8]
_dx2lib.DXL_GetModelInfo.restype  = POINTER(TDXL_ModelInfo)
def DXL_GetModelInfo(dvid,id):
  return _dx2lib.DXL_GetModelInfo(dvid,id)

_dx2lib.DXL_ScanDevices.argtypes = [c_void_p, POINTER(c_uint8)]
_dx2lib.DXL_ScanDevices.restype  = c_int
def DXL_ScanDevices(dvid,ids):
  return _dx2lib.DXL_ScanDevices(dvid,ids)

_dx2lib.DXL_PrintDevicesList.argtypes = [c_void_p]
_dx2lib.DXL_PrintDevicesList.restype  = c_bool
def DXL_PrintDevicesList(f):
  return _dx2lib.DXL_PrintDevicesList(f)

_dx2lib.DXL_InitDevicesList.argtypes = (None)
_dx2lib.DXL_InitDevicesList.restype  = (None)
def DXL_InitDevicesList():
  _dx2lib.DXL_InitDevicesList()

_dx2lib.DXL_GetModelInfoByModelNo.argtypes = [c_uint16]
_dx2lib.DXL_GetModelInfoByModelNo.restype  = POINTER(TDXL_ModelInfo)
def DXL_GetModelInfoByModelNo(modelno):
  return _dx2lib.DXL_GetModelInfoByModelNo(modelno)

