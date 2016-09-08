#ifndef __NMEA0183_PORT_H__
#define __NMEA0183_PORT_H__

void nmeaInitHook(void);
void nmeaPackCaptured(u8 * u8Buff, u16 u16Len);
void nmeaLocationInfoUpdateNotice(LocationVectorInfo * localInfo);
void nmeaOSSchedLock(void);
void nmeaOSSchedUnlock(void);
void nmeaDebugString(u8 *str);
void nmeaDebugPrintf(u8 *str, u16 u16Len);

#endif
