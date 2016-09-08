#include "CBasicTypeDefine.h"
#include "parseNMEA.h"

void nmeaInitHook(void)
{
	;
}

void nmeaPackCaptured(u8 * u8Buff, u16 u16Len)
{
	;
}

#define UNSIGNED_VAL_MIN	(0x00)
#define U8_VAL_MAX			(0xFF)
#define U16_VAL_MAX			(0xFFFF)
void nmeaLocationInfoUpdateNotice(LocationVectorInfo * localInfo)
{
	;
}

void nmeaOSSchedLock(void)
{
	;
}

void nmeaOSSchedUnlock(void)
{
	;
}

void nmeaDebugString(u8 *str)
{
	;
}

void nmeaDebugPrintf(u8 *str, u16 u16Len)
{
	;
}

