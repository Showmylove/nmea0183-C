#ifndef __PARSE_NMEA_H__
#define __PARSE_NMEA_H__

#include "CBasicTypeDefine.h"

#define GPS_NMEA_TEST

typedef enum {false = 0, true = !false} boolean;

typedef struct {
	u8 year;
	u8 month;
	u8 day;
}Date;

typedef struct {
	u8 hour;
	u8 minute;
	u8 second;
}Time;

typedef struct {
	Date date;
	Time time;
}DateTime;

typedef enum {
	LATITUDE_NORTH,
	LATITUDE_SOUTH,
}LatRegion;

typedef enum {
	LONGITUDE_EAST,
	LONGITUDE_WEST,
}LonRegion;

typedef struct {
	u32 lat;				/* Unit: Milli-dd. */
	LatRegion latRegion;	/* LAT_NORTH: North. LAT_SOUTH: South. */
	u32 lon;				/* Unit: Milli-dd. */
	LonRegion lonRegion;	/* LON_EAST: East. LON_WEST: West. */
}PositionInfo;

typedef struct {
	float speed;	/* Unit: km/h. */
	float altitude;	/* Unit: meter. */
	float course;	/* Unit: degree. */
}PointInfo;

typedef enum {
	LOC_INFO_INVALID,
	LOC_INFO_VALID,
}DataValidity;

typedef enum {
	PCAK_TYPE_NONE,
	PCAK_TYPE_GLL,
	PCAK_TYPE_VTG,
	PCAK_TYPE_RMC,
	PCAK_TYPE_GSV,
	PCAK_TYPE_GSA,
	PCAK_TYPE_GGA,
}PackType;

typedef enum {
	FIX_INVALID,
	FIX_2D,
	FIX_3D,
}FixStatus;

typedef struct {
	DataValidity dataValidity;
	PackType packType;
	FixStatus fixStatus;
	u8 satelliteNum;		/*定位星数*/
}LocationCommonPara;

typedef struct {
	u8 SNR;				/*GPS信噪比*/
	float HDOP;
	float VDOP;
	float PDOP;
}LocationPerformancePara;

typedef struct {
	PositionInfo positionInfo;
	PointInfo	pointInfo;
	DateTime dataTime;
	LocationCommonPara locationCommonPara;
	LocationPerformancePara locationPerformancePara;
}LocationVectorInfo;

boolean nmeaInit(LocationVectorInfo * localInfo);
void nmeaMultiDataRecv(u8 * u8Buff, u16 u16Len);
boolean nmeaAnalysis(u8 * u8Buff, u16 u16Len);

#endif

