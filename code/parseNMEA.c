#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "parseNMEA.h"
#include "nmea0183Port.h"

#define IS_TRUE_BIT(X)		(0 != (X))
#define IS_FALSE_BIT(X)		(0 == (X))
#define M_SetBit(X)			((X) = 1)
#define M_ClrBit(X)			((X) = 0)
#define M_MIN(X,Y)			((X) < (Y) ? (X) : (Y))
#define M_MAX(X,Y)			((X) > (Y) ? (X) : (Y))
#define M_NOT_ZERO_DEC(X)	{if(0 != (X)) {(X)--;} else {/*No action*/}}
#define IS_NULL_P(X)			(NULL == (X))

static LocationVectorInfo locationInfo;
static LocationVectorInfo * pLocationInfo = NULL;

#define GPS_RX_BUFF_SIZE	(88)

#define NMEA_FRAME_START_SIGN			('$')
#define NMEA_FRAME_CHECK_PREFIX_SIGN		('*')
#define NMEA_FRAME_SEPARATOR_SIGN		(',')
#define NMEA_FRAME_POINT_SIGN				('.')
#define NMEA_FRAME_END_SIGN_1				('\r')
#define NMEA_FRAME_END_SIGN_2				('\n')

#define NMEA_FRAME_START_SIGN_LEN			(1)
#define NMEA_FRAME_SEPARATOR_SIGN_LEN		(1)
#define NMEA_FRAME_CHECK_PREFIX_SIGN_LEN	(1)

typedef enum {
	GPS_DATA_RX_ST_IDLE,
	GPS_DATA_RX_ST_RXING,
	GPS_DATA_RX_ST_VERIFY_1,
	GPS_DATA_RX_ST_VERIFY_2,
	GPS_DATA_RX_ST_END_1,
	GPS_DATA_RX_ST_END_2,
}enGpsDataRxSt;

#define INDEX_2_INT_LEN_MAX	(8)
static u32 Index2Int(u8 u8Len)
{
	u32 u32RetVal = 1;

	if(u8Len > INDEX_2_INT_LEN_MAX) {
		;
	}
	else {
		for(u8 u8Cnt=0; u8Cnt<u8Len; u8Cnt++) {
			u32RetVal = u32RetVal * 10;
		}
	}

	return u32RetVal;
}

static s8 ASCII2Hex(u8 isrc)
{
	s8 itemp = 0;

	itemp = isrc;
	if((itemp >= '0') && (itemp <= '9')) {
		itemp -= '0';
	}
	else if((itemp >= 'A') && (itemp <= 'F')) {
		itemp -= 'A';
		itemp += 0x0a;
	}
	else if((itemp >= 'a') && (itemp <= 'f')) {
		itemp -= 'a';
		itemp += 0x0a;
	}
	else {
		itemp = (s8)-1;
	}

	return itemp;
}

static boolean ASCII2Byte(const u8 * psrc, u8 * pdst)
{
	boolean bRetVal = false;
	signed char itemph = 0;
	signed char itempl = 0;

	itemph = ASCII2Hex(*psrc);
	itempl = ASCII2Hex(*(psrc+1));

	if((-1 == itemph) || (-1 == itempl)) {
		;
	}
	else {
		itemph <<= 4;
		itemph |= itempl;
		*pdst = itemph;
		bRetVal = true;
	}

	return bRetVal;
}

static boolean GpsXorCheckCalc(const u8 *pcu8Dat, const u16 cu16Len, u8 * u8CalcVal)
{
	boolean bRetVal = false;

	if(IS_NULL_P(pcu8Dat)) {
		;
	}
	else {
		u8 u8Verify = 0;
		for(u16 u16Cnt=0; u16Cnt<cu16Len; u16Cnt++) {
			u8Verify ^= pcu8Dat[u16Cnt];
		}
		*u8CalcVal = u8Verify;
		bRetVal = true;
	}

	return bRetVal;
}

static boolean GpsDataVerifyIsOk(const u8 *pcu8Dat, const u16 cu16Len)
{
	boolean bRetVal = false;

	u8 * pu8CheckPrefix = (u8 *)strchr((const char *)pcu8Dat, NMEA_FRAME_CHECK_PREFIX_SIGN);
	if(NULL == pu8CheckPrefix) {
		;
	}
	else {
		u8 u8CalcVal;
		u8 u8CheckVal;

		u8 * pu8CalcData = (u8 *)pcu8Dat + NMEA_FRAME_START_SIGN_LEN;
		u16 u16CalcDataLen = pu8CheckPrefix - pu8CalcData;
		u8 * pu8CheckData = pu8CheckPrefix + NMEA_FRAME_CHECK_PREFIX_SIGN_LEN;

		if(IS_TRUE_BIT(GpsXorCheckCalc(pu8CalcData, u16CalcDataLen, &u8CalcVal))
		&& IS_TRUE_BIT(ASCII2Byte(pu8CheckData, &u8CheckVal))) {
			if(u8CalcVal == u8CheckVal) {
				bRetVal = true;
			}
			else {
				;
			}
		}
		else {
			;
		}
	}

	return bRetVal;
}

static void nmeaDataRecv(u8 gpsData)
{
	static enGpsDataRxSt senGpsDataRxSt = GPS_DATA_RX_ST_IDLE;
	static u8 su8GpsRxBuff[GPS_RX_BUFF_SIZE];
	static u16 su16Index;

	if(su16Index < sizeof(su8GpsRxBuff)) {
		;
	}
	else {
		M_ClrBit(su16Index);
		senGpsDataRxSt = GPS_DATA_RX_ST_IDLE;
	}

	switch(senGpsDataRxSt) {
		case GPS_DATA_RX_ST_IDLE:
			if(NMEA_FRAME_START_SIGN == gpsData) {
				M_ClrBit(su16Index);
				memset(su8GpsRxBuff, 0, sizeof(su8GpsRxBuff));
				su8GpsRxBuff[su16Index++] = gpsData;
				senGpsDataRxSt = GPS_DATA_RX_ST_RXING;
			}
			else {
				;
			}
			break;

		case GPS_DATA_RX_ST_RXING:
			su8GpsRxBuff[su16Index++] = gpsData;
			if(NMEA_FRAME_CHECK_PREFIX_SIGN == gpsData) {
				senGpsDataRxSt = GPS_DATA_RX_ST_VERIFY_1;
			}
			else {
				;
			}
			break;

		case GPS_DATA_RX_ST_VERIFY_1:
			su8GpsRxBuff[su16Index++] = gpsData;
			senGpsDataRxSt = GPS_DATA_RX_ST_VERIFY_2;
			break;

		case GPS_DATA_RX_ST_VERIFY_2:
			su8GpsRxBuff[su16Index++] = gpsData;
			if(IS_TRUE_BIT(GpsDataVerifyIsOk(su8GpsRxBuff, su16Index))) {
				senGpsDataRxSt = GPS_DATA_RX_ST_END_1;
			}
			else {
				senGpsDataRxSt = GPS_DATA_RX_ST_IDLE;
			}
			break;

		case GPS_DATA_RX_ST_END_1:
			if(NMEA_FRAME_END_SIGN_1 == gpsData) {
				su8GpsRxBuff[su16Index++] = gpsData;
				senGpsDataRxSt = GPS_DATA_RX_ST_END_2;
			}
			else {
				senGpsDataRxSt = GPS_DATA_RX_ST_IDLE;
			}
			break;

		case GPS_DATA_RX_ST_END_2:
			if(NMEA_FRAME_END_SIGN_2 == gpsData) {
				su8GpsRxBuff[su16Index++] = gpsData;
				nmeaPackCaptured(su8GpsRxBuff, su16Index);
			}
			else {
				;
			}
			senGpsDataRxSt = GPS_DATA_RX_ST_IDLE;
			break;

		default:
			senGpsDataRxSt = GPS_DATA_RX_ST_IDLE;
			break;
	}
}

void nmeaMultiDataRecv(u8 * u8Buff, u16 u16Len)
{
	while(u16Len--) {
		nmeaDataRecv(*u8Buff++);
	}
}

static boolean nmeaFieldAddrInfoExtract(u8 ** u8FieldBuff, u16 u16FieldNum, u8 * u8Buff, u16 u16Len)
{
	boolean bRetVal = false;

	if((NULL == u8FieldBuff) || (NULL == u8Buff) || (u16FieldNum > u16Len)) {
		;
	}
	else {
		u16 u16Offset = 0;
		u16 u16SeparatorNum = 0;

		while(u16Offset < u16Len) {
			u16SeparatorNum += (NMEA_FRAME_SEPARATOR_SIGN == *(u8Buff + u16Offset)) ? (1) : (0);
			u16Offset++;
		}

		if(u16SeparatorNum + 1 < u16FieldNum) {
			;
		}
		else {
			u16 u16ValidFieldNum = 0;
			u8 * u8TempAddr = u8Buff;

			u8FieldBuff[u16ValidFieldNum++] = u8Buff;

			for(u16 u16Cnt = 0; u16Cnt < u16FieldNum; u16Cnt++) {
				u8TempAddr = (u8 *)strchr((const char *)u8TempAddr, NMEA_FRAME_SEPARATOR_SIGN);
				if(NULL == u8TempAddr) {
					break;
				}
				else {
					u8TempAddr += NMEA_FRAME_SEPARATOR_SIGN_LEN;
					u8FieldBuff[u16ValidFieldNum++] = u8TempAddr;
				}
			}

			bRetVal = true;
		}
	}

	return bRetVal;
}

/* GPGLL field number */
#define GPS_GLL_LATITUDE				(1)
#define GPS_GLL_LAT_NORTH_SOUTH		(2)
#define GPS_GLL_LONGITUDE				(3)
#define GPS_GLL_LONG_EAST_WEST		(4)
#define GPS_GLL_UTC_TIME				(5)
#define GPS_GLL_STATUS					(6)

#define GPS_GLL_FIELD_NUM			(GPS_GLL_STATUS + 1)

static boolean parseGLL(LocationVectorInfo * localInfo, u8 * u8Buff, u16 u16Len)
{
	boolean bRetVal = false;

#ifdef GPS_NMEA_TEST
	nmeaDebugString("#####GLL#####\n");
#endif

	if((NULL == localInfo) || (NULL == u8Buff) || (0 == u16Len)) {
		;
	}
	else {
		u8 * u8FieldBuff[GPS_GLL_FIELD_NUM] = {NULL};

		if(false == nmeaFieldAddrInfoExtract(u8FieldBuff, GPS_GLL_FIELD_NUM, u8Buff, u16Len)) {
			;
		}
		else {
			localInfo->locationCommonPara.packType = PCAK_TYPE_GLL;
			bRetVal = true;
		}
	}

	return bRetVal;
}

/* GPVTG field number */
#define GPS_VTG_MOVE_ANGLE_T			(1)
#define GPS_VTG_TRUE_NORTH_REF		(2)
#define GPS_VTG_MOVE_ANGLE_M			(3)
#define GPS_VTG_MAGNETIC_NORTH_REF	(4)
#define GPS_VTG_SPEED_KNOTS			(5)
#define GPS_VTG_KNOTS					(6)
#define GPS_VTG_SPEED_KILOMETER		(7)
#define GPS_VTG_KILOMETER				(8)

#define GPS_VTG_FIELD_NUM			(GPS_VTG_KILOMETER + 1)

static boolean parseVTG(LocationVectorInfo * localInfo, u8 * u8Buff, u16 u16Len)
{
	boolean bRetVal = false;

#ifdef GPS_NMEA_TEST
	nmeaDebugString("#####VTG#####\n");
#endif

	if((NULL == localInfo) || (NULL == u8Buff) || (0 == u16Len)) {
		;
	}
	else {
		u8 * u8FieldBuff[GPS_VTG_FIELD_NUM] = {NULL};

		if(false == nmeaFieldAddrInfoExtract(u8FieldBuff, GPS_VTG_FIELD_NUM, u8Buff, u16Len)) {
			;
		}
		else {
			localInfo->locationCommonPara.packType = PCAK_TYPE_VTG;
			bRetVal = true;
		}
	}

	return bRetVal;
}

/* GPRMC field number */
#define GPS_RMC_UTC_TIME				(1)
#define GPS_RMC_STATUS					(2)
#define GPS_RMC_LATITUDE				(3)
#define GPS_RMC_LAT_NORTH_SOUTH		(4)
#define GPS_RMC_LONGITUDE				(5)
#define GPS_RMC_LONG_EAST_WEST		(6)
#define GPS_RMC_SPEED_KNOTS			(7)
#define GPS_RMC_COURSE_DEGREE		(8)
#define GPS_RMC_UTC_DATE				(9)
#define GPS_RMC_VARIATION				(10)
#define GPS_RMC_VAR_EAST_WEST		(11)
#define GPS_RMC_MODE					(12)

#define GPS_RMC_FIELD_NUM			(GPS_RMC_MODE + 1)

static boolean parseRMC(LocationVectorInfo * localInfo, u8 * u8Buff, u16 u16Len)
{
	boolean bRetVal = false;

#ifdef GPS_NMEA_TEST
	nmeaDebugString("#####RMC#####\n");
#endif

	if((NULL == localInfo) || (NULL == u8Buff) || (0 == u16Len)) {
		;
	}
	else {
		u8 * u8FieldBuff[GPS_RMC_FIELD_NUM] = {NULL};

		if(false == nmeaFieldAddrInfoExtract(u8FieldBuff, GPS_RMC_FIELD_NUM, u8Buff, u16Len)) {
			;
		}
		else {
			u32 u32TempLat = 0;
			u32 u32TempLon = 0;
			u32 u32dd = 0;
			u32 u32mm = 0;
			u8 u8DecimalLen = 0;
			u8 * pData = u8FieldBuff[GPS_RMC_UTC_DATE];
			u8 * pTime = u8FieldBuff[GPS_RMC_UTC_TIME];
			u8 * pLat = u8FieldBuff[GPS_RMC_LATITUDE];
			u8 * pLon = u8FieldBuff[GPS_RMC_LONGITUDE];

			if( 'A' == *u8FieldBuff[GPS_RMC_STATUS]
			|| 'a' == *u8FieldBuff[GPS_RMC_STATUS] ) {
				localInfo->locationCommonPara.dataValidity = LOC_INFO_VALID;
			}
			else {
				localInfo->locationCommonPara.dataValidity = LOC_INFO_INVALID;
			}

			// 日期
			localInfo->dataTime.date.day = ((pData[0] - '0')*10) + (pData[1] - '0');
			localInfo->dataTime.date.month = ((pData[2] - '0')*10) + (pData[3] - '0');
			localInfo->dataTime.date.year = ((pData[4] - '0')*10) + (pData[5] - '0');

			// 时间
			localInfo->dataTime.time.hour = ((pTime[0] - '0')*10) + (pTime[1] - '0');
			localInfo->dataTime.time.minute = ((pTime[2] - '0')*10) + (pTime[3] - '0');
			localInfo->dataTime.time.second = ((pTime[4] - '0')*10) + (pTime[5] - '0');

			// 纬度
			while((*pLat != NMEA_FRAME_POINT_SIGN) && (*pLat != NMEA_FRAME_SEPARATOR_SIGN)) {
				u32TempLat = (u32TempLat * 10) + (*pLat - '0');
				pLat++;
			}
			u32dd = u32TempLat / 100;
			u32mm = u32TempLat % 100;
			if(NMEA_FRAME_POINT_SIGN == *pLat) {
				pLat++;
			}
			else {
				;
			}

			while((u8DecimalLen < INDEX_2_INT_LEN_MAX)
				&& (*pLat != NMEA_FRAME_SEPARATOR_SIGN)) {
				u32mm = (u32mm * 10) + (*pLat - '0');
				pLat++;
				u8DecimalLen++;
			}
			u32TempLat = (u32)((u32dd * 1000000)
				+ ((u32mm / 60.0) * (1000000 / Index2Int(u8DecimalLen)) + 0.5));
			localInfo->positionInfo.lat = u32TempLat;
			if( 'S' == *u8FieldBuff[GPS_RMC_LAT_NORTH_SOUTH]
			|| 's' == *u8FieldBuff[GPS_RMC_LAT_NORTH_SOUTH] ) {
				localInfo->positionInfo.latRegion = LATITUDE_SOUTH;
			}
			else {
				localInfo->positionInfo.latRegion = LATITUDE_NORTH;
			}

			u8DecimalLen = 0;

			// 经度
			while((*pLon != NMEA_FRAME_POINT_SIGN) && (*pLon != NMEA_FRAME_SEPARATOR_SIGN)) {
				u32TempLon = (u32TempLon * 10) + (*pLon - '0');
				pLon++;
			}
			u32dd = u32TempLon / 100;
			u32mm = u32TempLon % 100;
			if(NMEA_FRAME_POINT_SIGN == *pLon) {
				pLon++;
			}
			else {
				;
			}
			while((u8DecimalLen < INDEX_2_INT_LEN_MAX)
				&& (*pLon != NMEA_FRAME_SEPARATOR_SIGN)) {
				u32mm = (u32mm * 10) + (*pLon - '0');
				pLon++;
				u8DecimalLen++;
			}
			u32TempLon = (u32)((u32dd * 1000000)
				+ ((u32mm / 60.0) * (1000000 / Index2Int(u8DecimalLen)) + 0.5));
			localInfo->positionInfo.lon = u32TempLon;
			if( 'W' == *u8FieldBuff[GPS_RMC_LONG_EAST_WEST]
			|| 'w' == *u8FieldBuff[GPS_RMC_LONG_EAST_WEST] ) {
				localInfo->positionInfo.lonRegion = LONGITUDE_WEST;
			}
			else {
				localInfo->positionInfo.lonRegion = LONGITUDE_EAST;
			}

			// 速度
			localInfo->pointInfo.speed = 1.852 * atof((const char *)u8FieldBuff[GPS_RMC_SPEED_KNOTS]);

			// 角度
			localInfo->pointInfo.course = atof((const char *)u8FieldBuff[GPS_RMC_COURSE_DEGREE]);

			localInfo->locationCommonPara.packType = PCAK_TYPE_RMC;

			bRetVal = true;
		}
	}

	return bRetVal;
}

/* GPGSV field number */
#define GPS_GSV_SENTENCE_NUM				(1)
#define GPS_GSV_SENTENCE_ITEM				(2)
#define GPS_GSV_VISIBLE_SATELLITES_NUM	(3)
#define GPS_GSV_PRN_CODE_1				(4)
#define GPS_GSV_SATELLITES_ELEVATION_1	(5)
#define GPS_GSV_SATELLITES_AZIMUTH_1		(6)
#define GPS_GSV_SNR_1						(7)
#define GPS_GSV_PRN_CODE_2				(8)
#define GPS_GSV_SATELLITES_ELEVATION_2	(9)
#define GPS_GSV_SATELLITES_AZIMUTH_2		(10)
#define GPS_GSV_SNR_2						(11)
#define GPS_GSV_PRN_CODE_3				(12)
#define GPS_GSV_SATELLITES_ELEVATION_3	(13)
#define GPS_GSV_SATELLITES_AZIMUTH_3		(14)
#define GPS_GSV_SNR_3						(15)
#define GPS_GSV_PRN_CODE_4				(16)
#define GPS_GSV_SATELLITES_ELEVATION_4	(17)
#define GPS_GSV_SATELLITES_AZIMUTH_4		(18)
#define GPS_GSV_SNR_4						(19)

#define GSV_COM_INFO_SEPARATOR_NUM	(4)
#define GSV_SNR_INFO_SEPARATOR_NUM	(3)
#define GSV_SATELLITE_INFO_NUM_MAX	(4)

static boolean parseGSV(LocationVectorInfo * localInfo, u8 * u8Buff, u16 u16Len)
{
	boolean bRetVal = false;

#ifdef GPS_NMEA_TEST
	nmeaDebugString("#####GSV#####\n");
#endif

	if((NULL == localInfo) || (NULL == u8Buff) || (0 == u16Len)) {
		;
	}
	else {
		localInfo->locationCommonPara.packType = PCAK_TYPE_GSV;

		bRetVal = true;
	}

	return bRetVal;

#if 0
	if(u8Buff == NULL) {
		;
	}
	else {
		u8 *com_para = u8Buff;
		u8 u8GsvHead;

		M_SetBit(u8GsvHead);

		localInfo->locationCommonPara.packType = PCAK_TYPE_GSV;

		for(u8 u8Cnt1 = 0; u8Cnt1 < GSV_COM_INFO_SEPARATOR_NUM; u8Cnt1++) {
			com_para = (u8*)strchr((const s8*)com_para, ',');
			if(NULL == com_para) {
				M_ClrBit(u8GsvHead);
				break;
			}
			else {
				com_para++;
			}
		}

		if(IS_FALSE_BIT(u8GsvHead)) {
			M_ClrBit(bRetVal);
		}
		else {
			for(u8 u8Cnt2 = 0; u8Cnt2 < GSV_SATELLITE_INFO_NUM_MAX; u8Cnt2++) {

				u8 u8Err;

				M_SetBit(u8Err);

				for(u8 u8Cnt3 = 0; u8Cnt3<GSV_SNR_INFO_SEPARATOR_NUM; u8Cnt3++) {
					com_para = (u8*)strchr((const s8*)com_para, ',');
					if(NULL == com_para) {
						M_ClrBit(u8Err);
						break;
					}
					else {
						com_para++;
					}
				}

				if(IS_FALSE_BIT(u8Err)) {
					M_ClrBit(bRetVal);
					break;
				}
				else {
					u8 u8TempVal = 0;
					u8TempVal = atoi((char const *)com_para);
					if(u8TempVal > signalSNR) {
						signalSNR = u8TempVal;
					}
					else {
						;
					}

#ifdef GPS_NMEA_TEST
					nmeaDebugString("\n**********\n");
					debugPrintfHex2Ascii(&u8TempVal, 1);
					debugPrintfHex2Ascii(&signalSNR, 1);
					nmeaDebugString("\n**********\n");
#endif

					com_para = (u8*)strchr((const s8*)com_para, ',');

					if(NULL == com_para) {
						break;
					}
					else {
						com_para++;
					}
				}
			}
		}
	}

	if(IS_FALSE_BIT(bRetVal)) {
		;
	}
	else {
		local_info->SignalSNR = signalSNR;
	}

	return bRetVal;
#endif
}

/* GPGSA field number */
#define GPS_GSA_POSITIONING_MODE			(1)
#define GPS_GSA_LOCATION_TYPE				(2)
#define GPS_GSA_PRN_CODE_1ST_CHANNAL	(3)
#define GPS_GSA_PRN_CODE_2ST_CHANNAL	(4)
#define GPS_GSA_PRN_CODE_3RD_CHANNAL	(5)
#define GPS_GSA_PRN_CODE_4TH_CHANNAL	(6)
#define GPS_GSA_PRN_CODE_5TH_CHANNAL	(7)
#define GPS_GSA_PRN_CODE_6TH_CHANNAL	(8)
#define GPS_GSA_PRN_CODE_7TH_CHANNAL	(9)
#define GPS_GSA_PRN_CODE_8TH_CHANNAL	(10)
#define GPS_GSA_PRN_CODE_9TH_CHANNAL	(11)
#define GPS_GSA_PRN_CODE_10TH_CHANNAL	(12)
#define GPS_GSA_PRN_CODE_11TH_CHANNAL	(13)
#define GPS_GSA_PRN_CODE_12TH_CHANNAL	(14)
#define GPS_GSA_PDOP						(15)
#define GPS_GSA_HDOP						(16)
#define GPS_GSA_VDOP						(17)

#define GPS_GSA_FIELD_NUM		(GPS_GSA_VDOP + 1)

#define GPS_GSA_3D_FIXED_VAL				(3)
#define GPS_GSA_2D_FIXED_VAL				(2)
#define GPS_GSA_INVALID_FIXED_VAL			(1)

static boolean parseGSA(LocationVectorInfo * localInfo, u8 * u8Buff, u16 u16Len)
{
	boolean bRetVal = false;

#ifdef GPS_NMEA_TEST
	nmeaDebugString("#####GSA#####\n");
#endif

	if((NULL == localInfo) || (NULL == u8Buff) || (0 == u16Len)) {
		;
	}
	else {
		u8 * u8FieldBuff[GPS_GSA_FIELD_NUM] = {NULL};

		if(false == nmeaFieldAddrInfoExtract(u8FieldBuff, GPS_GSA_FIELD_NUM, u8Buff, u16Len)) {
			;
		}
		else {
			int iTemp = 0;

			iTemp = atoi((const char *)u8FieldBuff[GPS_GSA_LOCATION_TYPE]);
			if(GPS_GSA_3D_FIXED_VAL == iTemp) {
				localInfo->locationCommonPara.fixStatus = FIX_3D;
			}
			else if(GPS_GSA_2D_FIXED_VAL == iTemp) {
				localInfo->locationCommonPara.fixStatus = FIX_2D;
			}
			else if(GPS_GSA_INVALID_FIXED_VAL == iTemp) {
				localInfo->locationCommonPara.fixStatus = FIX_INVALID;
			}
			else {
				localInfo->locationCommonPara.fixStatus = FIX_INVALID;
			}

			localInfo->locationPerformancePara.PDOP = atof((const char *)u8FieldBuff[GPS_GSA_PDOP]);
			localInfo->locationPerformancePara.HDOP = atof((const char *)u8FieldBuff[GPS_GSA_HDOP]);
			localInfo->locationPerformancePara.VDOP = atof((const char *)u8FieldBuff[GPS_GSA_VDOP]);

			localInfo->locationCommonPara.packType = PCAK_TYPE_GSA;

			bRetVal = true;
		}
	}

	return bRetVal;
}

/* GPGGA field number */
#define GPS_GGA_UTC_TIME				(1)
#define GPS_GGA_LATITUDE				(2)
#define GPS_GGA_LAT_NORTH_SOUTH		(3)
#define GPS_GGA_LONGITUDE				(4)
#define GPS_GGA_LONG_EAST_WEST		(5)
#define GPS_GGA_QUALITY_INDICATOR	(6)
#define GPS_GGA_NR_OF_SATELITES		(7)
#define GPS_GGA_HORZ_DILUTION			(8)
#define GPS_GGA_ALTITUDE				(9)
#define GPS_GGA_ALTITUDE_UNIT			(10)
#define GPS_GGA_GEOIDAL_HEIGHT		(11)
#define GPS_GGA_GEOIDAL_HEIGHT_UNIT	(12)
#define GPS_GGA_AGE_OF_DATA			(13)
#define GPS_GGA_STATION_ID			(14)

#define GPS_GGA_FIELD_NUM			(GPS_GGA_STATION_ID + 1)

static boolean parseGGA(LocationVectorInfo * localInfo, u8 * u8Buff, u16 u16Len)
{
	boolean bRetVal = false;

#ifdef GPS_NMEA_TEST
	nmeaDebugString("#####GGA#####\n");
#endif

	if((NULL == localInfo) || (NULL == u8Buff) || (0 == u16Len)) {
		;
	}
	else {
		u8 * u8FieldBuff[GPS_GGA_FIELD_NUM] = {NULL};

		if(false == nmeaFieldAddrInfoExtract(u8FieldBuff, GPS_GGA_FIELD_NUM, u8Buff, u16Len)) {
			;
		}
		else {
			localInfo->locationCommonPara.satelliteNum = atoi((const char *)u8FieldBuff[GPS_GGA_NR_OF_SATELITES]);
			localInfo->pointInfo.altitude = atof((const char *)u8FieldBuff[GPS_GGA_ALTITUDE]);

			localInfo->locationCommonPara.packType = PCAK_TYPE_GGA;

			bRetVal = true;
		}
	}

	return bRetVal;
}

typedef boolean (*packAnalysis)(LocationVectorInfo * localInfo, u8 * u8Buff, u16 u16Len);

typedef struct {
	s8 * head;
	packAnalysis analysis;
}nmeaHandler;

nmeaHandler nmeaHandlerMenu[] = {
	{"GGA", parseGGA},
	{"GSA", parseGSA},
	{"GSV", parseGSV},
	{"RMC", parseRMC},
	{"VTG", parseVTG},
	{"GLL", parseGLL},
};

static const u8 NMEA_MAX_ANLY_NUM = sizeof(nmeaHandlerMenu) / sizeof(nmeaHandler);

static boolean nmeaPackVerifyIsOk(u8 * u8Buff, u16 u16Len)
{
	boolean bRetVal = false;

	if((NULL == u8Buff) || (u16Len > GPS_RX_BUFF_SIZE)) {
		;
	}
	else {
		if( (u8Buff[0] != NMEA_FRAME_START_SIGN)
		|| (u8Buff[u16Len - 1] != NMEA_FRAME_END_SIGN_2)
		|| (u8Buff[u16Len - 2] != NMEA_FRAME_END_SIGN_1)
		|| (u8Buff[u16Len - 5] != NMEA_FRAME_CHECK_PREFIX_SIGN) ) {
			;
		}
		else {
			if(IS_TRUE_BIT(GpsDataVerifyIsOk(u8Buff, u16Len))) {
				bRetVal = true;
			}
			else {
				;
			}
		}
	}

	return bRetVal;
}

static boolean nmeaLocationInfoCheckAndCorrect(LocationVectorInfo * localInfo)
{
	boolean bRetVal = false;

	if(IS_NULL_P(localInfo)) {
		;
	}
	else {
		bRetVal = true;
	}

	return bRetVal;
}

static boolean nmeaUpdateLocationInfo(LocationVectorInfo * localInfo)
{
	boolean bRetVal = false;

	if(IS_NULL_P(pLocationInfo)) {
		nmeaOSSchedLock();
		nmeaLocationInfoUpdateNotice(&locationInfo);
		nmeaOSSchedUnlock();
	}
	else {
		nmeaOSSchedLock();
		memset(pLocationInfo, 0, sizeof(LocationVectorInfo));
		memcpy(pLocationInfo, &locationInfo, sizeof(LocationVectorInfo));
		memset(&locationInfo, 0, sizeof(locationInfo));
		nmeaOSSchedUnlock();
		nmeaLocationInfoUpdateNotice(pLocationInfo);
		bRetVal = true;
	}

	return bRetVal;
}

boolean nmeaAnalysis(u8 * u8Buff, u16 u16Len)
{
	boolean bRetVal = false;

	if(IS_FALSE_BIT(nmeaPackVerifyIsOk(u8Buff, u16Len))) {
		nmeaDebugString("Gps pack verify err!\n");
	}
	else {
#ifdef GPS_NMEA_TEST
		nmeaDebugPrintf(u8Buff, u16Len);
#endif
		for(u8 u8Index = 0; u8Index < NMEA_MAX_ANLY_NUM; u8Index++) {
			if(memcmp(nmeaHandlerMenu[u8Index].head, &u8Buff[3], 3) == 0) {
				bRetVal = nmeaHandlerMenu[u8Index].analysis(&locationInfo, u8Buff, u16Len);
			}
			else {
				;
			}
		}

		if(PCAK_TYPE_RMC == locationInfo.locationCommonPara.packType) {
			locationInfo.locationCommonPara.packType = PCAK_TYPE_NONE;
			nmeaLocationInfoCheckAndCorrect(&locationInfo);
			nmeaUpdateLocationInfo(&locationInfo);
		}
		else {
			;
		}
	}

	return bRetVal;
}

boolean nmeaInit(LocationVectorInfo * localInfo)
{
	boolean bRetVal = false;

	nmeaOSSchedLock();
	memset(&locationInfo, 0, sizeof(LocationVectorInfo));
	pLocationInfo = localInfo;
	nmeaOSSchedUnlock();
	nmeaInitHook();
	nmeaDebugString("Gps nmea initialize!\n");
	bRetVal = true;

	return bRetVal;
}

