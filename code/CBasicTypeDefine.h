/***************************************************
李宏展
2016-3-9 14:23:00

***************************************************/
#ifndef C_BASIC_TYPE_DEFINE_
#define C_BASIC_TYPE_DEFINE_
//基于32位MCU,STM32
#include"stdint.h"
#include"stddef.h"

typedef int32_t  s32;
typedef int16_t s16;
typedef char s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef volatile const int32_t vsc32;  /*!< Read Only */
typedef volatile const int16_t vsc16;  /*!< Read Only */
typedef volatile const int8_t vsc8;   /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef volatile uint32_t  vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

typedef volatile const uint32_t vuc32;  /*!< Read Only */
typedef volatile const uint16_t vuc16;  /*!< Read Only */
typedef volatile const uint8_t vuc8;   /*!< Read Only */

typedef enum {FALSE = 0, TRUE = !FALSE} bool;
typedef u8 BOOL; //FALSE 0, TRUE != 0

#pragma pack(1)//暂时继承,因其他地方可能有潜在的依赖关系
//最好不要限制对齐,待后续稳定测试没有问题再行修改
typedef struct
{
	u8 year;
	u8 month;
	u8 day;
}date_t;
typedef struct
{
	u8 hour;
	u8 minute;
	u8 second;
}time_t;
typedef struct
{
	date_t date;
	time_t time;
}datetime_t;
#pragma pack()


#endif


