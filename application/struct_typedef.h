#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H
#include "stdint.h"

#define __packed __attribute__((packed))

typedef signed char int8_t;
typedef signed short int int16_t;
//typedef signed int int32_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
//typedef unsigned int uint32_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;


/* �������뿪�أ��л��������Ų� */
//#define INFANTRY1_HUAN
#define INFANTRY2_ZHANG

/*
������

#ifdef INFANTRY1_HUAN
#define YAW_OFFSET_ECD_FORWARD			00
#define YAW_OFFSET_ECD_BACKWARD			00
#else
	#ifdef INFANTRY2_ZHANG
	#define YAW_OFFSET_ECD_FORWARD			00
	#define YAW_OFFSET_ECD_BACKWARD			00
	#endif
#endif

*/

#endif



