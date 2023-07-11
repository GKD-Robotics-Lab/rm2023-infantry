#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#ifndef __packed

#define __packed __attribute__(())

#endif

#include "stdint.h"
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



