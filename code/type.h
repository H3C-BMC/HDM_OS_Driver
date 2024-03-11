/*****************************************************************************
                版权所有(C)，2007-2022，杭州华三通信技术有限公司
------------------------------------------------------------------------------
                            type.h
  产 品 名: VERSION
  模 块 名:
  生成日期: 2022年3月18日
  作    者: x22827
  文件描述: 基本数据定义

------------------------------------------------------------------------------
   修改历史
   日期        姓名             描述
  --------------------------------------------------------------------------

*****************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif /* __cplusplus */

#ifndef _TYPE_H_
#define _TYPE_H_

#ifdef __GNUC__
#define PACKED __attribute__ ((packed))
#else
#define PACKED
#pragma pack(1)
#endif



#define UCHAR           unsigned char
#define USHORT          unsigned short
#define UINT            unsigned int
#define ULONG           unsigned long
#define DULONG          unsigned long long

#define CHAR            char
#define SHORT           short
#define INT             int
#define LONG            long
#define DLONG           long long

#define BOOL_T          bool

#define VOID            void
#define STATIC          static

#define BOOL_TRUE       true
#define BOOL_FALSE      false



#endif /* _TYPE_H_ */

#ifdef __cplusplus
}
#endif /* __cplusplus */

