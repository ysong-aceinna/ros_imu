/*******************************************************
 * @file macro.h
 * @author SongYang (ysong@aceinna.com)
 * @brief 
 * @date 2021-05-01
 * 
 * @copyright Copyright (c) 2021
 * 
*******************************************************/
#pragma once

#if !defined (MAX)
#define MAX(x,y) ((x)>(y)?(x):(y))
#endif

#if !defined (MIN)
#define MIN(x,y) ((x)<(y)?(x):(y))
#endif

#if !defined (SWAPTWO)
#define SWAPTWO( a, b, temp ) \
{                                       \
  (temp) = (a);                         \
  (a) = (b);                            \
  (b) = (temp);                         \
}
#endif

// common macros
#if !defined (SAFEDELETE)
#define SAFEDELETE(p) if( (p) ){ delete (p); (p) = NULL; }
#endif

#if !defined (SAFEDELETEARRAY)
#define SAFEDELETEARRAY(p) if( (p) ){ delete [] (p); (p) = NULL; }
#endif
