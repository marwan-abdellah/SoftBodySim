#ifndef SB_COMMON_H
#define SB_COMMON_H

#include <cassert>
#include <cstdio>

/**
 * Few useful macros to simplyfiy containers itering
 */
#define VAR(v,w) __typeof(w) v=w
#define FOREACH(it,c) for(VAR(it,(c)->begin());it!=(c)->end();++it)
#define FOREACH_R(it,c) for(VAR(it,(c).begin());it!=(c).end();++it)
#define REP(i, times) for(unsigned int i = 0; i < (unsigned int)times; i++)

#ifdef ENABLE_DEBUG
	#define DBG(fmt, ...) fprintf(stderr, "\033[22;32m:%s:%d:" fmt "\033[0m\n", __func__, __LINE__, ##__VA_ARGS__)
	#define ERR(fmt, ...) fprintf(stderr, "\033[01;31m:%s:%d:" fmt "\033[0m\n", __func__, __LINE__, ##__VA_ARGS__)
	#define WRN(fmt, ...) fprintf(stderr, "\033[01;33m:%s:%d:" fmt "\033[0m\n", __func__, __LINE__, ##__VA_ARGS__)
#else
	#define DBG(fmt, ...)
	#define ERR(fmt, ...)
	#define WRN(fmt, ...)
#endif

#define SB_ASSERT(expr) assert(expr)
#define SB_CHECK(expr, msg) do {if (expr) WRN("%s", msg); } while(0);

//add swizzle operators to use vec3.xy(), vec3.zx() functions
#define GLM_SWIZZLE

//remove warnings
#define GLM_FORCE_RADIANS

#endif
