#ifndef SB_COMMON_H
#define SB_COMMON_H

#include <cassert>
#include <cstdio>

#define VAR(v,w) __typeof(w) v=w

#define FOREACH(it,c) for(VAR(it,(c)->begin());it!=(c)->end();++it)
#define FOREACH_R(it,c) for(VAR(it,(c).begin());it!=(c).end();++it)
#define REP(i, times) for(unsigned int i = 0; i < (unsigned int)times; i++)

#define DBG(fmt, ...) fprintf(stderr, "\033[22;32m:%s:%d:" fmt "\033[0m\n", __func__, __LINE__, ##__VA_ARGS__)
#define ERR(fmt, ...) fprintf(stderr, "\033[01;31m:%s:%d:" fmt "\033[0m\n", __func__, __LINE__, ##__VA_ARGS__)
#define WRN(fmt, ...) fprintf(stderr, "\033[01;33m:%s:%d:" fmt "\033[0m\n", __func__, __LINE__, ##__VA_ARGS__)

#define SB_ASSERT(expr) assert(expr)

//add swizzle operators
#define GLM_SWIZZLE

//remove warnings
#define GLM_FORCE_RADIANS

#endif
