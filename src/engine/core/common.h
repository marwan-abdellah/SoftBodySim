#define VAR(v,w) __typeof(w) v=w

#define FOREACH(it,c) for(VAR(it,(c)->begin());it!=(c)->end();++it)
#define FOREACH_R(it,c) for(VAR(it,(c).begin());it!=(c).end();++it)

#define DBG(fmt, ...) fprintf(stderr, "\033[22;32m:%s:%d:" fmt, __func__, __LINE__, ##__VA_ARGS__)
#define ERR(fmt, ...) fprintf(stderr, "\033[01;31m:%s:%d:" fmt, __func__, __LINE__, ##__VA_ARGS__)
#define WRN(fmt, ...) fprintf(stderr, "\033[01;33m:%s:%d:" fmt, __func__, __LINE__, ##__VA_ARGS__)
