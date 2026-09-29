#pragma once
#include <stddef.h>
#include <stdlib.h>
#define MALLOC_CAP_SPIRAM   (1<<10)
#define MALLOC_CAP_8BIT     (1<<2)
#define MALLOC_CAP_INTERNAL (1<<11)
#define MALLOC_CAP_DEFAULT  (1<<12)
static inline void* heap_caps_malloc(size_t n,unsigned){return malloc(n);}
static inline void* heap_caps_calloc(size_t a,size_t b,unsigned){return calloc(a,b);}
static inline void  heap_caps_free(void*p){free(p);}
static inline size_t heap_caps_get_free_size(unsigned){return 0;}
static inline size_t heap_caps_get_largest_free_block(unsigned){return 0;}
