/*
 * Written by Ivanov Dmitry, 2016 (dmitriy.ivanov@yotadevices.com).
 *
 * Copyright(c) YotaDevices Corporation 2016.
 */

#ifndef __S1D13522MEM_H__
#define	__S1D13522MEM_H__

struct s1d13522_mem_cache;

struct s1d13522_mem_cache *s1d13522_init_mem_chache(void);
void s1d13522_destroy_mem_cache(struct s1d13522_mem_cache *cache);
void s1d13522_put_mem_obj(struct s1d13522_mem_cache *cache, void *obj);
void *s1d13522_get_mem_obj(struct s1d13522_mem_cache *cache, size_t size);

#endif
