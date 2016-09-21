/*-----------------------------------------------------------------------------
 Memory cache manager code for Epson S1D13522 LCD frame buffer driver.

 Copyright(c) YotaDevices Corporation 2016.

 This file is subject to the terms and conditions of the GNU General Public
 License. See the file COPYING in the main directory of this archive for
 more details.

----------------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/rwsem.h>
#include <linux/compiler.h>
#include <linux/atomic.h>
#include <linux/gfp.h>

#include "s1d13522fb.h"

struct s1d13522_mem_cache {
	struct rw_semaphore cache_lock;
	struct kmem_cache *local_mem_cache;
	atomic_t num_obj;
};

/*
 * must be called only for write locked cache
 */
static inline void wait_for_empty_cache(struct s1d13522_mem_cache *cache)
{
	while (atomic_read(&cache->num_obj) > 0) {
		up_write(&cache->cache_lock);
		schedule();
		down_write(&cache->cache_lock);
	}
}


struct s1d13522_mem_cache *s1d13522_init_mem_chache(void)
{
	struct s1d13522_mem_cache *cache;

	cache = kzalloc(sizeof(struct s1d13522_mem_cache), GFP_KERNEL);
	if (cache) {
		init_rwsem(&cache->cache_lock);
		atomic_set(&cache->num_obj, 0);
	}

	return cache;
}

void s1d13522_destroy_mem_cache(struct s1d13522_mem_cache *cache)
{
	down_write(&cache->cache_lock);

	wait_for_empty_cache(cache);
	kmem_cache_destroy(cache->local_mem_cache);
	cache->local_mem_cache = NULL;

	up_write(&cache->cache_lock);

	kfree(cache);
}


void s1d13522_put_mem_obj(struct s1d13522_mem_cache *cache, void *obj)
{
	down_read(&cache->cache_lock);

	kmem_cache_free(cache->local_mem_cache, obj);
	atomic_dec(&cache->num_obj);

	up_read(&cache->cache_lock);
}

void *s1d13522_get_mem_obj(struct s1d13522_mem_cache *cache, size_t size)
{
	struct kmem_cache *mem_cache;
	void *obj;

	down_write(&cache->cache_lock);
realloc:
	if (unlikely(!cache->local_mem_cache)) {
		/*
		 * align memory size to avoid
		 * memory fragmentation and improve
		 * speed of allocation
		 */
		cache->local_mem_cache = \
			kmem_cache_create("s1d13522fb", size, 0,
					  SLAB_TEMPORARY, NULL);
		if (!cache->local_mem_cache) {
			up_write(&cache->cache_lock);
			return NULL;
		}

	}
	mem_cache = cache->local_mem_cache;

	if (unlikely(mem_cache->size < size)) {
		wait_for_empty_cache(cache);
		kmem_cache_destroy(mem_cache);
		cache->local_mem_cache = NULL;
		goto realloc;
	}

	downgrade_write(&cache->cache_lock);

	obj = kmem_cache_alloc(mem_cache, GFP_KERNEL | __GFP_NOFAIL);
	if (!obj)
		atomic_inc(&cache->num_obj);

	up_read(&cache->cache_lock);

	return obj;
}
