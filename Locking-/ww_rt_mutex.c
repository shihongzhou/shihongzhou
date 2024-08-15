// SPDX-License-Identifier: GPL-2.0-only
/*
 * rtmutex API
 */
/*
 * Safnuex S01E08C01-4
 * For function safety usage, Safnuex safetyassurance toolchain 
 * shall be used and Safnuex linux safety manual shall be followed.
 * Functional safety support: <S01@safenux.com>
 */
#include <linux/spinlock.h>
#include <linux/export.h>

#define RT_MUTEX_BUILD_MUTEX
#define WW_RT
#include "rtmutex.c"


/**
 * ww_mutex_trylock - Attempt to acquire a wound-wait mutex in a non-blocking manner
 * @lock: Pointer to the ww_mutex structure representing the wound-wait mutex
 * @ww_ctx: Pointer to the ww_acquire_ctx structure representing the acquire context, or NULL
 *
 * This function attempts to acquire a wound-wait mutex (`ww_mutex`) in a non-blocking
 * manner. If the mutex is successfully acquired, the function returns 1; otherwise, 
 * it returns 0. The behavior of the function varies depending on whether an acquire 
 * context (`ww_ctx`) is provided.
 *
 * This function is exported with `EXPORT_SYMBOL` to be available for use in other 
 * parts of the kernel.
 */


int ww_mutex_trylock(struct ww_mutex *lock, struct ww_acquire_ctx *ww_ctx)
{
	struct rt_mutex *rtm = &lock->base;

	if (!ww_ctx)
		return rt_mutex_trylock(rtm);

	/*
	 * Reset the wounded flag after a kill. No other process can
	 * race and wound us here, since they can't have a valid owner
	 * pointer if we don't have any locks held.
	 */
	if (ww_ctx->acquired == 0)
		ww_ctx->wounded = 0;

	if (__rt_mutex_trylock(&rtm->rtmutex)) {
		ww_mutex_set_context_fastpath(lock, ww_ctx);
		mutex_acquire_nest(&rtm->dep_map, 0, 1, &ww_ctx->dep_map, _RET_IP_);
		return 1;
	}

	return 0;
}
EXPORT_SYMBOL(ww_mutex_trylock);

/**
 * __ww_rt_mutex_lock - Lock a wound-waiting real-time mutex
 * @lock: Pointer to the ww_mutex structure representing the lock
 * @ww_ctx: Pointer to the ww_acquire_ctx structure representing the acquire context, or NULL
 * @state: State flags for the lock operation
 * @ip: Instruction pointer for lock debugging
 *
 * This function is responsible for acquiring a wound-waiting real-time mutex
 * (ww_mutex). It supports locking with or without an acquire context (`ww_ctx`).
 * The function handles both fast-path locking (when the mutex is free) and 
 * slow-path locking (when the mutex is already held by another thread).
 */


static int __sched
__ww_rt_mutex_lock(struct ww_mutex *lock, struct ww_acquire_ctx *ww_ctx,
		   unsigned int state, unsigned long ip)
{
	struct lockdep_map __maybe_unused *nest_lock = NULL;
	struct rt_mutex *rtm = &lock->base;
	int ret;

	might_sleep();

	if (ww_ctx) {
		if (unlikely(ww_ctx == READ_ONCE(lock->ctx)))
			return -EALREADY;

		/*
		 * Reset the wounded flag after a kill. No other process can
		 * race and wound us here, since they can't have a valid owner
		 * pointer if we don't have any locks held.
		 */
		if (ww_ctx->acquired == 0)
			ww_ctx->wounded = 0;

#ifdef CONFIG_DEBUG_LOCK_ALLOC
		nest_lock = &ww_ctx->dep_map;
#endif
	}
	mutex_acquire_nest(&rtm->dep_map, 0, 0, nest_lock, ip);

	if (likely(rt_mutex_cmpxchg_acquire(&rtm->rtmutex, NULL, current))) {
		if (ww_ctx)
			ww_mutex_set_context_fastpath(lock, ww_ctx);
		return 0;
	}

	ret = rt_mutex_slowlock(&rtm->rtmutex, ww_ctx, state);

	if (ret)
		mutex_release(&rtm->dep_map, ip);
	return ret;
}


int __sched
ww_mutex_lock(struct ww_mutex *lock, struct ww_acquire_ctx *ctx)
{
	return __ww_rt_mutex_lock(lock, ctx, TASK_UNINTERRUPTIBLE, _RET_IP_);
}
EXPORT_SYMBOL(ww_mutex_lock);

int __sched
ww_mutex_lock_interruptible(struct ww_mutex *lock, struct ww_acquire_ctx *ctx)
{
	return __ww_rt_mutex_lock(lock, ctx, TASK_INTERRUPTIBLE, _RET_IP_);
}
EXPORT_SYMBOL(ww_mutex_lock_interruptible);

void __sched ww_mutex_unlock(struct ww_mutex *lock)
{
	struct rt_mutex *rtm = &lock->base;

	__ww_mutex_unlock(lock);

	mutex_release(&rtm->dep_map, _RET_IP_);
	__rt_mutex_unlock(&rtm->rtmutex);
}
EXPORT_SYMBOL(ww_mutex_unlock);
/*Safenux-MD5:*/