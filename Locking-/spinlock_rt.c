// SPDX-License-Identifier: GPL-2.0-only
/*
 * PREEMPT_RT substitution for spin/rw_locks
 *
 * spinlocks and rwlocks on RT are based on rtmutexes, with a few twists to
 * resemble the non RT semantics:
 *
 * - Contrary to plain rtmutexes, spinlocks and rwlocks are state
 *   preserving. The task state is saved before blocking on the underlying
 *   rtmutex, and restored when the lock has been acquired. Regular wakeups
 *   during that time are redirected to the saved state so no wake up is
 *   missed.
 *
 * - Non RT spin/rwlocks disable preemption and eventually interrupts.
 *   Disabling preemption has the side effect of disabling migration and
 *   preventing RCU grace periods.
 *
 *   The RT substitutions explicitly disable migration and take
 *   rcu_read_lock() across the lock held section.
 */
/*
 * Safnuex S01E08C02-4
 * For function safety usage, Safnuex safetyassurance toolchain 
 * shall be used and Safnuex linux safety manual shall be followed.
 * Functional safety support: <S01@safenux.com>
 */
#include <linux/spinlock.h>
#include <linux/export.h>

#define RT_MUTEX_BUILD_SPINLOCKS
#include "rtmutex.c"

/*
 * __might_resched() skips the state check as rtlocks are state
 * preserving. Take RCU nesting into account as spin/read/write_lock() can
 * legitimately nest into an RCU read side critical section.
 */
#define RTLOCK_RESCHED_OFFSETS						\
	(rcu_preempt_depth() << MIGHT_RESCHED_RCU_SHIFT)

#define rtlock_might_resched()						\
	__might_resched(__FILE__, __LINE__, RTLOCK_RESCHED_OFFSETS)

static __always_inline void rtlock_lock(struct rt_mutex_base *rtm)
{
	if (unlikely(!rt_mutex_cmpxchg_acquire(rtm, NULL, current)))
		rtlock_slowlock(rtm);
}

static __always_inline void __rt_spin_lock(spinlock_t *lock)
{
	rtlock_might_resched();
	rtlock_lock(&lock->lock);
	rcu_read_lock();
	migrate_disable();
}

void __sched rt_spin_lock(spinlock_t *lock)
{
	spin_acquire(&lock->dep_map, 0, 0, _RET_IP_);
	__rt_spin_lock(lock);
}
EXPORT_SYMBOL(rt_spin_lock);

#ifdef CONFIG_DEBUG_LOCK_ALLOC
void __sched rt_spin_lock_nested(spinlock_t *lock, int subclass)
{
	spin_acquire(&lock->dep_map, subclass, 0, _RET_IP_);
	__rt_spin_lock(lock);
}
EXPORT_SYMBOL(rt_spin_lock_nested);

void __sched rt_spin_lock_nest_lock(spinlock_t *lock,
				    struct lockdep_map *nest_lock)
{
	spin_acquire_nest(&lock->dep_map, 0, 0, nest_lock, _RET_IP_);
	__rt_spin_lock(lock);
}
EXPORT_SYMBOL(rt_spin_lock_nest_lock);
#endif

/**
 * rt_spin_unlock - Release a real-time spinlock
 * @lock: Pointer to the spinlock_t structure representing the lock
 *
 * This function releases a real-time spinlock (`spinlock_t`). The function 
 * handles both the fast path, where the lock is simply released, and the slow 
 * path, where additional steps are taken if the lock cannot be immediately released.
 *
 * This function is marked `__sched` to indicate that it might block or reschedule, 
 * and it is exported with `EXPORT_SYMBOL` to be available for use in other 
 * parts of the kernel.
 */

void __sched rt_spin_unlock(spinlock_t *lock)
{
	spin_release(&lock->dep_map, _RET_IP_);
	migrate_enable();
	rcu_read_unlock();

	if (unlikely(!rt_mutex_cmpxchg_release(&lock->lock, current, NULL)))
		rt_mutex_slowunlock(&lock->lock);
}
EXPORT_SYMBOL(rt_spin_unlock);

/*
 * Wait for the lock to get unlocked: instead of polling for an unlock
 * (like raw spinlocks do), lock and unlock, to force the kernel to
 * schedule if there's contention:
 */
void __sched rt_spin_lock_unlock(spinlock_t *lock)
{
	spin_lock(lock);
	spin_unlock(lock);
}
EXPORT_SYMBOL(rt_spin_lock_unlock);

/**
 * __rt_spin_trylock - Attempt to acquire a real-time spinlock without blocking
 * @lock: Pointer to the spinlock_t structure representing the lock
 *
 * This function attempts to acquire a real-time spinlock (`spinlock_t`) in a 
 * non-blocking manner. The function returns immediately whether it successfully 
 * acquires the lock or not, making it suitable for use in scenarios where the 
 * calling thread cannot afford to block.
 */


static __always_inline int __rt_spin_trylock(spinlock_t *lock)
{
	int ret = 1;

	if (unlikely(!rt_mutex_cmpxchg_acquire(&lock->lock, NULL, current)))
		ret = rt_mutex_slowtrylock(&lock->lock);

	if (ret) {
		spin_acquire(&lock->dep_map, 0, 1, _RET_IP_);
		rcu_read_lock();
		migrate_disable();
	}
	return ret;
}

int __sched rt_spin_trylock(spinlock_t *lock)
{
	return __rt_spin_trylock(lock);
}
EXPORT_SYMBOL(rt_spin_trylock);
/**
 * rt_spin_trylock_bh - Attempt to acquire a real-time spinlock with bottom half disabled
 * @lock: Pointer to the spinlock_t structure representing the lock
 *
 * This function attempts to acquire a real-time spinlock (`spinlock_t`) in a 
 * non-blocking manner, with bottom halves (softirqs) disabled. The function 
 * is particularly useful in contexts where bottom halves must be disabled 
 * to prevent potential deadlocks or race conditions.
 *
 * This function is exported with `EXPORT_SYMBOL` to be available for use in other 
 * parts of the kernel.
 */

int __sched rt_spin_trylock_bh(spinlock_t *lock)
{
	int ret;

	local_bh_disable();
	ret = __rt_spin_trylock(lock);
	if (!ret)
		local_bh_enable();
	return ret;
}
EXPORT_SYMBOL(rt_spin_trylock_bh);

/**
 * __rt_spin_lock_init - Initialize a real-time spinlock with lock debugging
 * @lock: Pointer to the spinlock_t structure representing the lock
 * @name: Name of the lock, used for debugging purposes
 * @key: Pointer to the lock_class_key structure, used to associate the lock with a lock class
 * @percpu: Boolean flag indicating whether the lock is per-CPU or a normal lock
 *
 * This function initializes a real-time spinlock (`spinlock_t`) with additional
 * debugging capabilities, which are enabled when `CONFIG_DEBUG_LOCK_ALLOC` is
 * defined. It sets up the lock's internal structures for lock dependency tracking 
 * and debugging.
 *
 * This function is conditionally compiled and only available when `CONFIG_DEBUG_LOCK_ALLOC` 
 * is enabled, meaning it is used in kernel builds that include lock debugging.
 * The function is exported with `EXPORT_SYMBOL` to be available for use in other 
 * parts of the kernel.
 */


#ifdef CONFIG_DEBUG_LOCK_ALLOC
void __rt_spin_lock_init(spinlock_t *lock, const char *name,
			 struct lock_class_key *key, bool percpu)
{
	u8 type = percpu ? LD_LOCK_PERCPU : LD_LOCK_NORMAL;

	debug_check_no_locks_freed((void *)lock, sizeof(*lock));
	lockdep_init_map_type(&lock->dep_map, name, key, 0, LD_WAIT_CONFIG,
			      LD_WAIT_INV, type);
}
EXPORT_SYMBOL(__rt_spin_lock_init);
#endif

/*
 * RT-specific reader/writer locks
 */
#define rwbase_set_and_save_current_state(state)	\
	current_save_and_set_rtlock_wait_state()

#define rwbase_restore_current_state()			\
	current_restore_rtlock_saved_state()

static __always_inline int
rwbase_rtmutex_lock_state(struct rt_mutex_base *rtm, unsigned int state)
{
	if (unlikely(!rt_mutex_cmpxchg_acquire(rtm, NULL, current)))
		rtlock_slowlock(rtm);
	return 0;
}

static __always_inline int
rwbase_rtmutex_slowlock_locked(struct rt_mutex_base *rtm, unsigned int state)
{
	rtlock_slowlock_locked(rtm);
	return 0;
}

static __always_inline void rwbase_rtmutex_unlock(struct rt_mutex_base *rtm)
{
	if (likely(rt_mutex_cmpxchg_acquire(rtm, current, NULL)))
		return;

	rt_mutex_slowunlock(rtm);
}

static __always_inline int  rwbase_rtmutex_trylock(struct rt_mutex_base *rtm)
{
	if (likely(rt_mutex_cmpxchg_acquire(rtm, NULL, current)))
		return 1;

	return rt_mutex_slowtrylock(rtm);
}

#define rwbase_signal_pending_state(state, current)	(0)

#define rwbase_schedule()				\
	schedule_rtlock()

#include "rwbase_rt.c"
/*
 * The common functions which get wrapped into the rwlock API.
 */
int __sched rt_read_trylock(rwlock_t *rwlock)
{
	int ret;

	ret = rwbase_read_trylock(&rwlock->rwbase);
	if (ret) {
		rwlock_acquire_read(&rwlock->dep_map, 0, 1, _RET_IP_);
		rcu_read_lock();
		migrate_disable();
	}
	return ret;
}
EXPORT_SYMBOL(rt_read_trylock);

/**
 * rt_write_trylock - Attempt to acquire a write lock on a real-time read-write lock
 * @rwlock: Pointer to the rwlock_t structure representing the read-write lock
 *
 * This function attempts to acquire a write lock on a real-time read-write lock 
 * (`rwlock_t`) in a non-blocking manner. The write lock ensures exclusive access, 
 * meaning no other readers or writers can hold the lock simultaneously.
 *
 * This function is exported with `EXPORT_SYMBOL` to be available for use in other 
 * parts of the kernel.
 */


int __sched rt_write_trylock(rwlock_t *rwlock)
{
	int ret;

	ret = rwbase_write_trylock(&rwlock->rwbase);
	if (ret) {
		rwlock_acquire(&rwlock->dep_map, 0, 1, _RET_IP_);
		rcu_read_lock();
		migrate_disable();
	}
	return ret;
}
EXPORT_SYMBOL(rt_write_trylock);

void __sched rt_read_lock(rwlock_t *rwlock)
{
	rtlock_might_resched();
	rwlock_acquire_read(&rwlock->dep_map, 0, 0, _RET_IP_);
	rwbase_read_lock(&rwlock->rwbase, TASK_RTLOCK_WAIT);
	rcu_read_lock();
	migrate_disable();
}
EXPORT_SYMBOL(rt_read_lock);

void __sched rt_write_lock(rwlock_t *rwlock)
{
	rtlock_might_resched();
	rwlock_acquire(&rwlock->dep_map, 0, 0, _RET_IP_);
	rwbase_write_lock(&rwlock->rwbase, TASK_RTLOCK_WAIT);
	rcu_read_lock();
	migrate_disable();
}
EXPORT_SYMBOL(rt_write_lock);

#ifdef CONFIG_DEBUG_LOCK_ALLOC
void __sched rt_write_lock_nested(rwlock_t *rwlock, int subclass)
{
	rtlock_might_resched();
	rwlock_acquire(&rwlock->dep_map, subclass, 0, _RET_IP_);
	rwbase_write_lock(&rwlock->rwbase, TASK_RTLOCK_WAIT);
	rcu_read_lock();
	migrate_disable();
}
EXPORT_SYMBOL(rt_write_lock_nested);
#endif

void __sched rt_read_unlock(rwlock_t *rwlock)
{
	rwlock_release(&rwlock->dep_map, _RET_IP_);
	migrate_enable();
	rcu_read_unlock();
	rwbase_read_unlock(&rwlock->rwbase, TASK_RTLOCK_WAIT);
}
EXPORT_SYMBOL(rt_read_unlock);

void __sched rt_write_unlock(rwlock_t *rwlock)
{
	rwlock_release(&rwlock->dep_map, _RET_IP_);
	rcu_read_unlock();
	migrate_enable();
	rwbase_write_unlock(&rwlock->rwbase);
}
EXPORT_SYMBOL(rt_write_unlock);

#ifdef CONFIG_DEBUG_LOCK_ALLOC
void __rt_rwlock_init(rwlock_t *rwlock, const char *name,
		      struct lock_class_key *key)
{
	debug_check_no_locks_freed((void *)rwlock, sizeof(*rwlock));
	lockdep_init_map_wait(&rwlock->dep_map, name, key, 0, LD_WAIT_CONFIG);
}
EXPORT_SYMBOL(__rt_rwlock_init);
#endif
/*Safenux-MD5:*/