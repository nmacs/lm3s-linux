#include <linux/types.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/sched.h>

static inline __attribute__((format(printf, 1, 2)))
void no_printk(const char *fmt, ...)
{
}

#if 0
#define kenter(FMT, ...) \
	printk( "==> %s("FMT")\n", __func__, ##__VA_ARGS__)
#define kleave(FMT, ...) \
	printk( "<== %s()"FMT"\n", __func__, ##__VA_ARGS__)
#define kdebug(FMT, ...) \
	printk( "exec_pool (dbg): " FMT"\n", ##__VA_ARGS__)
#else
#define kenter(FMT, ...) \
	no_printk(KERN_DEBUG "==> %s("FMT")\n", __func__, ##__VA_ARGS__)
#define kleave(FMT, ...) \
	no_printk(KERN_DEBUG "<== %s()"FMT"\n", __func__, ##__VA_ARGS__)
#define kdebug(FMT, ...) \
	no_printk(KERN_DEBUG FMT"\n", ##__VA_ARGS__)
#endif

struct execution_slot
{
	size_t size;
	int busy;
	void *ptr;
	struct list_head list;
};

struct exec_pool
{
	void* memory;
	void* free_memory;
	size_t size;
	size_t free_space;
	struct mutex lock;
	struct list_head slots;
};

static struct exec_pool pool;

static struct execution_slot *find_free_slot_by_size(size_t size)
{
	struct execution_slot *slot;
	list_for_each_entry(slot, &pool.slots, list) {
		if( slot->busy == 0 && slot->size == size )
			return slot;
	}
	
	return 0;
}

static struct execution_slot *find_slot_by_ptr(void *ptr)
{
	struct execution_slot *slot;
	list_for_each_entry(slot, &pool.slots, list) {
		if( slot->ptr == ptr )
			return slot;
	}
	
	return 0;
}

static struct execution_slot *allocate_slot(size_t size)
{
	struct execution_slot *slot;
	
	if( pool.free_space < size )
		return 0;
	
	slot = (struct execution_slot *)kmalloc(sizeof(struct execution_slot), GFP_KERNEL);
	if( slot == 0 )
		return 0;
	
	slot->ptr = pool.free_memory;
	slot->size = size;
	slot->busy = 0;
	
	list_add(&slot->list, &pool.slots);
	
	pool.free_memory = (char*)pool.free_memory + size;
	pool.free_space -= size;
	
	kdebug("allocate new slot, free_space=%u", pool.free_space);
	
	return slot;
}

static void free_slot(struct execution_slot* slot)
{
	list_del(&slot->list);
	kfree(slot);
}

static void free_unused_tail_slots(void)
{
	struct execution_slot *slot, *next;

	list_for_each_entry_safe(slot, next, &pool.slots, list) {
		if( slot->busy )
			return;
		
		pool.free_memory = (char*)pool.free_memory - slot->size;
		pool.free_space += slot->size;
		
		if( pool.free_memory != slot->ptr )
			kdebug("!!!!!!!!!!!!!!!!!!!!! ERROR: pool.free_memory != slot->ptr");
		
		kdebug("free tail slot, free_space=%u", pool.free_space);
		
		free_slot(slot);
	}
}

void* exec_pool_allocate(size_t size)
{
	struct execution_slot *slot = 0;
	
	kdebug("allocate %u bytes for process %s", size, current->comm);
	
	mutex_lock(&pool.lock);
	
	slot = find_free_slot_by_size(size);
	if( slot == 0 )
		slot = allocate_slot(size);
	if( slot )
		slot->busy = 1;
	
	mutex_unlock(&pool.lock);
	
	if( slot )
	{
		kdebug("memory allocated at %p", slot->ptr);
		return slot->ptr;
	}
	else
		return 0;
}

void exec_pool_free(void *ptr)
{
	struct execution_slot *slot = 0;
	
	kdebug("free memory of process %s", current->comm);
	
	mutex_lock(&pool.lock);
	
	slot = find_slot_by_ptr(ptr);
	if( slot )
	{
		slot->busy = 0;
		free_unused_tail_slots();
	}
	
	mutex_unlock(&pool.lock);
}

void exec_pool_show_free_space(void)
{
	printk("exec_pool: free_space=%u\n", pool.free_space);
}

/****************************************************************************/

static int __init init_exec_pool(void)
{
	pool.size = CONFIG_NOMMU_EXEC_POOL_SIZE * 1024;
	pool.free_space = pool.size;
	
	INIT_LIST_HEAD(&pool.slots);
	mutex_init(&pool.lock);

	pool.memory = pool.free_memory = kmalloc(pool.size, GFP_ATOMIC);
	if( pool.memory == 0 )
		return -ENOMEM;
	
	return 0;
}

core_initcall(init_exec_pool);

/****************************************************************************/