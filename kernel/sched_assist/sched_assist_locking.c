#include <linux/version.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <trace/events/sched.h>
#include <../kernel/sched/sched.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <../fs/proc/internal.h>
#include <linux/sched/signal.h>
#include "sched_assist_common.h"

#include <linux/mm.h>
#include <linux/rwsem.h>


#define RQ_LOCKING_TASK_LIMIT (5)
#define KSWAPD_COMM "kswapd0"
static int g_kswapd_pid = -1;

bool task_skip_protect(struct task_struct *p)
{
	return g_kswapd_pid == p->pid;
}

static void locking_protect_skip_process(void)
{
	static bool get_success = false;
	struct task_struct *p;

	if (get_success)
		return;

	rcu_read_lock();
	for_each_process(p) {
		if (p->flags & PF_KTHREAD) {
			if (!strncmp(p->comm, KSWAPD_COMM,
				sizeof(KSWAPD_COMM) - 1)) {
				g_kswapd_pid = p->pid;
				break;
			}
		}
	}
	rcu_read_unlock();
	get_success = true;
}

void locking_init_rq_data(struct rq *rq)
{
	locking_protect_skip_process();
	if (!rq) {
		return;
	}

	INIT_LIST_HEAD(&rq->locking_thread_list);
	rq->rq_locking_task = 0;
	rq->rq_picked_locking_cont = 0;
}

void record_locking_info(struct task_struct *p, unsigned long settime)
{
	if (settime) {
		p->locking_depth++;
	} else {
		if (p->locking_depth > 0) {
			if (--(p->locking_depth))
				return;
		}
	}
	p->locking_time_start = settime;
}
EXPORT_SYMBOL_GPL(record_locking_info);

void clear_locking_info(struct task_struct *p)
{
	p->locking_time_start = 0;
}

bool task_inlock(struct task_struct *p)
{
	return p->locking_time_start > 0;
}

bool locking_protect_outtime(struct task_struct *p)
{
	return time_after(jiffies, p->locking_time_start);
}

void enqueue_locking_thread(struct rq *rq, struct task_struct *p)
{
	struct list_head *pos, *n;
	bool exist = false;

	if (!rq || !p || !list_empty(&p->locking_entry)) {
		return;
	}

	p->enqueue_time = rq->clock;

	if (p->locking_time_start) {
		list_for_each_safe(pos, n, &rq->locking_thread_list) {
			if (pos == &p->locking_entry) {
				exist = true;
				break;
			}
		}
		if (!exist) {
			list_add_tail(&p->locking_entry, &rq->locking_thread_list);
			rq->rq_locking_task++;
			get_task_struct(p);
		}
	}
}

void dequeue_locking_thread(struct rq *rq, struct task_struct *p)
{
	struct list_head *pos, *n;

	if (!rq || !p) {
		return;
	}
	p->enqueue_time = 0;
	if (!list_empty(&p->locking_entry)) {
		list_for_each_safe(pos, n, &rq->locking_thread_list) {
			if (pos == &p->locking_entry) {
				list_del_init(&p->locking_entry);
				task_rq(p)->rq_locking_task--;
				put_task_struct(p);
				return;
			}
		}
	}
}

void pick_locking_thread(struct rq *rq, struct task_struct **p, struct sched_entity **se)
{
	struct task_struct *ori_p = *p;
	struct task_struct *key_task;
	struct sched_entity *key_se;

	if (!rq || !ori_p || !se)
		return;

	if (test_task_ux(*p))
		goto pick_locking_fail;

	if (rq->rq_picked_locking_cont >= RQ_LOCKING_TASK_LIMIT)
		goto pick_locking_fail;

pick_again:
	if (!list_empty(&rq->locking_thread_list)) {
		key_task = list_first_entry_or_null(&rq->locking_thread_list,
					struct task_struct, locking_entry);
		if (key_task) {
			list_del_init(&key_task->locking_entry);
			rq->rq_locking_task--;
			put_task_struct(key_task);
			if (task_inlock(key_task)) {
				key_se = &key_task->se;
				if (key_se) {
					*p = key_task;
					*se = key_se;
					rq->rq_picked_locking_cont++;
					return;
				}
			}
			goto pick_again;
		}
	}
pick_locking_fail:
	if (rq->rq_picked_locking_cont > 0)
		rq->rq_picked_locking_cont -= 1;
}
