/* Descrviption: low-end device scheduler
 * Author: jianghongliang1@xiaomi.com
 * Version: 1.0
 * Date:  2023/03/02
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define pr_fmt(fmt) "gaea: " fmt
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/miscdevice.h>
#include <linux/security.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/pm_qos.h>
#include <linux/cred.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched/signal.h>
#include <linux/list.h>
#include <linux/syscore_ops.h>
#include <trace/hooks/sched.h>
#include "../../../kernel/sched/sched.h"
#include "../include/mi_module.h"
#include <linux/sched/walt.h>
//#include "gaea.h"
#include <linux/rbtree.h>
#include <linux/stop_machine.h>
#include <trace/hooks/dtask.h>
#include <trace/hooks/rwsem.h>
#include <trace/hooks/binder.h>
#include <../../android/binder_internal.h>

enum gaea_cmd {
	UI_GET_VTASK_TOKEN,
	REN_GET_VTASK_TOKEN,
	MI_DYN_VTASK_TOKEN,
	UI_RELEASE_VTASK_TOKEN,
	REN_RELEASE_VTASK_TOKEN,
	MI_DYN_RELEASE_VTASK_TOKEN,
	FRAME_BOOST,
	TASK_BOOST_REQ,
	GAEA_CMD_TYPES,
};

enum MI_TASK_TYPE {
    MI_NORMAL_TASK,
    MI_MINOR_TASK,
    MI_REBIND_TASK,
    MI_32BIT_TASK,
    MI_LAT_TASK,
    MI_VVIP_TASK,
    MI_NEW_VTASK,
    MI_AGING_VTASK,
    MI_UI_TASK,
    MI_REN_TASK,
    MI_IN_RTBOOST,
    MI_LOCK_TASK,
    MI_BINDER_TASK,
    MI_TASK_TYPES
};

#define MASK_MI_VVTASK      (1 << MI_VVIP_TASK)
#define MASK_MI_BINDER_TASK (1 << MI_BINDER_TASK)
#define MASK_MI_LOCK_TASK	(1 << MI_LOCK_TASK)

#define MUTEX_FLAGS		   0x07
#define RWSEM_OWNER_FLAGS_MASK     (1UL << 0 | 1UL << 1 | 1UL << 2)

#define mi_time_after(a,b)	\
	 (((long)(b) - (long)(a)) < 0)
#define mi_time_before(a,b) mi_time_after(b,a)

static int mi_gaea_enable = 0;
module_param(mi_gaea_enable, uint, 0644);
static int gaea_debug = 0;
module_param(gaea_debug, uint, 0644);

static struct proc_dir_entry *proc_gaea_cpurq_file;
static unsigned long mi_gaea_cpurq[8] = {0,};
static int foreground_uid = -1;
static int foreground_pid = -1;
static int fg_render_tid = -1;

struct walt_get_indicies_hooks mi_walt_get_indicies_func[WALT_CFS_TYPES];
EXPORT_SYMBOL_GPL(mi_walt_get_indicies_func);

struct walt_lb_pull_tasks_hooks mi_walt_lb_pull_tasks_func[WALT_CFS_TYPES];
EXPORT_SYMBOL_GPL(mi_walt_lb_pull_tasks_func);

int get_foreground_uid(void)
{
	return foreground_uid;
}
EXPORT_SYMBOL_GPL(get_foreground_uid);

void set_foreground_app(int uid)
{
	if (unlikely(gaea_debug))
		pr_info("Change foreground uid from %d to %d\n",
				foreground_uid, uid);
	foreground_uid = uid;
}
EXPORT_SYMBOL_GPL(set_foreground_app);

static inline bool foreground_task(struct task_struct *p)
{
	int uid = from_kuid(&init_user_ns, task_uid(p));

	if (likely(foreground_uid))
		return uid == foreground_uid;

	return false;
}

static inline bool mi_lock_vip_task(struct task_struct *p)
{
	if (unlikely(!p))
		return false;

	return p->android_oem_data1[0] & MASK_MI_LOCK_TASK;
}

static inline bool mi_binder_vip_task(struct task_struct *p)
{
	if (unlikely(!p))
		return false;

	return p->android_oem_data1[0] & MASK_MI_BINDER_TASK;
}

bool mi_vip_task(struct task_struct *p)
{
	if (unlikely(!p))
		return false;

	if (!foreground_task(p))
		return false;

	if (mi_time_before(jiffies, p->android_oem_data1[1] + HZ)) {
		if (unlikely(gaea_debug))
			pr_info("current tsk %d flag is %d", p->pid, p->android_oem_data1[0]);

		return p->android_oem_data1[0] & MASK_MI_VVTASK;
	} else {
		p->android_oem_data1[0] &= ~MASK_MI_VVTASK; // clean vip flag
		p->android_oem_data1[1] = 0; // touch_jif
	}

	return false;
}
EXPORT_SYMBOL_GPL(mi_vip_task);

static inline void mi_set_vip_task(struct task_struct *p)
{
	if (!p)
		return;

	if (!foreground_task(p))
		return;

	p->android_oem_data1[0] |= (1 << MI_VVIP_TASK);
	p->android_oem_data1[1] = jiffies;

	if (unlikely(gaea_debug))
		pr_info("tsk %d flag is %d, start jif: %lu", p->pid, p->android_oem_data1[0], p->android_oem_data1[1]);
}

static inline struct task_struct *__mutex_owner(struct mutex *lock)
{
	return (struct task_struct *)(atomic_long_read(&lock->owner) & ~MUTEX_FLAGS);
}

static inline struct task_struct *rwsem_owner(struct rw_semaphore *sem)
{
	return (struct task_struct *)
		(atomic_long_read(&sem->owner) & ~RWSEM_OWNER_FLAGS_MASK);
}

static void mi_mutex_list_add_hook(void *nouse, struct mutex *lock,
		struct mutex_waiter *waiter, struct list_head *list,
		bool * already_on_list)
{
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	struct mutex_waiter *waiter_temp = NULL;
	bool is_vip_temp, is_lock_vip_temp;

	if (!mi_gaea_enable || !lock || !waiter || !list)
		return;

	if (!mi_vip_task(current) && !mi_lock_vip_task(current))
		return;

	list_for_each_safe(pos, n, list) {
		if (!pos)
			return;

		waiter_temp = list_entry(pos, struct mutex_waiter, list);
		if (!waiter_temp || !waiter_temp->task)
			return;

		is_vip_temp = mi_vip_task(waiter_temp->task);
		is_lock_vip_temp = mi_lock_vip_task(waiter_temp->task);
		if (!is_vip_temp && !is_lock_vip_temp) {
			list_add(&waiter->list, waiter_temp->list.prev);
			*already_on_list = true;
			return;
		}
	}

	if (pos == list) {
		list_add_tail(&waiter->list, list);
		*already_on_list = true;
		return;
	}
}

static bool mi_task_exiting(struct task_struct *p)
{
	if (unlikely(!p))
		return true;

	if (unlikely(p->state == TASK_DEAD))
		return true;

	if (unlikely(p->exit_state))
		return true;

	if (unlikely(refcount_read(&p->usage) == 0))
		return true;

	return false;
}

static void do_set_lock_vip(struct task_struct *task)
{
	if (!task || (!mi_vip_task(current) && !mi_lock_vip_task(current)))
		return;

	if (!mi_lock_vip_task(task)) {
		if (unlikely(gaea_debug))
			pr_info ("lock_tsk->pid:%d, current->pid:%d", task->pid, current->pid);
		task->android_oem_data1[0] |= (1 << MI_LOCK_TASK);
	}
}

static void mi_mutex_wait_start_hook(void *nouse, struct mutex *lock)
{
	bool is_owner_exit = false;
	struct task_struct *lock_tsk;

	if (!mi_gaea_enable)
		return;

	rcu_read_lock();
	lock_tsk = __mutex_owner(lock);
	is_owner_exit = mi_task_exiting(lock_tsk);
	if (is_owner_exit) {
		rcu_read_unlock();
		return ;
	}

	do_set_lock_vip(lock_tsk);
	rcu_read_unlock();
}

static bool is_rt_policy(int policy)
{
	return policy == SCHED_FIFO || policy == SCHED_RR;
}

static void mi_after_dequeue_task_hook(void *nouse, struct rq *rq, struct task_struct *task)
{
	if (mi_lock_vip_task(task) && is_rt_policy(task->policy)) {
		if (unlikely(gaea_debug))
			pr_info("pid:%d is cleaning lock vip flag", task->pid);

		task->android_oem_data1[0] &= (~(1 << MI_LOCK_TASK));
	}
}

void do_set_binder_vip(struct binder_transaction *t, struct task_struct *task)
{
	struct task_struct *from_task;
	struct binder_thread *bthread;
	bool is_vip;

	struct sched_param param = {.sched_priority = 1};
	bthread = t->from;
	if (unlikely(!bthread))
		return;

	from_task = bthread->task;
	if (unlikely(!from_task))
		return;

	is_vip = mi_vip_task(from_task);

	if (is_vip && !is_rt_policy(task->policy) && (!(t->flags & TF_ONE_WAY))) {
		if (!task)
			return;

		if (unlikely(gaea_debug))
			pr_info ("vip binder tsk :%d:%s", task->pid, task->comm);

		task->android_oem_data1[0] |= (1 << MI_BINDER_TASK);
		sched_setscheduler_nocheck(task, SCHED_FIFO, &param);
	}
}

static void mi_binder_set_vip_hook(void *nouse, struct binder_transaction *t, struct task_struct *task)
{
	if (!mi_gaea_enable)
		return;

	rcu_read_lock();
	if (!t || !task){
		rcu_read_unlock();
		return;
	}

	do_set_binder_vip(t, task);
	rcu_read_unlock();

	return;
}

static void mi_binder_restore_vip_hook(void *nouse, struct binder_transaction *in_reply_to, struct task_struct *task)
{
	if (!mi_gaea_enable)
		return;

	if (unlikely(!task))
		return;

	if (mi_binder_vip_task(task)) {
		if (unlikely(gaea_debug))
			pr_info("pid:%d is cleaning binder vip flag", task->pid);

		task->android_oem_data1[0] &= (~(1 << MI_BINDER_TASK));
	}
}

void set_foreground_pid(int pid)
{
	if (unlikely(gaea_debug))
		pr_info("Change foreground pid from %d to %d\n",
				foreground_pid, pid);
	foreground_pid = pid;
}

void set_fg_render_tid(int tid)
{
	if (unlikely(gaea_debug))
		pr_info("Change foreground render from %d to %d\n",
				fg_render_tid, tid);
	fg_render_tid = tid;
}

static inline bool gaea_ui_task(int pid)
{
	if (pid == foreground_pid)
		return true;
	return false;
}

static inline bool gaea_ren_task(int tid)
{
	if (tid == fg_render_tid)
		return true;

	return false;
}

static inline bool gaea_fg_minor_task(int tgid)
{
	if (tgid == foreground_pid)
		return true;

	return false;
}

static long gaea_ioctl(struct file *fp, unsigned int cmd,
				 unsigned long arg)
{
	int traced_uid;
	int user_cmd = _IOC_NR(cmd);

	if (!mi_gaea_enable)
		return 0;

	traced_uid = from_kuid(&init_user_ns, current_uid());
	if (gaea_debug)
		pr_err("gaea comming. uid %d, pid %d, %d %d %llu",
			traced_uid, current->pid, __LINE__, user_cmd,
			ktime_to_us(ktime_get()));

	switch (user_cmd) {
	case UI_GET_VTASK_TOKEN:
		set_foreground_app(traced_uid);
		//Focus
		set_foreground_pid(current->pid);
		mi_set_vip_task(current);
		break;

	case REN_GET_VTASK_TOKEN:
		set_fg_render_tid(current->pid);
		mi_set_vip_task(current);
		break;

	case UI_RELEASE_VTASK_TOKEN:
		break;

	case REN_RELEASE_VTASK_TOKEN:
		break;

	default:
		break;
	}

	return 0;
}

static int gaea_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int gaea_release(struct inode *ignored, struct file *file)
{
	return 0;
}

static int gaea_mmap(struct file *file, struct vm_area_struct *vma)
{
	return 0;
}

static const struct file_operations gaea_fops = {
	.owner = THIS_MODULE,
	.open = gaea_open,
	.release = gaea_release,
	.mmap = gaea_mmap,
	.unlocked_ioctl = gaea_ioctl,
	.compat_ioctl = gaea_ioctl,
};

static struct miscdevice gaea_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "metis",
	.fops = &gaea_fops,
};

void register_mi_walt_get_indicies(int mod, mi_walt_get_indicies f)
{
	if (mod >= WALT_CFS_TYPES) {
		pr_err("invalid mod register\n");
		return;
	}
	pr_info("%s now\n",  __FUNCTION__);
	mi_walt_get_indicies_func[mod].f = f;
}
EXPORT_SYMBOL_GPL(register_mi_walt_get_indicies);

void unregister_mi_walt_get_indicies(int mod)
{
	if (mod >= WALT_CFS_TYPES) {
		pr_err("invalid mod unregister\n");
		return;
	}
	pr_info("%s now\n",  __FUNCTION__);
	mi_walt_get_indicies_func[mod].f = NULL;
}
EXPORT_SYMBOL_GPL(unregister_mi_walt_get_indicies);

void register_mi_walt_lb_pull_tasks(int mod, mi_walt_lb_pull_tasks f)
{
	if (mod >= WALT_CFS_TYPES) {
		pr_err("invalid mod register\n");
		return;
	}
	pr_info("%s now\n",  __FUNCTION__);
	mi_walt_lb_pull_tasks_func[mod].f = f;
}
EXPORT_SYMBOL_GPL(register_mi_walt_lb_pull_tasks);

void unregister_mi_walt_lb_pull_tasks(int mod)
{
	if (mod >= WALT_CFS_TYPES) {
		pr_err("invalid mod unregister\n");
		return;
	}
	pr_info("%s now\n",  __FUNCTION__);
	mi_walt_lb_pull_tasks_func[mod].f = NULL;
}
EXPORT_SYMBOL_GPL(unregister_mi_walt_lb_pull_tasks);

static inline bool system_process(struct task_struct *p)
{
    int uid = from_kuid(&init_user_ns, task_uid(p));

    return (uid == 1000);
}

static inline bool kernel_process(struct task_struct *p)
{
    int uid = from_kuid(&init_user_ns, task_uid(p));

    return (uid == 0);
}

static inline bool application_process(struct task_struct *p)
{
    int uid = from_kuid(&init_user_ns, task_uid(p));

    return (uid >= 10000);
}

static void walt_lb_pull_tasks_hook(struct task_struct *p,
				int dst_cpu, int src_cpu, bool *skip_lb)
{
	if (!mi_gaea_enable)
		return;

	if (!p)
		return;

	if (gaea_ui_task(p->pid) || gaea_ren_task(p->pid)) {
		if (is_rt_policy(p->policy)) {
			if (capacity_orig_of(dst_cpu) < capacity_orig_of(src_cpu)) {
				*skip_lb= true;
				return;
			}
		}
	}

	if (mi_binder_vip_task(p) || mi_lock_vip_task(p)) {
		*skip_lb = false;
		return;
	}

	if (system_process(p) && !strcmp(p->comm, "surfaceflinger")) {
		*skip_lb = false;
		return;
	}

	if (system_process(p) && !strcmp(p->comm, "system_server")) {
		*skip_lb = false;
		return;
	}

	// Focus
	if (gaea_fg_minor_task(p->tgid)) {
		*skip_lb = false;
		return;
	}

	if (capacity_orig_of(dst_cpu) > capacity_orig_of(src_cpu)) {
		*skip_lb = true;
		return;
	}
}

static void walt_get_indicies_hook(struct task_struct *p,
				int *order_index, int *end_index,
				int num_sched_clusters, bool *check_return)
{
	if (!mi_gaea_enable)
		return;

	if (unlikely(is_compat_thread(task_thread_info(p)))) {
		*check_return = false;
		return;
	}

	if (gaea_ui_task(p->pid)) {
		*order_index = 1;
		*end_index = 1;
		*check_return = true;
		return;
	}

	if (gaea_ren_task(p->pid)) {
		*order_index = 1;
		*end_index = 1;
		*check_return = true;
		return;
	}

	if (gaea_fg_minor_task(p->tgid)) {
		if (!strcmp(p->comm, "Vending-LogicTh")) {
			*order_index = 1;
			*end_index = 1;
			*check_return = true;
			return;
		}
	}

	if (gaea_fg_minor_task(p->tgid)) {
		if (strstr(p->comm, "VOutlet-V")) {
			if (gaea_debug)
				pr_info("Focus: make %d:%s go to big core!\n", p->pid, p->comm);

			*order_index = 1;
			*end_index = 1;
			*check_return = true;
			return;
		}
	}

	return;
}

static int proc_show_gaea_cpurq(struct seq_file *m, void *v)
{
#define BUF_LEN 100

	int i, cnt = 0;
	char buf[BUF_LEN];

	memset(buf, 0, sizeof(char) * 100);

	for (i = 0; i < 8; i++)
		cnt += snprintf(buf + cnt, BUF_LEN - cnt, "cpu%d:%lu ", i, mi_gaea_cpurq[i]);

	cnt += snprintf(buf + cnt, BUF_LEN - cnt, "\n");

	seq_printf(m, "%s", buf);

	return 0;
}

static int proc_open_gaea_cpurq(struct inode *inode, struct file *file)
{
	return single_open(file, proc_show_gaea_cpurq, NULL);
}

static const struct proc_ops proc_gaea_cpurq_op = {
	.proc_open = proc_open_gaea_cpurq,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
	.proc_write = NULL,
};

static int __init gaea_init(void)
{
	int ret;

	proc_gaea_cpurq_file = proc_create("gaea_cpurq", 0644, NULL, &proc_gaea_cpurq_op);
	if (!proc_gaea_cpurq_file)
		pr_err("proc_fas file create failed.\n");

	register_mi_walt_get_indicies(GAEA_TASK, walt_get_indicies_hook);
	register_mi_walt_lb_pull_tasks(GAEA_TASK, walt_lb_pull_tasks_hook);

	// binder vip
	register_trace_android_vh_binder_restore_priority(mi_binder_restore_vip_hook, NULL);
	register_trace_android_vh_binder_set_priority(mi_binder_set_vip_hook, NULL);
	register_trace_android_vh_alter_mutex_list_add(mi_mutex_list_add_hook, NULL);
	register_trace_android_vh_mutex_wait_start(mi_mutex_wait_start_hook, NULL);
	register_trace_android_rvh_after_dequeue_task(mi_after_dequeue_task_hook, NULL);

	ret = misc_register(&gaea_misc);

	return 0;
}

static void __exit gaea_exit()
{
	if (proc_gaea_cpurq_file)
		proc_remove(proc_gaea_cpurq_file);

	unregister_mi_walt_get_indicies(GAEA_TASK);
	unregister_mi_walt_lb_pull_tasks(GAEA_TASK);
	misc_deregister(&gaea_misc);
}

late_initcall(gaea_init);
module_exit(gaea_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("gaea-driver by David");
