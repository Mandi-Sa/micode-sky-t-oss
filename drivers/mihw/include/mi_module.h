#ifndef MI_MODULE_H
#define MI_MODULE_H

typedef void (*mi_walt_get_indicies)(struct task_struct *, int *, int *, int, bool *);

typedef void (*mi_walt_lb_pull_tasks)(struct task_struct *, int dst_cpu, int src_cpu, bool *skip_lb);

enum MI_WALT_CFS_MOD{
	GAEA_TASK,
	WALT_CFS_TYPES,
};

struct walt_get_indicies_hooks {
	mi_walt_get_indicies f;
};

struct walt_lb_pull_tasks_hooks {
	mi_walt_lb_pull_tasks f;
};
#endif
