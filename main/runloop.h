#ifndef __RUNLOOP_H
#define __RUNLOOP_H

typedef void (*runloop_task_t)(void *args);

void runloop_init(void);
void runloop_run(runloop_task_t task, void *args);
void runloop_run_from_isr(runloop_task_t task, void *args);

#endif /* __RUNLOOP_H */
