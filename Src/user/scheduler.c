#include "scheduler.h"



#define TASK_CNT_MAX		(10u)


static task_t tasks_queue[TASK_CNT_MAX] = { {0} };
/* count number of task that was created */
static uint8_t task_cnt = 0;

//=============================================================
// method declaration:
void exe_method				(void);
void add_task_method		(void (*pfTask)(void), uint32_t periode);

void _dummy(void);  // check why is this needed !!
//=============================================================

scheduler_t Scheduler = {
    &exe_method,
    &add_task_method
};

//=============================================================
// methods implementations:

void add_task_method    (void (*pfTask)(void), uint32_t periode) {

    task_t *pTask_slot;

	if (task_cnt < TASK_CNT_MAX)
	{
        pTask_slot = &tasks_queue[task_cnt];

        pTask_slot->task_periode = periode;
        pTask_slot->task_elapsed_tm = task_cnt; // add offset to avoid(minimise) task overlap
        pTask_slot->pfTask_run = pfTask;
		
		++task_cnt;
	} else {
		ASSERT_EMBED(0);  // max task reached
	}
}


void exe_method (void) {

	task_t* pTask;
	uint8_t i = 0;

	for (i = 0; i < task_cnt; ++i)
	{
        pTask = &tasks_queue[i];
        pTask->task_elapsed_tm += SCHEDULER_TIME_PERIOD;

        if (pTask->task_elapsed_tm >= pTask->task_periode) {
            pTask->task_elapsed_tm = 0;
            pTask->pfTask_run();
        }
    }
}

//=============================================================
