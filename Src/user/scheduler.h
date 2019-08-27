/**
 * @file scheduler.h
 * @author Peter Medvese (peter.medvesek@gorenje.com)
 * @brief   simpl scheduler for tasks. intent of this scheduler is to replace if(time_is_pas) type of code 
 * @version 0.1
 * @date 2019-08-27
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>

/* dependencies */
#include "app_setup.h"
#include "debug_user.h"


#ifndef SCHEDULER_TIME_PERIOD
    #warning "scheduler time periode is not defined by user, set to default of 1ms !!"
    #define SCHEDULER_TIME_PERIOD       (1u) // 1ms assumed
#endif

typedef struct {
    void        (*pfTask_run) (void);
    uint32_t    task_periode;
    uint32_t    task_elapsed_tm;
}task_t;


typedef struct {
    void (*exe)         (void);
    void (*add_task)    (void (*pfTask)(void), uint32_t periode);

    uint8_t _idle_F; // not used
}scheduler_t;

extern scheduler_t Scheduler;

//void scheduler_init(void);

#endif /* SCHEDULER_H */
