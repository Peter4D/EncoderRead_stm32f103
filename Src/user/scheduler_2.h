/*
 * scheduler_2.h
 *
 * Created: 11. 12. 2017 10:19:34
 *  Author: peter.medvesek
 */ 


#ifndef SCHEDULER_2_H_
#define SCHEDULER_2_H_

#include <stdint.h>

typedef struct {
	void (*task_run) (void);
	uint32_t tm_periode;
	uint32_t tm_elapsed;
}task_t;

typedef struct {

}singleShot_t;


typedef struct {
    /* public methods */
	void (*run)				(void);
	void (*task_exe)		(void);
	void (*add_task)		(void (*task)(void), uint32_t periode);
	void (*new_singleShot)	(void(*single_fptr)(void));

    /* private data !*/
	uint8_t _task_active_F; 
	uint8_t _single_active_F;
	uint8_t _active_task_ID;
	
}scheduler_t;

extern scheduler_t Scheduler;


#endif /* SCHEDULER_2_H_ */