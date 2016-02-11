/*!
* CBcomm_rt_gyro_tester.cpp
*
*  Created on: Aug 26, 2009
*      Author: clmc
*/

// system includes
#include <getopt.h>
#include <execinfo.h>

#include <semaphore.h>
#include <pthread.h>
#include <time.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>
#include <rtdm/rtdm.h>
#include <stdlib.h>

// m3dmg includes
#include "m3dmgErrors.h"
#include "m3dmgSerial.h"
#include "m3dmgAdapter.h"
#include "m3dmgUtils.h"
#include "m3dmg.h"
// #include "m3dmg-local.h"

enum MiscSensors {
	B_Q0_IMU = 1,
	B_Q1_IMU,
	B_Q2_IMU,
	B_Q3_IMU,

	B_AD_A_IMU,
	B_AD_B_IMU,
	B_AD_G_IMU,

	B_XACC_IMU,
	B_YACC_IMU,
	B_ZACC_IMU,
};

#define SEC_INTO_NANO_SEC 1000000000
#define NANO_SEC_INTO_SEC ((double)1.0/(double)1000000000)

#define SLOW_BAUD_RATE 19200
#define MIDDLE_BAUD_RATE 38400
#define FAST_BAUD_RATE 115200

#define SERVO_RATE (100)

#define logPrintf rt_printf

// defines
#define STACK_SIZE 100000
#define STD_PRIO 40
RT_TASK rt_gyro_tester_task_ptr;

static double startTime;
static double currentTime;
static int task_period_ns;

// #define MONITOR_TIMING
#define PRINT_OUT_MAX_PACKET 100

#ifdef MONITOR_TIMING
static unsigned long long servo_elapsed_time;
static unsigned long long servo_period_time;
int servo_current_cycle_time;
#endif

// local functions
static void rt_gyro_tester_task_exit(int dummy);
static void rt_gyro_tester_task(void *cookie);

void warn_upon_switch(int sig __attribute__((unused)));

// local variables
RTIME now, start, cur_time;
M3DMG the_m3dmg;

// signal-handler, to ensure clean exit on Ctrl-C
static void rt_gyro_tester_task_exit(int dummy) {
	printf("rt_gyro_tester_task_exit>> cleanup\n");
	shutdown_3dmg(&the_m3dmg);
	rt_task_delete(&rt_gyro_tester_task_ptr);
	printf("rt_gyro_tester_task_exit>> end\n");
	exit(0);
}

void warn_upon_switch(int sig __attribute__((unused))) {
    void *bt[32];
    int nentries;

    // Dump a backtrace of the frame which caused the switch to secondary mode:
    nentries = backtrace(bt,sizeof(bt) / sizeof(bt[0]));
    backtrace_symbols_fd(bt,nentries,fileno(stdout));
}

const char m_smName[] = "SL2IMU_shared_memory";
const char m_semName[] = "sem_SL2IMU_shared_memory";

int cbCommShmFd;
sem_t *cbCommShmSem;

typedef struct smMiscGyroData {
	int flag;
	double values[4+3+3+1];
} smMiscGyroData;
smMiscGyroData *sm_misc_gyro_data;

static void rt_gyro_tester_task(void *cookie) {

	int m3dmg_id;
	int counter;
	unsigned int old_p = 0;

	struct timespec ts;

	logPrintf("\n\n\nStarting CBcommRTGyroTester\n");

		struct timespec ns;
		ns.tv_sec=0;
		ns.tv_nsec=5*1000*1000; // do not pull as fast as you can and wait in between!!

	//SDC to SL connection through shared memory
	int rc;
	cbCommShmFd = shm_open(m_smName, O_RDWR|O_CREAT|O_EXCL, S_IRWXU|S_IRWXG|S_IRWXO);
	if(cbCommShmFd==-1) {
		// TODO: unlink shared memory when exiting...
		cbCommShmFd = shm_open(m_smName, O_RDWR|O_CREAT, S_IRWXU|S_IRWXG|S_IRWXO);
		if(cbCommShmFd==-1) {
			logPrintf("rt_gyro_tester_task>> cannot get pointer to shared mem... return\n");
			return;
		}
	} else {
		// logPrintf("rt_gyro_tester_task>> truncating...\n");
		rc = ftruncate(cbCommShmFd, sizeof(smMiscGyroData));
		if(rc==-1) {
			logPrintf("rt_gyro_tester_task>> truncate failed... return\n");
			return;
		}
	}
	// logPrintf("rt_gyro_tester_task>> mapping %i bytes... \n", sizeof(smMiscGyroData));
	sm_misc_gyro_data = (smMiscGyroData*) mmap(NULL, sizeof(smMiscGyroData), PROT_READ|PROT_WRITE, MAP_SHARED, cbCommShmFd, 0);
	if(sm_misc_gyro_data == MAP_FAILED) {
		logPrintf("rt_gyro_tester_task>> cannot map misc sensors... return\n");
		return;
	}

	// logPrintf("rt_gyro_tester_task>> opening semaphore... \n");
	cbCommShmSem = sem_open(m_semName, O_CREAT|O_EXCL, S_IRWXU|S_IRWXG|S_IRWXO, 1);
	if(cbCommShmSem == SEM_FAILED) {
		// logPrintf("rt_gyro_tester_task>> semaphore already exists, deleting it\n");
		rc = sem_unlink(m_semName);
		if(rc==-1) {
			logPrintf("rt_gyro_tester_task>> cannot unlink, error %d ... return\n", errno);
			return;
		} else {
			cbCommShmSem = sem_open(m_semName, O_CREAT|O_EXCL, S_IRWXU|S_IRWXG|S_IRWXO, 1);
			if(cbCommShmSem == SEM_FAILED) {
				logPrintf("rt_gyro_tester_task>> cannot create sem... return\n");
				return;
			}
		}
	}
	// logPrintf("rt_gyro_tester_task>> semaphore initialization done...\n");

	m3dmg_id = init_3dmg();
	init_gyro(m3dmg_id, &the_m3dmg);

	//	test_3dmg(m3dmg_id);
	//	return;

	clock_gettime(CLOCK_REALTIME,&ts);
	startTime = (double) ts.tv_sec + ((double)ts.tv_nsec)/1.e9;

//	rc = rt_task_set_periodic(NULL, TM_NOW, task_period_ns);
//	if(rc) {
//		printf("rt_gyro_tester_task>> rt_task_set_periodic() failed: code %d\n", rc);
//		rt_task_suspend(NULL);
//	}

	while(true) {

		counter=0;
#ifdef MONITOR_TIMING
		servo_elapsed_time = rt_timer_read();
#endif
		while(counter < PRINT_OUT_MAX_PACKET) {

#ifdef MONITOR_TIMING
			servo_current_cycle_time = (int)(rt_timer_read() - servo_elapsed_time);
#endif

			nanosleep(&ns,NULL);
			// waits until next cycle...
//			rc = rt_task_wait_period(NULL);
//			if(rc) {
//				if(rc == -ETIMEDOUT) {
//					logPrintf("rt_gyro_tester_task>> timer overrun occurred\n");
//				} else {
//					logPrintf("rt_gyro_tester_task>> rt_task_wait_period() failed: code %d\n", rc);
//					rt_task_suspend(NULL);
//				}
//				continue;
//			}

#ifdef MONITOR_TIMING
			servo_period_time = rt_timer_read() - servo_elapsed_time;
			servo_elapsed_time = rt_timer_read();
			logPrintf(">> servo_period_time = %lli \t servo_current_cycle_time = %i\n",servo_period_time,servo_current_cycle_time);
#endif
			// logPrintf("rt_gyro_tester_task>> m3dmg_process_input\n");
			if(m3dmg_process_input(&the_m3dmg) > 0) {

			}

			if (old_p != the_m3dmg.packets) {

				counter++;
				old_p = the_m3dmg.packets;

				// valid = 0;
				if(sem_wait(cbCommShmSem) == 0) {
					// valid = 1;
					sm_misc_gyro_data->flag = 1;
					sm_misc_gyro_data->values[B_Q0_IMU] = the_m3dmg.quaternion[0];
					sm_misc_gyro_data->values[B_Q1_IMU] = the_m3dmg.quaternion[1];
					sm_misc_gyro_data->values[B_Q2_IMU] = the_m3dmg.quaternion[2];
					sm_misc_gyro_data->values[B_Q3_IMU] = the_m3dmg.quaternion[3];

					sm_misc_gyro_data->values[B_AD_A_IMU] = the_m3dmg.angular_velocity[0];
					sm_misc_gyro_data->values[B_AD_B_IMU] = the_m3dmg.angular_velocity[1];
					sm_misc_gyro_data->values[B_AD_G_IMU] = the_m3dmg.angular_velocity[2];

					sm_misc_gyro_data->values[B_XACC_IMU] = the_m3dmg.acceleration[0];
					sm_misc_gyro_data->values[B_YACC_IMU] = the_m3dmg.acceleration[1];
					sm_misc_gyro_data->values[B_ZACC_IMU] = the_m3dmg.acceleration[2];
					sem_post(cbCommShmSem);
				}

				// logPrintf("Time: %g\t #Packets: %d\t #Errors:%d %d %d %d\n", the_m3dmg.time, the_m3dmg.packets, the_m3dmg.errors[0], the_m3dmg.errors[1], the_m3dmg.errors[2], the_m3dmg.errors[3]);
//				logPrintf("Q: %g %g %g %g\n", the_m3dmg.quaternion[0], the_m3dmg.quaternion[1], the_m3dmg.quaternion[2], the_m3dmg.quaternion[3]);
//				logPrintf("M: %g %g %g\n", the_m3dmg.magnetometer[0], the_m3dmg.magnetometer[1], the_m3dmg.magnetometer[2]);
//				logPrintf("A: %g %g %g\n", the_m3dmg.acceleration[0], the_m3dmg.acceleration[1], the_m3dmg.acceleration[2]);
//				logPrintf ("V: %g %g %g\n", the_m3dmg.angular_velocity[0], the_m3dmg.angular_velocity[1], the_m3dmg.angular_velocity[2]);
//				logPrintf ("\n");

			}
		}

		clock_gettime(CLOCK_REALTIME,&ts);
		currentTime = ((double) ts.tv_sec + ((double)ts.tv_nsec)/1.e9) - startTime;

		logPrintf("Total time = %f sec ",currentTime);
		logPrintf("--> Rate = %f Hz\n",(double)PRINT_OUT_MAX_PACKET/currentTime);

		clock_gettime(CLOCK_REALTIME,&ts);
		startTime = (double) ts.tv_sec + ((double)ts.tv_nsec)/1.e9;
	}

	shutdown_3dmg (&the_m3dmg);
	return;
}

/*!
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char *argv[]) {

	// install signal handler
	signal(SIGTERM, rt_gyro_tester_task_exit);
	signal(SIGINT, rt_gyro_tester_task_exit);

	signal(SIGXCPU, warn_upon_switch);

	mlockall(MCL_CURRENT | MCL_FUTURE);

	// initialize real-time printf
	rt_print_auto_init(1);

	// rt_gyro_tester_task(NULL);

	task_period_ns = (((double)1.0) / ((double)SERVO_RATE)) * (int)((double)SEC_INTO_NANO_SEC);
	int err;
	err = rt_task_spawn(&rt_gyro_tester_task_ptr, "gyro_tester_task", STACK_SIZE, STD_PRIO, 0, &rt_gyro_tester_task, NULL);
	if(err) {
		printf("main>> ERROR: rt_task_spawn\n");
		return 0;
	}

	pause();

	return 0;

}

