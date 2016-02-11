#include <native/task.h>
#include <native/timer.h>
#include <sys/mman.h>
#include <cstdio>

#define LOG_SIZE 100000

int main(int argc, char* argv[])
{
	long system_time[LOG_SIZE];
	long work_duration[LOG_SIZE];
	unsigned long overruns[LOG_SIZE];

	mlockall(MCL_CURRENT | MCL_FUTURE);

	rt_task_shadow(NULL, "periodic_thread", 50, 0);
	if(0 != rt_task_set_periodic(NULL, TM_NOW, 0.001 * 1e9))
	{
		printf ("Could not make task periodic\n");
	    return -1;
	}
	rt_task_wait_period(NULL);

	for(int it=0; it<LOG_SIZE; ++it)
	{
	  rt_task_wait_period(&(overruns[it]));
	  system_time[it] = long(rt_timer_ticks2ns(rt_timer_read()));

	  // do something
	  long work_start = long(rt_timer_ticks2ns(rt_timer_read()));
	  rt_task_sleep(400000);
	  work_duration[it] = long(rt_timer_ticks2ns(rt_timer_read())) - work_start;
	}

	printf("writing log...\n");
	FILE *slog_file = fopen("periodic_task.log","w");
	for(int i=0; i<LOG_SIZE; ++i)
	{
	  printf("writing log %d\n", i);
	  fprintf(slog_file, "%ld ", system_time[i]);
	  fprintf(slog_file, "%ld\n", work_duration[i]);
	  fprintf(slog_file, "%lu\n", overruns[i]);
	}
	fclose(slog_file);

	return 0;
}
