// derived from:
// http://www.rts.uni-hannover.de/mitarbeiter/kiszka/rtaddon/
// rtserial documentation:
// http://download.gna.org/xenomai/documentation/html/api/group__rtserial.html

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>

#include <rtdm/rtserial.h>

#define LOG_PREFIX    "rt_serial_uprog: "
#define WTASK_PREFIX  "write_task: "
#define RTASK_PREFIX  "read_task : "

#define RTSER_FILE    "rtser0"
#define READ_FILE     "rtser0"

int timer_started = 0;
int my_fd = -1;

#define STATE_FILE_OPENED         1
#define STATE_TASK_CREATED        2

unsigned int my_state = 0;

//                           --s-ms-us-ns
RTIME write_task_period_ns =   1000000000llu;
RT_TASK write_task;
RT_TASK read_task;

static const struct rtser_config ser_config = {
    0xFFFF,                     /* config_mask */
    9600,                       /* baud_rate */
    RTSER_ODD_PARITY ,          /* parity */
    RTSER_8_BITS ,              /* data_bits */
    RTSER_1_STOPB ,             /* stop_bits */
    RTSER_NO_HAND ,             /* handshake */
    RTSER_DEF_FIFO_DEPTH,       /* fifo_depth*/
    RTSER_DEF_TIMEOUT,          /* rx_timeout */
    RTSER_DEF_TIMEOUT,          /* tx_timeout */
    RTSER_DEF_TIMEOUT,          /* event_timeout */
    RTSER_RX_TIMESTAMP_HISTORY, /* timestamp_history */
    RTSER_EVENT_RXPEND          /* event mask */
};

static int close_file( int fd, unsigned char *name) {
  int ret,i=0;
  do {
    i++;
    ret = rt_dev_close(fd);
    switch(-ret){
      case EBADF:   printf(LOG_PREFIX "%s -> invalid fd or context\n",name);
                    break;
      case EAGAIN:  printf(LOG_PREFIX "%s -> EAGAIN (%d times)\n",name,i);
                    rt_task_sleep(50000); // wait 50us
                    break;
      case 0:       printf(LOG_PREFIX "%s -> closed\n",name);
                    break;
      default:      printf(LOG_PREFIX "%s -> ???\n",name);
                    break;
    }
  } while (ret == -EAGAIN && i < 10);
  return ret;
}

void cleanup_all(void) {
  if (my_state & STATE_FILE_OPENED) {
    close_file( my_fd, RTSER_FILE " (rtser)");
    my_state &= ~STATE_FILE_OPENED;
  }
  if (my_state & STATE_TASK_CREATED) {
    printf(LOG_PREFIX "delete write_task\n");
    rt_task_delete(&write_task);
    my_state &= ~STATE_TASK_CREATED;
  }
  if (my_state & STATE_TASK_CREATED) {
    printf(LOG_PREFIX "delete read_task\n");
    rt_task_delete(&read_task);
    my_state &= ~STATE_TASK_CREATED;
  }
}

void catch_signal(int sig) {
  cleanup_all();
  printf(LOG_PREFIX "exit\n");
  return;
}

void write_task_proc(void *arg) {
  int ret;
  ssize_t sz = sizeof(RTIME);
  ssize_t written = 0;
  unsigned char buf[17] = "CAPTAIN WAS HERE\0";
  unsigned long overrun;

  ret = rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(write_task_period_ns));
  if (ret) {
    printf(WTASK_PREFIX "error while set periodic, code %d\n",ret);
    goto exit_write_task;
  }

  while (1) {
    ret = rt_task_wait_period(&overrun);
    if (ret) {
      printf(WTASK_PREFIX "error while rt_task_wait_period, code %d\n",ret);
      goto exit_write_task;
    }
    sz = sizeof(buf);
    written = rt_dev_write(my_fd, &buf, sizeof(buf));
    printf(WTASK_PREFIX "rt_dev_write written=%d sz=%d\n", written, sz);
    if (written != sz ) {
      if (written < 0 ) {
        printf(WTASK_PREFIX "error while rt_dev_write, code %d\n",written);
      } else {
        printf(WTASK_PREFIX "only %d / %d byte transmitted\n",written, sz);
      }
      goto exit_write_task;
    }
  }
exit_write_task:
  if (my_state & STATE_FILE_OPENED) {
    if (!close_file( my_fd, RTSER_FILE " (write)")) {
      my_state &= ~STATE_FILE_OPENED;
    }
  }
  printf(WTASK_PREFIX "exit\n");
}

void read_task_proc(void *arg) {
  int ret;
//  RTIME irq_time   = 0;
  ssize_t sz = sizeof(RTIME);
  ssize_t red = 0;
  struct rtser_event rx_event;
  unsigned char buf[17];

  while (1) {
    /* waiting for event */
    ret = rt_dev_ioctl(my_fd, RTSER_RTIOC_WAIT_EVENT, &rx_event );
    if (ret) {
      printf(RTASK_PREFIX "error while RTSER_RTIOC_WAIT_EVENT, code %d\n",ret);
      goto exit_read_task;
    }
    // if you need a receive timestamp, you can get it this way
    //irq_time = rx_event.rxpend_timestamp;
    sz = sizeof(buf);
    red = rt_dev_read(my_fd, &buf, sizeof(buf));
    if (red == sz ) {
		printf(RTASK_PREFIX "rt_dev_read=%s\n",buf);
    } else {
      if (red < 0 ) {
        printf(RTASK_PREFIX "error while rt_dev_read, code %d\n",red);
      } else {
        printf(RTASK_PREFIX "only %d / %d byte received \n",red,sz);
      }
      goto exit_read_task;
    }
  }
exit_read_task:
  if (my_state & STATE_FILE_OPENED) {
    if (!close_file( my_fd, READ_FILE " (rtser)")) {
      my_state &= ~STATE_FILE_OPENED;
    }
  }
  printf(RTASK_PREFIX "exit\n");
}

int main(int argc, char* argv[]) {
  int ret = 0;
  signal(SIGTERM, catch_signal);
  signal(SIGINT, catch_signal);

  printf(LOG_PREFIX "PRESS CTRL-C to EXIT\n");
  /* no memory-swapping for this programm */
  mlockall(MCL_CURRENT | MCL_FUTURE);

  /* open rtser0 */
  my_fd = rt_dev_open( RTSER_FILE, 0);
  if (my_fd < 0) {
    printf(LOG_PREFIX "can't open %s\n", RTSER_FILE);
    goto error;
  }
  my_state |= STATE_FILE_OPENED;
  printf(LOG_PREFIX "rtser_file opened\n");

  /* writing write-config */
  ret = rt_dev_ioctl(my_fd, RTSER_RTIOC_SET_CONFIG, &ser_config);
  if (ret) {
    printf(LOG_PREFIX "error while RTSER_RTIOC_SET_CONFIG, code %d\n",ret);
    goto error;
  }
  printf(LOG_PREFIX "serial-port-config written\n");

  /* create read_task */
  ret = rt_task_create(&read_task,"read_task",0,51,0);
  if (ret) {
    printf(LOG_PREFIX "failed to create read_task, code %d\n",ret);
    goto error;
  }
  my_state |= STATE_TASK_CREATED;
  printf(LOG_PREFIX "read-task created\n");

  /* create write_task */
  ret = rt_task_create(&write_task,"write_task",0,50,0);
  if (ret) {
    printf(LOG_PREFIX "failed to create write_task, code %d\n",ret);
    goto error;
  }
  my_state |= STATE_TASK_CREATED;
  printf(LOG_PREFIX "write-task created\n");

  /* start write_task */
  printf(LOG_PREFIX "starting write-task\n");
  ret = rt_task_start(&write_task,&write_task_proc,NULL);
  if (ret) {
    printf(LOG_PREFIX "failed to start write_task, code %d\n",ret);
    goto error;
  }

  /* start read_task */
  printf(LOG_PREFIX "starting read-task\n");
  ret = rt_task_start(&read_task,&read_task_proc,NULL);
  if (ret) {
    printf(LOG_PREFIX "failed to start read_task, code %d\n",ret);
    goto error;
  }

  pause();
  return 0;

error:
  cleanup_all();
  return ret;
}
