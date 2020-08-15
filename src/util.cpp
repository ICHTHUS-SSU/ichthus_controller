#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/timerfd.h>

namespace ichthus_controller {

void quit(const char *tag){
  printf("%s error: %s (errno=%d)\n", tag, strerror(errno), errno);
  exit(1);
}

int init_timer(int usec) {
  int ret;
  int fd = -1;
  struct itimerspec timeout;

  if (usec >= 1000000) {
    printf("init_timer() accepts 1~999999 us.\n");
    exit(1);
  }

  if ((fd = timerfd_create(CLOCK_REALTIME, 0)) <= 0)
    quit("timerfd_create");

  //if ((ret = fcntl(fd, F_SETFL, O_NONBLOCK)) != 0)
  //  quit("fcntl");

  timeout.it_value.tv_sec = 0;
  timeout.it_value.tv_nsec = usec * 1000;
  timeout.it_interval.tv_sec = 0; // recurring
  timeout.it_interval.tv_nsec = usec * 1000;
  if ((ret = timerfd_settime(fd, 0, &timeout, NULL)) != 0)
    quit("timerfd_settime");

  return fd;
}

int wait_timer(int timerfd) {
  int ret;
  unsigned long long missed;
  if ((ret = read(timerfd, &missed, sizeof(missed))) < 0)
    printf("read error\n");

  //printf("missed=%lu\n",missed);

  return ret;
}

}
