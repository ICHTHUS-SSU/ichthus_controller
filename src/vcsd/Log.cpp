#include <iostream>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

using namespace std;
extern int get_cfParam(char *name);

int log_fd;

void open_log(char *fname)
{
    log_fd = open(fname, O_CREAT | O_RDWR | O_APPEND, 0666);
    if (log_fd < 0)
    {
        printf("Log Open err\n");
    }
}

void write_log(char *buf)
{
    if(get_cfParam("log") == 0) return;
    struct timeval val;
    struct tm *ptm;
    char log[200];

    gettimeofday(&val, NULL);
    ptm = localtime(&val.tv_sec);

    memset(log, 0x00, sizeof(log));

    // format : YYMMDDhhmmssuuuuuu
    sprintf(log, "%02d:/%02d:/%02d:/%06ld, %s ", 
    ptm->tm_hour, ptm->tm_min, ptm->tm_sec, val.tv_usec, buf);

    write(log_fd, log, strlen(log));
}
