

#include <string.h>
#include <termios.h>
#include <stdio.h>
#include <pthread.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define OBD_USB_PORT "/dev/ttyACM0"
#define WAIT_OBD(str_size) usleep((str_size + 25) * 100)

extern int set_rtParam(char *name, char *field, double val);
extern void write_log(char *buf);
extern void sendOdom(double param_val, int bparam);
extern double convert_kmh_to_mps(double tvel);
extern int get_rtParam_int(char *name, char *field);
typedef termios Termios;

typedef struct _obd_
{
  Termios options;
  int rdlen;
  int fd;
  int vel;
} OBD;

extern int nobd2_var;
extern pthread_mutex_t glob_mutex;
unsigned int obd_seqno = 0;
// extern Var obd2_var[];
char AT_RESET[5] = "AT Z\r";
char AT_SP[8] = "AT SP 0\r";
char VEHICLE_SPEED[6] = "01 0D\r";
//char *AT_RESET = "Z TA\r";
//char *AT_SP = "\r0 PS TA";
//char * VEHICLE_SPEED = "\rD0 10";
//char * CARRIAGE_RETURN = "\r";

OBD obd;
pthread_t obd_cyclic_thread;

/*Init OBD II*/
int obdUp()
{
  char buf[128] = {}; //buffer for OBD data

  obd.fd = open(OBD_USB_PORT, O_RDWR | O_NOCTTY | O_SYNC);
  if (obd.fd < 0)
  {
    printf("OBD Open Error\n");
  }

  bzero(&obd.options, sizeof(obd.options));

  obd.options.c_cflag |= B38400 | CS8 | CLOCAL | CREAD | IGNPAR;
  obd.options.c_cflag &= ~CSIZE & ~PARENB & ~CSTOPB & ~CRTSCTS;

  tcflush(obd.fd, TCIFLUSH);
  tcsetattr(obd.fd, TCSANOW, &obd.options);

  if (write(obd.fd, AT_RESET, sizeof(AT_RESET)) < sizeof(AT_RESET))
  {
    printf("OBD AT_RESET Write Error\n");
    return -1;
  }
  WAIT_OBD(sizeof(AT_RESET));

  obd.rdlen = read(obd.fd, buf, sizeof(buf));
  if (obd.rdlen > 0)
  {
    buf[obd.rdlen] = '\0';
    printf("%s\n", buf);
    if (write(obd.fd, VEHICLE_SPEED, sizeof(VEHICLE_SPEED)) < sizeof(VEHICLE_SPEED))
    {
      printf("OBD VEHICLE_SPEED Write Error\n");
      return -1;
    }
    WAIT_OBD(sizeof(VEHICLE_SPEED));
  }
  else
  {
    printf("OBD Read Error\n");
    return -1;
  }
  return 1;
}

void *obdCyclic(void *name)
{
  /* added by han 20191010
  int rdlen, i, j;
  char buf[128] = {0};
  char change_buf[128] = {0};
  int response;
  static int timestamp = 0;
  char VEHICLE_SPEED[] = "01 0D\r";
  char CARRIAGE_RETURN[] = "\r";
 
  while (1)
  {
    if (write(obd.fd, CARRIAGE_RETURN, sizeof(CARRIAGE_RETURN)) < sizeof(CARRIAGE_RETURN))
    {
      printf("OBD CARRIAGE_RETURN Write Error\n");
    }
    usleep(sizeof(CARRIAGE_RETURN) * 100);
    rdlen = read(obd.fd, buf, sizeof(buf));
    if (rdlen > 0)
    {
      for (i = 6, j = 0; i < rdlen; ++i, ++j)
      {
        if (buf[i] != ' ')
        {
          change_buf[j] = buf[i];
        }
        else
        {
          --j;
        }
      }
      change_buf[j] = '\0';
      sscanf(change_buf, "%x", &obd.vel);
      if (obd.vel < 200)
      {
        set_rtParam("OBD.actual_velocity", "value", obd.vel);
        // if(get_rtParam_int("OBD.publish_to_agent","value") == 1 )
        // {
        //   sendOdom(++seqno,convert_kmh_to_mps(obd.vel),0);
        // }
        if(get_rtParam_int("OBD.publish_to_agent","value") == 1 )
        {
          sendOdom(convert_kmh_to_mps(obd.vel),2);
        }

      }
    }
    else
    {
      //sprintf(buf, "OBD Vel read Error\n");
      //write_log(buf);
    }
  }*/
}

int obdOn()
{

  printf("OBD_ON called\n");
  if (pthread_create(&obd_cyclic_thread, 0, obdCyclic, NULL))
  {
    printf("OBD thread err\n");
  }
  return 1;
}

int obdOff()
{
  printf("OBD_OFF called\n");
  pthread_cancel(obd_cyclic_thread);
  pthread_join(obd_cyclic_thread, NULL);
  return 1;
}

int obdDown()
{
  printf("OBD_DOWN called\n");
  close(obd.fd);
  return 1;
}
