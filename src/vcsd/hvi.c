#include <string.h>
#include <termios.h>
#include <stdio.h>
#include <pthread.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <wiringPi.h>

#define STOP 1
#define RUN  2

extern int set_rtParam(char* name, char* field, double val);
extern int get_rtParam_int(char *name, char *field);
extern int get_cfParam(char *name);

extern pthread_mutex_t glob_mutex;

pthread_t hvi_cyclic_thread;

static int in_button = NULL;
static int out_button_center = NULL;
static int out_button_left = NULL;
static int out_button_right = NULL;
static int estop_run = NULL;
static int estop_pause = NULL;
static int buzzer_front = NULL;
static int buzzer_rear = NULL;
static int light_left_red = NULL;
static int light_left_green = NULL;
static int light_right_red = NULL;
static int light_right_green = NULL;
void setSigntower(int value)
{
    if (value == STOP) {
        // turn off green light
        digitalWrite(light_left_green, HIGH);
        digitalWrite(light_right_green, HIGH);

        // turn on red light
        digitalWrite(light_left_red, LOW);
        digitalWrite(light_right_red, LOW);
    }

    else if (value == RUN) {
        // turn off red light
        digitalWrite(light_left_red, HIGH);
        digitalWrite(light_right_red, HIGH);

        // turn on green light
        digitalWrite(light_left_green, LOW);
        digitalWrite(light_right_green, LOW);
    }

    else 
        fprintf(stderr, "wrong input at setSigntower\n");
}

void setBuzzer(int value)
{
    if (value == STOP) {
        // turn off buzzer
        digitalWrite(buzzer_front, HIGH);
        digitalWrite(buzzer_rear, HIGH);
    }

    else if (value == RUN) {
        // turn on buzzer
        digitalWrite(buzzer_front, LOW);
        digitalWrite(buzzer_rear, LOW);
    }

    else
        fprintf(stderr, "wrong input at setBuzzer\n");
}
int hviUp()
{
    fprintf(stdout, "HVI setup started-----\n");
    
    in_button = get_cfParam("HVI.estop_btn_seat_gpio");
    out_button_center = get_cfParam("HVI.estop_btn_center_gpio");
    out_button_left = get_cfParam("HVI.estop_btn_left_gpio");
    out_button_right = get_cfParam("HVI.estop_btn_right_gpio");
    estop_run = get_cfParam("HVI.estop_remote_run_gpio");
    estop_pause = get_cfParam("HVI.estop_remote_pause_gpio");
    buzzer_front = get_cfParam("HVI.buzzer_front_gpio");
    buzzer_rear = get_cfParam("HVI.buzzer_rear_gpio");
    light_left_red = get_cfParam("HVI.signtower_left_red_gpio");
    light_left_green = get_cfParam("HVI.signtower_left_green_gpio");
    light_right_red = get_cfParam("HVI.signtower_right_red_gpio");
    light_right_green = get_cfParam("HVI.signtower_right_green_gpio");
    
    wiringPiSetupGpio();
    
    pinMode(in_button, INPUT);
    pinMode(out_button_center, INPUT);
    pinMode(out_button_left, INPUT);
    pinMode(out_button_right, INPUT);
    pinMode(estop_run, INPUT);
    pinMode(estop_pause, INPUT);
    pinMode(buzzer_front, OUTPUT);
    pinMode(buzzer_rear, OUTPUT);
    pinMode(light_left_red, OUTPUT);
    pinMode(light_left_green, OUTPUT);
    pinMode(light_right_red, OUTPUT);
    pinMode(light_right_green, OUTPUT);

    digitalWrite(light_left_green, HIGH);
    digitalWrite(light_right_green, HIGH);
    digitalWrite(light_left_red, HIGH);
    digitalWrite(light_right_red, HIGH);
    digitalWrite(buzzer_front, HIGH);
    digitalWrite(buzzer_rear, HIGH);


    setSigntower(1);
    fprintf(stdout, "HVI setup finished-----\n");
    return 1;
}

void* hviCyclic(void* name)
{
    int state = 0;
    int pre_state = 0;

    while(1) {
        if(digitalRead(in_button) == HIGH
        || digitalRead(out_button_center) == HIGH
        || digitalRead(out_button_left) == HIGH
        || digitalRead(out_button_right) == HIGH
        || digitalRead(estop_pause) == HIGH) {
            state = STOP;
        }

        if(digitalRead(in_button) == LOW
        && digitalRead(out_button_center) == LOW
        && digitalRead(out_button_left) == LOW
        && digitalRead(out_button_right) == LOW
        && digitalRead(estop_pause) == LOW
        && digitalRead(estop_run) == HIGH) {
            state = RUN;
        }

        if(pre_state != state) {
            if (state == STOP) {
                set_rtParam("HVI.mode", "value", STOP);
                set_rtParam("Motion.state","value",6);
                pre_state = STOP;
            } else if (state == RUN) {
                set_rtParam("HVI.mode", "value", RUN);
                set_rtParam("Motion.state","value",11);
                pre_state = RUN;
            } else {
                fprintf(stderr, "HVI.mode state is broken!!!\n");
                set_rtParam("HVI.mode", "value", STOP);
                set_rtParam("Motion.state","value",6);
            }

        }
    }
}

int hviOn()
{
    fprintf(stdout, "HVI_ON called\n");
    if (pthread_create(&hvi_cyclic_thread, 0 , hviCyclic, NULL)) {
        fprintf(stderr, "HVI thread err\n");
    }
    return 1;
}

int hviOff()
{
    fprintf(stdout, "HVI_OFF called\n");
    pthread_cancel(hvi_cyclic_thread);
    pthread_join(hvi_cyclic_thread, NULL);
    return 1;
}

int hviDown()
{
    fprintf(stdout, "HVI_DOWN called\n");
    digitalWrite(light_left_green, HIGH);
    digitalWrite(light_right_green, HIGH);
    digitalWrite(light_left_red, HIGH);
    digitalWrite(light_right_red, HIGH);
    digitalWrite(buzzer_front, HIGH);
    digitalWrite(buzzer_rear, HIGH);
    return 1;
}


