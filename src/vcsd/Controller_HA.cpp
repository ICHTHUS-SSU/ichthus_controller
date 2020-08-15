#include <stdio.h>
#include <string>
#include <map>
#include <iostream>
#include "hybridautomata.h"
#include <math.h>
#include <syslog.h> //added by hyo
using namespace std;

HybridAutomata *tuneCC_HA;
HybridAutomata *selfdrive_HA;
HybridAutomata *avc_HA;


extern int homingThrottle();
extern int homingBrake();
extern int get_cfParam(char *name);
extern int get_rtParam_int(char *name, char *field);
extern double get_rtParam_double(char *name, char *field);
extern int brakeModule(char *arg1, double arg2);
extern int throttleModule(char *arg1, double arg2);
extern float get_slave_virtual_target_pos(unsigned int slave_no);
extern float get_slave_virtual_current_pos(unsigned int slave_no);
extern int steerModule(char *arg1, double arg2);
extern void write_log(char *buf);
#define FREQUENCY 1

////////////////////////////////////////////////////////////////////////
///////////////////////////CruiseControl Controlller/////////////////////
////////////////////////////////////////////////////////////////////////

enum
{
    CC_START,
    CC_STANDBY,
    CC_DECIDE,
    CC_ACCEL,
    CC_DECEL,
    CC_FINISH
};

enum
{
    ACCELERATOR,
    DECELERATOR
};

char black[] = {0x1b, '[', '0', ';', '3', '0', 'm', 0};
char dark_gray[] = {0x1b, '[', '1', ';', '3', '0', 'm', 0};
char red[] = {0x1b, '[', '0', ';', '3', '1', 'm', 0};
char light_red[] = {0x1b, '[', '1', ';', '3', '1', 'm', 0};
char green[] = {0x1b, '[', '0', ';', '3', '2', 'm', 0};
char light_green[] = {0x1b, '[', '1', ';', '3', '2', 'm', 0};
char brown[] = {0x1b, '[', '0', ';', '3', '3', 'm', 0};
char yellow[] = {0x1b, '[', '1', ';', '3', '3', 'm', 0};
char blue[] = {0x1b, '[', '0', ';', '3', '4', 'm', 0};
char light_blue[] = {0x1b, '[', '1', ';', '3', '4', 'm', 0};
char purple[] = {0x1b, '[', '0', ';', '3', '5', 'm', 0};
char light_purple[] = {0x1b, '[', '1', ';', '3', '5', 'm', 0};
char cyan[] = {0x1b, '[', '0', ';', '3', '6', 'm', 0};
char light_cyan[] = {0x1b, '[', '1', ';', '3', '6', 'm', 0};
char light_gray[] = {0x1b, '[', '0', ';', '3', '7', 'm', 0};
char white[] = {0x1b, '[', '1', ';', '3', '7', 'm', 0};

double I_err;
double D_err;
double cur_err;
double prev_err;
double Kp_term;
double Ki_term;
double Kd_term;
double power;
int pedal_state = ACCELERATOR;
//int lookahead_tvel = get_rtParam_int("CruiseControl.target_velocity", "value");
double lookahead_tvel = 0;
//int new_tvel = lookahead_tvel;
double new_tvel = 0;
int timestamp = 0;
//int cc_log_fd;
unsigned int cc_print_freq = 0;
unsigned int sc_print_freq = 0;

void reset_pid_values_for_cc()
{
    I_err = 0;
    D_err = 0;
    cur_err = 0;
    prev_err = 0;
    Kp_term = 0;
    Ki_term = 0;
    Kd_term = 0;
    power = 0;
    pedal_state = ACCELERATOR;
    //lookahead_tvel = get_rtParam_int("CruiseControl.target_velocity", "value");
    lookahead_tvel = 0;
    new_tvel = lookahead_tvel;
}
void printStatusBar(double tvel, double cvel, int plus, int minus, int pedal_state)
{
    int switch_color = 1;
    int i;
    for (i = 0; i < cvel; i++)
    {
        if (i % 10 == 0)
        {
            if (pedal_state == ACCELERATOR)
            {
                if (switch_color == 1)
                {
                    cout << light_green;
                    switch_color = 0;
                }
                else if (switch_color == 0)
                {
                    cout << green;
                    switch_color = 1;
                }
            }
            else if (pedal_state == DECELERATOR)
            {
                if (switch_color == 1)
                {
                    cout << light_red;
                    switch_color = 0;
                }
                else if (switch_color == 0)
                {
                    cout << red;
                    switch_color = 1;
                }
            }
        }
        cout << "0";
    }
    cout << endl;

    for (i = 0; i < (int)(tvel + plus); i++)
    {
        if (i == (int)(tvel - minus))
        {
            cout << brown << "^";
            continue;
        }
        if (i == (int)(tvel))
        {
            cout << yellow << "^";
            continue;
        }
        cout << " ";
    }
    cout << brown << "^\n";

    for (i = 0; i < (int)(tvel + plus - 2); i++)
    {
        if (i == (int)(tvel - minus - 1))
        {
            cout << brown << (int)tvel - minus;
            continue;
        }
        if (i == (int)(tvel - 2))
        {
            cout << yellow << tvel;
            continue;
        }
        cout << " ";
    }
    cout << brown << (int)(tvel + plus) << endl;
    cout << white;
}
int CCdecide()
{
    static int kp, ki, kd;
    double actual_vel;
    static double prev_actual_vel = 0;
    static int changed_tvel = 0;
    static int keep_state = 0;
    static int plus_margin;
    static int minus_margin; // = get_param("behavior_cc.vel_margin"); // it was 3
    static int is_tvel_low = 1;
    static int check_count = 0;       //local
    static int changed_state = 0;     //local
    static double check_vel[4] = {0}; // local
    static int check_vel_count = 0;   //local
    static double very_past_err = 0;
    static double past_err = 0;
    //char brake_pos_buf[128] = {0};
    //char lookahead_tvel_buf[128] = {0};
    char buf[200] = {0};

    if (get_cfParam("Main.use_can") == 1)
    {
        actual_vel = get_rtParam_double("CAN.actual_velocity", "value");
    }
    else if (get_cfParam("Main.use_obd") == 1)
    {
        actual_vel = get_rtParam_double("OBD.actual_velocity", "value");
    }

    if (actual_vel > 255)
    {
        actual_vel = prev_actual_vel;
    }
    plus_margin = (int)(actual_vel * get_cfParam("CruiseControl.accel_velocity_tolerance") / 1000.0);
    minus_margin = (int)(actual_vel * get_cfParam("CruiseControl.decel_velocity_tolerance") / 1000.0);
    if (plus_margin < 4)
    {
        plus_margin = 4;
    }
    if (minus_margin < 4)
    {
        minus_margin = 4;
    }
    prev_actual_vel = actual_vel;

    if (new_tvel != get_rtParam_double("CruiseControl.target_velocity", "value")) // user changed tvel
    {
        new_tvel = get_rtParam_double("CruiseControl.target_velocity", "value");
        changed_tvel = 1;
        keep_state = 1;
        //delta = get_param("behavior_cc.vel_margin");
        lookahead_tvel = actual_vel;
        if (get_rtParam_double("CruiseControl.target_velocity", "value") > actual_vel)
        {
            is_tvel_low = 0;
        }
        else
        {
            is_tvel_low = 1;
        }
        changed_state = 0;
        check_vel_count = 0;
    }
    if (changed_tvel)
    {
        if (is_tvel_low)
        {
            pedal_state = DECELERATOR;
           /* if (actual_vel > new_tvel + 10) // it changed very higher
            {
                lookahead_tvel = actual_vel - 7; //should change lookahead_tvel smaller
            }
            else if (actual_vel > new_tvel + 3) // it changed very higher
            {
                lookahead_tvel = actual_vel - 3; //should change lookahead_tvel smaller
            }
            else // now its stable
            {*/
                lookahead_tvel = new_tvel;
                changed_tvel = 0;
           //}
        }
        else //tvel 이 actual_vel 보다 더 큰 경우
        {
            pedal_state = ACCELERATOR;
            if (actual_vel < new_tvel - 10) //its getting very slower
            {
                lookahead_tvel = actual_vel + 7; // should change lookahead_tvel little bit bigger
            }
            else if (actual_vel < new_tvel - 3) //its getting very slower
            {
                lookahead_tvel = actual_vel + 3; // should change lookahead_tvel little bit bigger
            }
            else
            {
                lookahead_tvel = new_tvel;
                changed_tvel = 0;
            }
            //}
        }
    }
    else // if user do not change tvel
    {
        lookahead_tvel = new_tvel;
        changed_tvel = 0;
    }
    very_past_err = past_err; 
    past_err = new_tvel - actual_vel;
    cur_err = lookahead_tvel - actual_vel;

    switch (pedal_state)
    {
    case ACCELERATOR:
        //printf("ACCEL\n");
        if (get_cfParam("Main.use_can") == 1)
        {
            kp = get_cfParam("CruiseControl.CAN.accel_gain_p");
            ki = get_cfParam("CruiseControl.CAN.accel_gain_i");
            kd = get_cfParam("CruiseControl.CAN.accel_gain_d");
        }
        else if (get_cfParam("Main.use_obd") == 1)
        {
            kp = get_cfParam("CruiseControl.OBD.accel_gain_p");
            ki = get_cfParam("CruiseControl.OBD.accel_gain_i");
            kd = get_cfParam("CruiseControl.OBD.accel_gain_d");
        }

        if (changed_state) // it was DECELARATOR right before(noticed that it is uphill)
        {
            if (lookahead_tvel > actual_vel) //is slow
            {
                check_vel[check_vel_count] = actual_vel;
                ++check_vel_count;
                if (check_vel_count > 3) //여러번 페달 제어를 시도했음에도 수렴을 하지 않았으므로 steady state error 가 발생한 것으로 판단.
                {
                    if (check_vel[0] >= check_vel[3])
                    {
                        if (get_cfParam("Main.use_obd") == 1)
                        {
                            I_err = ((double)get_cfParam("CruiseControl.accumulated_error_base") + (double)get_cfParam("CruiseControl.accumulated_error_delta") * (check_vel[0] - check_vel[3])) / (double)get_cfParam("CruiseControl.OBD.accel_gain_i");
                        } 
                    }
                    changed_state = 0;
                    check_vel_count = 0;
                }
            }
            else
            {
                changed_state = 0;
                check_vel_count = 0;
            }
        }
        //if ((get_slave_virtual_target_pos(get_cfParam("ECAT.motor_id_throttle")) <= get_cfParam("HomingPedals.tolerance") / get_cfParam("Throttle.convert_position"))
        if (actual_vel > lookahead_tvel + plus_margin)
        {
            if (keep_state)
            {
                if (very_past_err > past_err) // but is getting slower
                {
                    ++check_count;
                }
                else if (past_err > very_past_err) // its getting more faster!
                {
                    if (!check_count) 
                    {
                        --check_count;
                    }
                }
                if (check_count > 3) // idle time over, should change state
                {
                    check_count = 0;
                    keep_state = 0;
                }

            }
            else // should change state
            {
                pedal_state = DECELERATOR;
                I_err = 0;
                prev_err = 0;
                if (get_cfParam("Main.use_can") == 1)
                {
                    kp = get_cfParam("CruiseControl.CAN.decel_gain_p");
                    ki = get_cfParam("CruiseControl.CAN.decel_gain_i");
                    kd = get_cfParam("CruiseControl.CAN.decel_gain_d");
                }
                else if (get_cfParam("Main.use_obd") == 1)
                {
                    kp = get_cfParam("CruiseControl.OBD.decel_gain_p");
                    ki = get_cfParam("CruiseControl.OBD.decel_gain_i");
                    kd = get_cfParam("CruiseControl.OBD.decel_gain_d");
                }
                check_count = 0;
                keep_state = 0;
            }
        }

        else // is not fast yet
        {
            keep_state = 0;
            check_count = 0;
        }
        break;
    case DECELERATOR:
        //printf("DECEL\n");
        if (get_cfParam("Main.use_can") == 1)
        {
            kp = get_cfParam("CruiseControl.CAN.decel_gain_p");
            ki = get_cfParam("CruiseControl.CAN.decel_gain_i");
            kd = get_cfParam("CruiseControl.CAN.decel_gain_d");
        }
        else if (get_cfParam("Main.use_obd") == 1)
        {
            kp = get_cfParam("CruiseControl.OBD.decel_gain_p");
            ki = get_cfParam("CruiseControl.OBD.decel_gain_i");
            kd = get_cfParam("CruiseControl.OBD.decel_gain_d");
        }

        //if ((get_slave_virtual_target_pos(get_cfParam("ECAT.motor_id_brake")) <= (double)get_cfParam("HomingPedals.tolerance") / (double)get_cfParam("Brake.convert_position"))
        if (actual_vel < lookahead_tvel - minus_margin)
        {

            if (keep_state)
            {
                //printf("!!!!!!!!!!KEEP STATE_!!!!!!!!!!!!\n\n");
                if (very_past_err < past_err) // is very slow but getting faster
                {
                    ++check_count;
                }
                else if (past_err < very_past_err) // is getting very slower
                {
                    if (!check_count)
                    {
                        --check_count;
                    }
                }
                if (check_count > 3) // its very slow should switch state
                {
                    check_count = 0;
                    keep_state = 0;
                }
            }
            else
            {
                changed_state = 1;
                pedal_state = ACCELERATOR;
                I_err = 0;
                prev_err = 0;
                if (get_cfParam("Main.use_can") == 1)
                {
                    kp = get_cfParam("CruiseControl.CAN.accel_gain_p");
                    ki = get_cfParam("CruiseControl.CAN.accel_gain_i");
                    kd = get_cfParam("CruiseControl.CAN.accel_gain_d");
                }
                else if (get_cfParam("Main.use_obd") == 1)
                {
                    kp = get_cfParam("CruiseControl.OBD.accel_gain_p");
                    ki = get_cfParam("CruiseControl.OBD.accel_gain_i");
                    kd = get_cfParam("CruiseControl.OBD.accel_gain_d");
                }
                check_count = 0;
                keep_state = 0;
            }
        }

        else // is fast
        {
            keep_state = 0;
            check_count = 0;
        }
        break;
    }

    Kp_term = (double)kp * cur_err;
    I_err += cur_err;
    Ki_term = (double)ki * I_err;
    D_err = (prev_err - cur_err);
    Kd_term = (double)kd * D_err;
    power = Kp_term + Ki_term + Kd_term;
    prev_err = cur_err;

    double tpos = power / (double)get_cfParam("Throttle.convert_position");
    double bpos = (-power) / (double)get_cfParam("Brake.convert_position");
    char log_buf[128];
    if (cc_print_freq == 0)
    {

        syslog(LOG_ERR,"######## Cruise Control ########\n");
        syslog(LOG_ERR,"##     plus_margin = %d       \n", plus_margin);
        syslog(LOG_ERR,"##     minus_margin = %d      \n", minus_margin);
        syslog(LOG_ERR,"##     target Vel = %lf        \n", new_tvel);
        syslog(LOG_ERR,"##      current Vel = %lf      \n", actual_vel);
        //syslog(LOG_ERR,"##    lookahead_tvel = %lf      \n", lookahead_tvel);
        syslog(LOG_ERR,"##       power = %lf      \n", power);


        if (pedal_state == ACCELERATOR)
        {
            syslog(LOG_ERR,"##        ACCELERATION        \n");
            //syslog(LOG_ERR,"##     Throttle_pos = %lf      \n", tpos*get_cfParam("Throttle.convert_position"));
        }
        else
        {
            syslog(LOG_ERR,"##        DECELERATION        \n");
            //syslog(LOG_ERR,"##      Brake_pos = %lf      \n", bpos* (double)get_cfParam("Brake.convert_position"));
        }
        syslog(LOG_ERR,"################################\n");


        /*cout << white << "\n######## Cruise Control ########\n";
        cout <<          "##    cur_vel = "<< actual_vel<<"  power = "<<power ;
        if (pedal_state == ACCELERATOR)
        {
            cout << " state = " << light_green << " ACCEL\n";
        }
        else
        {
            cout << " state = " << light_red << " DECEL\n";
        }
        cout << white << "################################\n\n";
        */
        printStatusBar(new_tvel, actual_vel, plus_margin, minus_margin, pedal_state);
        cc_print_freq = FREQUENCY;
        sprintf(log_buf, "tvel : %lf, cvel : %lf, plus_margin : %d, minus_margin : %d, state : %d\n", new_tvel, actual_vel, plus_margin, minus_margin, pedal_state);
        write_log(log_buf);
    }
    else
        cc_print_freq--;
    return 1;
}

int CCaccel()
{
    double pos = power / (double)get_cfParam("Throttle.convert_position");

    homingBrake();
    /*
    if (pos > (double)get_cfParam("Throttle.virtual_max_position"))
    {
        throttleModule("Throttle.target_position", get_cfParam("Throttle.virtual_max_position"));
    }
    else
    */
    if (pos < (double)get_cfParam("Throttle.virtual_min_position"))
    {
        homingThrottle();
    }
    else
    {

        throttleModule("Throttle.target_position", pos);
    }
    return 1;
}

int CCdecel()
{
    double pos = (-power) / (double)get_cfParam("Brake.convert_position");
    double start_pos = (double)get_cfParam("Brake.start_position") / (double)get_cfParam("Brake.convert_position");
    homingThrottle();

    if ((start_pos+pos) >= (double)get_cfParam("Brake.virtual_max_position"))
    {
        brakeModule("Brake.target_position", get_cfParam("Brake.virtual_max_position"));
    }
    else if ((start_pos + pos) < (double)get_cfParam("Brake.virtual_min_position"))
    {
        homingBrake();
    }
    else
    {

        brakeModule("Brake.target_position", start_pos + pos);
        //brakeModule("Brake.target_position", pos);
    }
    return 1;
}

int CCfinish()
{
    return 1;
}

int CCstandby()
{
    return 1;
}

class CC_S2D : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_cfParam("Main.use_obd") == 1)
        {
            if (get_rtParam_int("OBD.actual_velocity", "timestamp") != timestamp)
            {
                //cout << "condition : CC_D2D" << endl;
                timestamp = get_rtParam_int("OBD.actual_velocity", "timestamp");
                return true;
            }
        }
        else if (get_cfParam("Main.use_can") == 1)
        {
            if (get_rtParam_int("CAN.actual_velocity", "timestamp") != timestamp)
            {
                //cout << "condition : CC_D2D" << endl;
                timestamp = get_rtParam_int("CAN.actual_velocity", "timestamp");
                return true;
            }
        }

        return false;
    }
};
class CC_S2F : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_cfParam("Main.use_obd") == 1)
        {
            if (get_rtParam_int("OBD.actual_velocity", "timestamp") == timestamp)
            {
                //cout << "condition : CC_D2D" << endl;
                return true;
            }
        }
        else if (get_cfParam("Main.use_can") == 1)
        {
            if (get_rtParam_int("CAN.actual_velocity", "timestamp") == timestamp)
            {
                //cout << "condition : CC_D2D" << endl;
                return true;
            }
        }

        return false;
    }
};
class CC_D2A : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (pedal_state == ACCELERATOR)
        {
            // cout << "condition : CC_D2A" << endl;
            return true;
        }

        return false;
    }
};

class CC_D2D : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (pedal_state == DECELERATOR)
        {
            //cout << "condition : CC_D2D" << endl;
            return true;
        }
        return false;
    }
};
void initHA_tuneCC()
{
    tuneCC_HA = new HybridAutomata(CC_START, CC_FINISH);

    tuneCC_HA->setState(CC_STANDBY, CCstandby);
    tuneCC_HA->setState(CC_DECIDE, CCdecide);
    tuneCC_HA->setState(CC_ACCEL, CCaccel);
    tuneCC_HA->setState(CC_DECEL, CCdecel);
    tuneCC_HA->setState(CC_FINISH, CCfinish);

    CC_D2A *cc_d2a = new CC_D2A();
    CC_D2D *cc_d2d = new CC_D2D();
    CC_S2D *cc_s2d = new CC_S2D();
    CC_S2F *cc_s2f = new CC_S2F();

    tuneCC_HA->setCondition(CC_START, NULL, CC_STANDBY);
    tuneCC_HA->setCondition(CC_STANDBY, cc_s2d, CC_DECIDE);
    tuneCC_HA->setCondition(CC_STANDBY, cc_s2f, CC_FINISH);
    tuneCC_HA->setCondition(CC_DECIDE, cc_d2a, CC_ACCEL);
    tuneCC_HA->setCondition(CC_DECIDE, cc_d2d, CC_DECEL);
    tuneCC_HA->setCondition(CC_ACCEL, NULL, CC_FINISH);
    tuneCC_HA->setCondition(CC_DECEL, NULL, CC_FINISH);
}
int operateHA_tuneCC()
{
    if (tuneCC_HA->curState == CC_FINISH)
    {
        tuneCC_HA->curState = CC_START;
    }
    return tuneCC_HA->operate();
}

////////////////////////////////////////////////////////////////////////
///////////////////////////end of CruiseControl Controlller/////////////
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
///////////////////////////SteerControl Controlller/////////////////////
////////////////////////////////////////////////////////////////////////

double convert_kmh_to_mps(double tvel)
{
    return tvel / 3.6;
}
double get_radius()
{
    double omega;
    double v;
    double radius;
    double wheel_base = (double)get_cfParam("SteerControl.wheel_base") / 100.0;
    double min_radius = (double)get_cfParam("SteerControl.min_radius") / 100.0;

    omega = get_rtParam_double("SteerControl.target_angular_velocity", "value");
    v = convert_kmh_to_mps(get_rtParam_double("CruiseControl.target_velocity", "value"));
    radius = v / omega;

    //syslog(LOG_ERR,"##      radius : %lf        \n", radius);
    if (sc_print_freq == 0)
    {
        // printf("\t######## Steer Control ########\n");
        // printf("\t##     wheel_base = %f       \n", wheel_base);
        // printf("\t##     omega = %f            \n", omega);
    }

    if ((omega == 0) || (v == 0))
        return 0;

    if ((radius < min_radius) && (radius > 0))
        radius = min_radius;
    else if ((radius > -min_radius) && (radius < 0))
        radius = -min_radius;
    return atan(wheel_base / radius);
}
int steerControl()
{
    if (get_cfParam("ECAT.motor_id_steer") < get_rtParam_int("ECAT.num_motors", "value"))
    {
        double angle = get_radius();
        double wheel_motor_pos = angle * (double)get_cfParam("SteerControl.position_per_degree");

        //syslog(LOG_ERR,"##        Wheel_motor_pos : %lf        \n", wheel_motor_pos);
        //syslog(LOG_ERR, "Wheel Motor Pos : %lf\n", wheel_motor_pos);
        steerModule("Steerwheel.target_position", wheel_motor_pos);
        if (sc_print_freq == 0)
        {
            // printf("\t##   Wheel_motor_POS = %f    \n", wheel_motor_pos);
            // printf("\t################################\n");
            sc_print_freq = FREQUENCY;
        }
        else
            sc_print_freq--;
        return 1;
    }
    return -1;
}

////////////////////////////////////////////////////////////////////////
///////////////////////////end of SteerControl Controlller//////////////
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
///////////////////////////SelfDriving Controlller/////////////////////
////////////////////////////////////////////////////////////////////////
enum
{
    SD_START,
    SD_SC,
    SD_CC,
    SD_FINISH
};
int SDfinish()
{

    return;
}

void initHA_selfdriving()
{
    selfdrive_HA = new HybridAutomata(SD_START, SD_FINISH);

    selfdrive_HA->setState(SD_SC, steerControl);
    selfdrive_HA->setState(SD_CC, operateHA_tuneCC);
    selfdrive_HA->setState(SD_FINISH, SDfinish);


    selfdrive_HA->setCondition(SD_START, NULL, SD_SC);
    selfdrive_HA->setCondition(SD_SC, NULL, SD_CC);
    selfdrive_HA->setCondition(SD_CC, NULL, SD_FINISH);
}
int operateHA_selfdriving()
{
    if (selfdrive_HA->curState == SD_FINISH)
    {
        selfdrive_HA->curState = SD_START;
    }
    return selfdrive_HA->operate();
}
////////////////////////////////////////////////////////////////////////
///////////////////////////end of SelfDriving Controlller//////////////
////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////
///////////////////////////AVC Controlller/////////////////////
////////////////////////////////////////////////////////////////////////

extern int estop();



enum
{
    AVC_START,
    AVC_SELFDRIVING,
    AVC_ESTOP,
    AVC_PULLOVER,
    AVC_FINISH
};
int AVCfinish()
{
    return;
}
class AVC_S2E : public Condition
{

  public:
    bool check(HybridAutomata *HA)
    {
        if(get_rtParam_double("CruiseControl.target_velocity", "value") == -1.0)
	// || rpi == p
        {
            reset_pid_values_for_cc();
            cout << "condition : AVC_S2E" << endl;
            return true;
        }
        return false;
    }
};
class AVC_S2P : public Condition
{

  public:
    bool check(HybridAutomata *HA)
    {
        if(get_rtParam_double("CruiseControl.target_velocity", "value") == 0.0)
	// || rpi == p
        {
            reset_pid_values_for_cc();
            cout << "condition :AVC_S2P" << endl;
            return true;
        }
        return false;
    }
};
class AVC_S2S : public Condition
{

  public:
    bool check(HybridAutomata *HA)
    {
        if(get_rtParam_double("CruiseControl.target_velocity", "value") >= 0.0)
        // && rpi == s
        {
            //cout << "condition : AVC_S2S" << endl;
            return true;
        }
        return false;
    }
};
class AVC_E2S : public Condition
{

  public:
    bool check(HybridAutomata *HA)
    {
        if(get_rtParam_double("CruiseControl.target_velocity", "value") >= 0.0)
        // && rpi == s
        {
            cout << "condition : AVC_E2S" << endl;
            return true;
        }
        return false;
    }
};
class AVC_E2P : public Condition
{

  public:
    bool check(HybridAutomata *HA)
    {
        if(get_rtParam_double("CruiseControl.target_velocity", "value") == 0.0)
        // && rpi == s
        {
            cout << "condition : AVC_E2P" << endl;
            return true;
        }
        return false;
    }
};
class AVC_E2E : public Condition
{
    double actual_vel;

  public:
    bool check(HybridAutomata *HA)
    {
        if(get_rtParam_double("CruiseControl.target_velocity", "value") == -1.0)
        // || rpi == p
        {
            //cout << "condition : AVC_E2E" << endl;
            return true;
        }
        return false;
    }
};
class AVC_P2S : public Condition
{

  public:
    bool check(HybridAutomata *HA)
    {
        if(get_rtParam_double("CruiseControl.target_velocity", "value") > 0.0)
        // && rpi == s
        {
            cout << "condition : AVC_P2S" << endl;
            return true;
        }
        return false;
    }
};
class AVC_P2E : public Condition
{

  public:
    bool check(HybridAutomata *HA)
    {
        if(get_rtParam_double("CruiseControl.target_velocity", "value") == -1.0)
        // && rpi == s
        {
            cout << "condition : AVC_P2E" << endl;
            return true;
        }
        return false;
    }
};
class AVC_P2P : public Condition
{

  public:
    bool check(HybridAutomata *HA)
    {
        if(get_rtParam_double("CruiseControl.target_velocity", "value") == 0.0)
        // && rpi == s
        {
            //cout << "condition : AVC_P2P" << endl;
            return true;
        }
        return false;
    }
};
extern int operateHA_pullover();
void initHA_avc()
{
    avc_HA = new HybridAutomata(AVC_START,AVC_FINISH);


    avc_HA->setState(AVC_SELFDRIVING, operateHA_selfdriving);
    avc_HA->setState(AVC_ESTOP, estop);
    avc_HA->setState(AVC_PULLOVER, operateHA_pullover);
    avc_HA->setState(AVC_FINISH, AVCfinish);

    AVC_S2E *avc_s2e = new AVC_S2E();
    AVC_S2S *avc_s2s = new AVC_S2S();
    AVC_S2P *avc_s2p = new AVC_S2P();
    AVC_E2S *avc_e2s = new AVC_E2S();
    AVC_E2E *avc_e2e = new AVC_E2E();
    AVC_E2P *avc_e2p = new AVC_E2P();
    AVC_P2E *avc_p2e = new AVC_P2E();
    AVC_P2S *avc_p2s = new AVC_P2S();
    AVC_P2P *avc_p2p = new AVC_P2P();


    avc_HA->setCondition(AVC_START, NULL, AVC_ESTOP);
    avc_HA->setCondition(AVC_ESTOP, avc_e2e, AVC_ESTOP);
    avc_HA->setCondition(AVC_ESTOP, avc_e2s, AVC_SELFDRIVING);
    //avc_HA->setCondition(AVC_ESTOP, avc_e2p, AVC_PULLOVER);
    avc_HA->setCondition(AVC_SELFDRIVING, avc_s2e, AVC_ESTOP);
    //avc_HA->setCondition(AVC_SELFDRIVING, avc_s2p, AVC_PULLOVER);
    avc_HA->setCondition(AVC_SELFDRIVING, avc_s2s, AVC_SELFDRIVING);
    avc_HA->setCondition(AVC_PULLOVER, avc_p2e, AVC_ESTOP);
    avc_HA->setCondition(AVC_PULLOVER, avc_p2s, AVC_SELFDRIVING);
    avc_HA->setCondition(AVC_PULLOVER, avc_p2p, AVC_PULLOVER);

}
int operateHA_avc()
{
    if (avc_HA->curState == AVC_FINISH)
    {
        avc_HA->curState = AVC_START;
    }
    return avc_HA->operate();
}
////////////////////////////////////////////////////////////////////////
///////////////////////////end of AVC Controlller//////////////
////////////////////////////////////////////////////////////////////////
