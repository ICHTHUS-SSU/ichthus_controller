#include <stdio.h>
#include <string>
#include <map>
#include <iostream>
#include <syslog.h>
#include "hybridautomata.h"

HybridAutomata *estop_HA;
HybridAutomata *pullover_HA;
HybridAutomata *homingpedals_HA;
HybridAutomata *selftest_HA;
HybridAutomata *ready2start_HA;

extern int get_cfParam(char *name);
extern float get_slave_virtual_current_pos(unsigned int slave_no);
extern int brakeModule(char *arg1, double arg2);
extern int throttleModule(char *arg1, double arg2);
extern int lidarModule(char *arg1, double arg2);
extern int steerModule(char *arg1, double arg2);
extern int gearstickModule(char *arg1, double arg2);
extern int get_slave_physical_act_pos(unsigned int slave_no);
extern int get_slave_physical_target_pos(unsigned int slave_no);
extern double get_rtParam_double(char *name, char *field);
extern int set_rtParam(char *name, char *field, double val);
extern int get_rtParam_int(char *name, char *field);
extern void disableMotor(int slave);
enum
{
    EC_START,
    STANDBY,
    PULLOVER,
    HOMINGPEDALS,
    ESTOP,
    SELFTEST,
    CC,
    SELFDRIVING,
    EC_FINISH
};
////////////////////////////////////////////////////////////////////////
///////////////////////////HomingPedals Module//////////////////////////
////////////////////////////////////////////////////////////////////////
enum
{
    HP_START,
    HP_THROTTLE,
    HP_BRAKE,
    // HP_STEER,
    HP_FINISH
};

int homingThrottle()
{
    //set_rtParam("CruiseControl.target_velocity", "value", 0);
    throttleModule("Throttle.target_position", get_cfParam("Throttle.virtual_min_position"));
    return 1;
}
int homingBrake()
{
    brakeModule("Brake.target_position", get_cfParam("Brake.virtual_min_position"));
    return 1;
}
/*int homingSteer()
{
    disableMotor(get_cfParam("ECAT.motor_id_steer"));
    return 1;
}*/
int homingFinish()
{
    printf("HOMING!!!\n");
    set_rtParam("Motion.state", "value",STANDBY);
    return 1;
}
void initHA_homingpedals()
{
    homingpedals_HA = new HybridAutomata(HP_START, HP_FINISH);

    homingpedals_HA->setState(HP_THROTTLE, homingThrottle);
    homingpedals_HA->setState(HP_BRAKE, homingBrake);
    // homingpedals_HA->setState(HP_STEER, homingSteer);
    homingpedals_HA->setState(HP_FINISH, homingFinish);

    //homingpedals_HA->setCondition(HP_START, NULL, HP_FINISH);
    
    homingpedals_HA->setCondition(HP_START, NULL, HP_THROTTLE);
    homingpedals_HA->setCondition(HP_THROTTLE, NULL, HP_BRAKE);
    homingpedals_HA->setCondition(HP_BRAKE, NULL, HP_FINISH);
    
    // homingpedals_HA->setCondition(HP_STEER, NULL, HP_FINISH);
}
int operateHA_homingpedals()
{
    if(homingpedals_HA -> curState == HP_FINISH)
    {
       homingpedals_HA -> curState = HP_START;
    }
    return homingpedals_HA->operate();
    
}
////////////////////////////////////////////////////////////////////////
///////////////////////////end of HomingPedals Module///////////////////
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
///////////////////////////E-Stop Module////////////////////////////////
////////////////////////////////////////////////////////////////////////
enum
{
    E_START,
    E_STOP,
    E_FINISH
};

class ESTOP2ESTOP : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_brake")) < get_cfParam("Brake.virtual_max_position"))
            return true;
        else
            return false;
    }
};
class ESTOP2FINISH : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_brake")) >= get_cfParam("Brake.virtual_max_position"))
            return true;
        else
            return false;
    }
};
int estop()
{
    homingThrottle();
    brakeModule("Brake.target_position", get_cfParam("Brake.virtual_max_position"));
    // set_rtParam("CruiseControl.target_velocity", "value", 0);
    // reset_pid_values_for_cc();
    return 1;
}
int efinish()
{
    set_rtParam("Motion.state", "value",STANDBY);
    return 1;
}
void initHA_estop()
{
    estop_HA = new HybridAutomata(E_START, E_FINISH);

    estop_HA->setState(E_STOP, estop);
    estop_HA->setState(E_FINISH, efinish);

    ESTOP2ESTOP *estop2stop = new ESTOP2ESTOP();
    ESTOP2FINISH *estop2finish = new ESTOP2FINISH();

    estop_HA->setCondition(E_START, NULL, E_STOP);
    estop_HA->setCondition(E_STOP, estop2stop, E_STOP);
    estop_HA->setCondition(E_STOP, estop2finish, E_FINISH);
}
int operateHA_estop()
{
    if(estop_HA -> curState == E_FINISH)
    {
       estop_HA -> curState = E_START;
    }
    return estop_HA->operate();
    
}

////////////////////////////////////////////////////////////////////////
///////////////////////////end of E-Stop Module/////////////////////////
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
///////////////////////////Pullover Module//////////////////////////////
////////////////////////////////////////////////////////////////////////
enum
{
    PO_START,
    PO_DECIDE,
    PO,
    PO_FINISH
};
float pullover_tick_count = 0;
clock_t start_time,finish_time;
clock_t cur_time;

class START2DECIDE : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_brake")) < get_cfParam("Brake.virtual_max_position")-2)
        {
            printf("START2DECIDE %lf\n", get_slave_virtual_current_pos(1));
            start_time = clock();
            pullover_tick_count=0;
            return true;
        }
        else return false;
    }
};

class START2FINISH : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
         if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_brake")) >= get_cfParam("Brake.virtual_max_position")-2)
        {

            printf("START2FINISH %lf\n", get_slave_virtual_current_pos(1));
            
            return true;
        }
        else
            return false;
    }
};
class PO2DECIDE : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_brake")) < get_cfParam("Brake.virtual_max_position"))
        {
            printf("PO2DECIDE\n");
            return true;
        }
        else
            return false;
    }
};
class PO2FINISH : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_brake")) >= get_cfParam("Brake.virtual_max_position"))
        {
            printf("PO2FINISH\n");
            finish_time = clock();
            return true;
        }
        else
            return false;
    }
};

int decidePullover()
{
    static double init_vel;
    double cur_vel;
    static double delta_time;
    static double opt_vel;
    
    int err_margin = get_cfParam("Pullover.error_margin");
    double jerk = get_cfParam("Pullover.jerk");
    double pos_per_tick = (double)get_cfParam("Pullover.position_per_tick") / (double)get_cfParam("Brake.convert_position");
    

    /*if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_brake")) >= (double)get_cfParam("Brake.virtual_max_position"))
    {
    brakeModule("Brake.target_position", get_cfParam("Brake.virtual_max_position"));
    return 1;
    }*/
    homingThrottle();
    if (pullover_tick_count == 0)
    {
        init_vel = get_rtParam_double("CAN.actual_velocity", "value") / 3.6;
    }
    

    cur_vel = get_rtParam_double("CAN.actual_velocity", "value") / 3.6;
    delta_time = (double)(clock() - start_time) / 100000;
    opt_vel = (double)(jerk * delta_time) * delta_time + init_vel;

    printf("************************\n");
    printf("** init_vel = %.3lf\n", init_vel);
    printf("** cur_vel = %.3lf\n", cur_vel);
    printf("** delta_time = %.3lf\n", delta_time);
    printf("** opt_vel = %.3lf\n", opt_vel);
    printf("** opt_vel = %.3lf\n", opt_vel);
    printf("** cur_pos = %d\n", get_slave_physical_act_pos(get_cfParam("ECAT.motor_id_brake")));
    printf("** tar_pos = %d\n", get_slave_physical_target_pos(get_cfParam("ECAT.motor_id_brake")));
    printf("************************\n");

    if (opt_vel > cur_vel || opt_vel == cur_vel)
    {
        pullover_tick_count -= pos_per_tick;
    }
    else if (opt_vel < cur_vel)
    {
        pullover_tick_count += pos_per_tick;
    }
    return 1;

    //reset_pid_values_for_cc();
}
int pullover()
{

    double start_pos = (double)get_cfParam("Brake.start_position") / (double)get_cfParam("Brake.convert_position");
    double target_pos;
    /*if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_brake")) >= (double)get_cfParam("Brake.virtual_max_position"))
    {
    brakeModule("Brake.target_position", get_cfParam("Brake.virtual_max_position"));
    return 1;
    }
    */
    target_pos = start_pos + pullover_tick_count;
    if ((start_pos + pullover_tick_count) >= get_cfParam("Brake.virtual_max_position"))
    {
        target_pos = get_cfParam("Brake.virtual_max_position");
    }
    brakeModule("Brake.target_position", target_pos);
    return 1;
}
int pofinish()
{
    //set_rtParam("Motion.state", "value",STANDBY);
    printf("Pullover time : %lf\n",finish_time - start_time);
    return 1;
}
void initHA_pullover()
{
    pullover_HA = new HybridAutomata(PO_START, PO_FINISH);

    pullover_HA->setState(PO_DECIDE, decidePullover);
    pullover_HA->setState(PO, pullover);
    pullover_HA->setState(PO_FINISH, pofinish);

    START2DECIDE *s2d = new START2DECIDE();
    START2FINISH *s2f = new START2FINISH();
    PO2DECIDE *po2d = new PO2DECIDE();
    PO2FINISH *po2f = new PO2FINISH();

    pullover_HA->setCondition(PO_START, s2d, PO_DECIDE);
    pullover_HA->setCondition(PO_START, s2f, PO_FINISH);
    pullover_HA->setCondition(PO_DECIDE, NULL, PO);
    pullover_HA->setCondition(PO, po2d, PO_DECIDE);
    pullover_HA->setCondition(PO, po2f, PO_FINISH);
}
int operateHA_pullover()
{
    if(pullover_HA -> curState == PO_FINISH)
    {
       pullover_HA -> curState = PO_START;
    }
    return pullover_HA->operate();
    
}

////////////////////////////////////////////////////////////////////////
///////////////////////////end of Pullover Module///////////////////////
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
///////////////////////////Selftest Module//////////////////////////////
////////////////////////////////////////////////////////////////////////
enum
{
    ST_START,
    ST_T,
    ST_B,
    ST_G_MAX,
    ST_G_MIN,
    ST_S_MAX,
    ST_S_MIN,
    ST_S_O,
    ST_L_MAX,
    ST_L_MIN,
    ST_FINISH
};
class ST_T2T : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_throttle")) < (get_cfParam("Throttle.virtual_max_position") - 5))
            return true;
        else
            return false;
    }
};

class ST_T2B : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_throttle")) >= (get_cfParam("Throttle.virtual_max_position")-5))
            return true;
        else
            return false;
    }
};
class ST_B2B : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_brake")) < (get_cfParam("Brake.virtual_max_position")-5))
            return true;
        else
            return false;
    }
};
class ST_B2G_max : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_brake")) >= (get_cfParam("Brake.virtual_max_position")-5))
            return true;
        else
            return false;
    }
};
class ST_G_max2G_max : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if ((int)get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_gearstick")) != 3)
        {
            //printf(" %lf\t%d\n",get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_gearstick")) ,get_cfParam("Gearstick.drive_position"));
            return true;
        }
        else
            return false;
    }
};
class ST_G_max2G_min : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if ((int)get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_gearstick")) == 3)
            return true;
        else
            return false;
    }
};
class ST_G_min2G_min : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if ((int)get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_gearstick")) != 0)
            return true;
        else
            return false;
    }
};
class ST_G_min2S_max : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if ((int)get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_gearstick")) == 0)
            return true;
        else
            return false;
    }
};
class ST_S_max2S_max : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_steer")) < (get_cfParam("Steerwheel.virtual_max_position") - 1))
            return true;
        else
            return false;
    }
};
class ST_S_max2S_min : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_steer")) >= (get_cfParam("Steerwheel.virtual_max_position")-1))
            return true;
        else
            return false;
    }
};
class ST_S_min2S_min : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_steer")) > (get_cfParam("Steerwheel.virtual_min_position")+1))
            return true;
        else
            return false;
    }
};
class ST_S_min2S_o : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_steer")) <= get_cfParam("Steerwheel.virtual_min_position")+1)
            return true;
        else
            return false;
    }
};
class ST_S_o2S_o : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_steer")) != get_cfParam("Steerwheel.start_position"))
            return true;
        else
            return false;
    }
};
class ST_S_o2L_max : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_steer")) == get_cfParam("Steerwheel.start_position"))
            return true;
        else
            return false;
    }
};
class ST_L_max2L_max : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if ((get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_center")) < (get_cfParam("Lidar.center_max_degree") -10))  &&
        (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_left")) < (get_cfParam("Lidar.left_max_degree") -10)) &&
        (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_right")) > (get_cfParam("Lidar.right_min_degree")+10)))
        {
printf("L_MAX 2 L_MAX : %lf,%d\n%lf,%d\n%lf,%d\n", get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_center")) ,get_cfParam("Lidar.center_max_degree"), 
        get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_left")), get_cfParam("Lidar.left_max_degree"),
       get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_right")) , get_cfParam("Lidar.right_min_degree") );


        return true;
        }
        else
            return false;
    }
};
class ST_L_max2L_min : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if ((get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_center")) >= (get_cfParam("Lidar.center_max_degree")-10)) &&
        (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_left")) >= (get_cfParam("Lidar.left_max_degree") -10)) &&
        (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_right")) <= (get_cfParam("Lidar.right_min_degree")+10 )))
         {
printf("L_MAX 2 MIN : %lf,%d\n%lf,%d\n%lf,%d\n", get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_center")) ,get_cfParam("Lidar.center_max_degree"), 
        get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_left")), get_cfParam("Lidar.left_max_degree"),
       get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_right")) , get_cfParam("Lidar.right_min_degree") );


        return true;
        }
        else
            return false;
    }
};
class ST_L_min2L_min : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if ((get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_center")) > (get_cfParam("Lidar.center_min_degree") +10)) &&
        (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_left")) > (get_cfParam("Lidar.left_min_degree") + 10)) &&
        (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_right")) < (get_cfParam("Lidar.right_max_degree") -10 )))
            return true;
        else
            return false;
    }
};
class ST_L_min2F : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if ((get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_center")) <= (get_cfParam("Lidar.center_min_degree") + 10)) &&
        (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_left")) <= (get_cfParam("Lidar.left_min_degree") +10 )) &&
        (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_lidar_right")) >= (get_cfParam("Lidar.right_max_degree") -10 )))
            return true;
        else
            return false;
    }
};
int selftestThrottle()
{
    static double target_pos = 0;
    double pos_per_tick = (double)get_cfParam("SelfTest.Throttle.position_per_tick") / (double)get_cfParam("Throttle.convert_position");

    target_pos += pos_per_tick;
    throttleModule("Throttle.target_position", get_cfParam("Throttle.virtual_max_position") );
    return 1;
}
int selftestBrake()
{
    static double target_pos = 0;
    double pos_per_tick = (double)get_cfParam("SelfTest.Brake.position_per_tick") / (double)get_cfParam("Brake.convert_position");

    target_pos += pos_per_tick;
    homingThrottle();
    brakeModule("Brake.target_position", get_cfParam("Brake.virtual_max_position"));
    return 1;
}
int selftestGearstick_max()
{
    gearstickModule("Gearstick.target_position",3);
    return 1;
}
int selftestGearstick_min()
{
    gearstickModule("Gearstick.target_position",0);
    return 1;
}
int selftestSteer_max()
{
    homingBrake();
    steerModule("Steerwheel.target_position", get_cfParam("Steerwheel.virtual_max_position"));
    return 1;
}
int selftestSteer_min()
{
    steerModule("Steerwheel.target_position", get_cfParam("Steerwheel.virtual_min_position"));
    return 1;
}
int selftestSteer_origin()
{
    steerModule("Steerwheel.target_position", get_cfParam("Steerwheel.start_position"));
    return 1;
	}
int selftestLidars_max()
{
    lidarModule("Lidar.center_target_position", get_cfParam("Lidar.center_max_degree"));
    lidarModule("Lidar.left_target_position", get_cfParam("Lidar.left_max_degree"));
    lidarModule("Lidar.right_target_position", get_cfParam("Lidar.right_min_degree"));
    return 1;
}
int selftestLidars_min()
{
    lidarModule("Lidar.center_target_position", get_cfParam("Lidar.center_min_degree"));
    lidarModule("Lidar.left_target_position", get_cfParam("Lidar.left_min_degree"));
    lidarModule("Lidar.right_target_position", get_cfParam("Lidar.right_max_degree"));
    return 1;
}
int selftestFinish()
{
    set_rtParam("Motion.state", "value",STANDBY);
    return 1;
}
void initHA_selftest()
{
 
    selftest_HA = new HybridAutomata(ST_START, ST_FINISH);

    selftest_HA->setState(ST_T, selftestThrottle);
    selftest_HA->setState(ST_B, selftestBrake);
    selftest_HA->setState(ST_G_MAX, selftestGearstick_max);
    selftest_HA->setState(ST_G_MIN, selftestGearstick_min);
    selftest_HA->setState(ST_S_MAX, selftestSteer_max);
    selftest_HA->setState(ST_S_MIN, selftestSteer_min);
    selftest_HA->setState(ST_S_O, selftestSteer_origin);
    selftest_HA->setState(ST_L_MAX, selftestLidars_max);
    selftest_HA->setState(ST_L_MIN, selftestLidars_min);
    selftest_HA->setState(ST_FINISH, selftestFinish);
    

    ST_T2T *t2t = new ST_T2T();
    ST_T2B *t2b = new ST_T2B();
    ST_B2B *b2b = new ST_B2B();
    ST_B2G_max *b2g_max = new ST_B2G_max();
    ST_G_max2G_max *g_max2g_max = new ST_G_max2G_max();
    ST_G_max2G_min *g_max2g_min = new ST_G_max2G_min();
    ST_G_min2G_min *g_min2g_min = new ST_G_min2G_min();
    ST_G_min2S_max *g_min2s_max = new ST_G_min2S_max();
    ST_S_max2S_max *s_max2s_max = new ST_S_max2S_max();
    ST_S_max2S_min *s_max2s_min = new ST_S_max2S_min();
    ST_S_min2S_min *s_min2s_min = new ST_S_min2S_min();
    ST_S_min2S_o *s_min2s_o = new ST_S_min2S_o();
    ST_S_o2S_o *s_o2s_o = new ST_S_o2S_o();
    ST_S_o2L_max *s_o2l_max = new ST_S_o2L_max();
    ST_L_max2L_max *l_max2l_max = new ST_L_max2L_max();
    ST_L_max2L_min *l_max2l_min = new ST_L_max2L_min();
    ST_L_min2L_min *l_min2l_min = new ST_L_min2L_min();
    ST_L_min2F *l_min2F= new ST_L_min2F();

    selftest_HA->setCondition(ST_START, NULL, ST_T);
    selftest_HA->setCondition(ST_T, t2t, ST_T);
    selftest_HA->setCondition(ST_T, t2b, ST_B);
    selftest_HA->setCondition(ST_B, b2b, ST_B);
    selftest_HA->setCondition(ST_B, b2g_max, ST_G_MAX);
    selftest_HA->setCondition(ST_G_MAX, g_max2g_max, ST_G_MAX);
    selftest_HA->setCondition(ST_G_MAX, g_max2g_min, ST_G_MIN);
    selftest_HA->setCondition(ST_G_MIN, g_min2g_min, ST_G_MIN);
    selftest_HA->setCondition(ST_G_MIN, g_min2s_max, ST_S_MAX);
    selftest_HA->setCondition(ST_S_MAX, s_max2s_max, ST_S_MAX);
    selftest_HA->setCondition(ST_S_MAX, s_max2s_min, ST_S_MIN);
    selftest_HA->setCondition(ST_S_MIN, s_min2s_min, ST_S_MIN);
    selftest_HA->setCondition(ST_S_MIN, s_min2s_o, ST_S_O);
    selftest_HA->setCondition(ST_S_O, s_o2s_o, ST_S_O);
    selftest_HA->setCondition(ST_S_O, s_o2l_max, ST_L_MAX);
    selftest_HA->setCondition(ST_L_MAX, l_max2l_max, ST_L_MAX);
    selftest_HA->setCondition(ST_L_MAX, l_max2l_min, ST_L_MIN);
    selftest_HA->setCondition(ST_L_MIN, l_min2l_min, ST_L_MIN);
    selftest_HA->setCondition(ST_L_MIN, l_min2F, ST_FINISH);
}

int operateHA_selftest()
{
    if(selftest_HA -> curState == ST_FINISH)
    {
       selftest_HA -> curState = ST_START;
    }
    return selftest_HA->operate();
    
}

////////////////////////////////////////////////////////////////////////
///////////////////////////end of Selftest Module///////////////////////
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
/////////////////////////// Ready2Start Controlller/////////////////////
////////////////////////////////////////////////////////////////////////
extern int fix_steer();
extern int poselidarModule(char *arg1, double arg2);
extern int gearstickModule(char *arg1, double arg2);
enum
{
    R2S_START,
    R2S_BRAKE,
    R2S_FIXSTEER,
    R2S_POSELIDAR,
    R2S_GEARSTICK_D,
    R2S_FINISH
};

int R2Sfinish()
{
    printf("Ready to START!!\n");
    syslog(LOG_ERR,"Ready to START!!\n");
    set_rtParam("Motion.state", "value",STANDBY);
    return 1;
}
int poseLidar_drivingmode()
{
    if (get_cfParam("ECAT.motor_id_lidar_right") < get_rtParam_int("ECAT.num_motors", "value"))
    {
        poselidarModule("pl.mode" , 1);
        return 1;
    }
    return -1;
}
int gearStick_dmode()
{
    if (get_cfParam("ECAT.motor_id_gearstick") < get_rtParam_int("ECAT.num_motors", "value"))
    {
    	gearstickModule("g.tpos" , 3);
	    return 1;
    }
    return -1;
}
class R2S_B2B : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_brake")) < (get_cfParam("Brake.virtual_max_position")-5))
            return true;
        else
            return false;
    }
};
class R2S_B2FIX : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if (get_slave_virtual_current_pos(get_cfParam("ECAT.motor_id_brake")) >= (get_cfParam("Brake.virtual_max_position")-5))
            return true;
        else
            return false;
    }
};
class R2S_FIX2FIX : public Condition

{
  public:
    bool check(HybridAutomata *HA)
    {
        if(get_cfParam("ECAT.motor_id_steer") < get_rtParam_int("ECAT.num_motors", "value"))
        {
            if((get_rtParam_double("CAN.steering_angle", "value") > 10) ||
            (get_rtParam_double("CAN.steering_angle", "value") < -10))
        	    return true;
            else
                return false;
        }
        else return false;
    }
};

class R2S_FIX2POSELIDAR : public Condition
{
  public:
    bool check(HybridAutomata *HA)
    {
        if(get_cfParam("ECAT.motor_id_steer") < get_rtParam_int("ECAT.num_motors", "value"))
        {

            if((get_rtParam_double("CAN.steering_angle", "value") < 10) &&
            (get_rtParam_double("CAN.steering_angle", "value") > -10))
        	    return true;
            else
                return false;
        }
        else return true;
    }
};

void initHA_ready2start()
{
    ready2start_HA = new HybridAutomata(R2S_START,R2S_FINISH);
    
    ready2start_HA->setState(R2S_FIXSTEER, fix_steer);
    ready2start_HA->setState(R2S_BRAKE, estop);
    ready2start_HA->setState(R2S_POSELIDAR, poseLidar_drivingmode);
    ready2start_HA->setState(R2S_GEARSTICK_D, gearStick_dmode);
    ready2start_HA->setState(R2S_FINISH, R2Sfinish);


    R2S_B2B *b2b = new R2S_B2B();
    R2S_B2FIX *b2f =new R2S_B2FIX();
    R2S_FIX2FIX *f2f= new R2S_FIX2FIX();
    R2S_FIX2POSELIDAR *f2pl= new R2S_FIX2POSELIDAR();

    ready2start_HA->setCondition(R2S_START, NULL, R2S_BRAKE);
    ready2start_HA->setCondition(R2S_BRAKE, b2b, R2S_BRAKE);
    ready2start_HA->setCondition(R2S_BRAKE, b2f, R2S_FIXSTEER);
    ready2start_HA->setCondition(R2S_FIXSTEER, f2f, R2S_FIXSTEER);
    ready2start_HA->setCondition(R2S_FIXSTEER, f2pl, R2S_POSELIDAR);
    ready2start_HA->setCondition(R2S_POSELIDAR, NULL, R2S_GEARSTICK_D);
    ready2start_HA->setCondition(R2S_GEARSTICK_D, NULL, R2S_FINISH);

}
int operateHA_ready2start()
{
    if (ready2start_HA->curState == R2S_FINISH)
    {
        ready2start_HA->curState = R2S_START;
    }
    return ready2start_HA->operate();
}


////////////////////////////////////////////////////////////////////////
///////////////////////////end of Ready2Start Controlller//////////////
////////////////////////////////////////////////////////////////////////
