#include <stdio.h>
#include <string>
#include <map>
#include <cstring>
#include <iostream>
#include <unistd.h>

using namespace std;

extern void initEcatCyclicModules();
extern void initHA_ecat();
extern int operateHA_ecat();
extern void add_rtParam(pair<string, string> p, char *name, char *nickname,
                   map<string, double> m, int value, int timestamp, int version, int property);
extern int set_rtParam(char *name, char *field, double val);
extern int paramHandler(char *arg1, double arg2);
extern int EthercatCyclicHandler(char *arg1, double arg2);
extern int get_cfParam(char *name);
extern int get_rtParam_int(char *name, char *field);

map<string, double> Ecat_State;
pair<string, string> Ecat_State_name;
map<string, double> Num_Motors;
pair<string, string> Num_Motors_name;
map<string, double> Obd_Pub_2_Agent;
pair<string, string> Obd_Pub_2_Agent_name;
enum
{
    START,
    UP,
    ON,
    OFF,
    DOWN,
    FINISH
};

void initEcatModule()
{
    int init_num_motors = get_cfParam("ECAT.num_motors");
    initHA_ecat();
    add_rtParam(Obd_Pub_2_Agent_name, "OBD.publish_to_agent","OBD.pub2agent",Obd_Pub_2_Agent ,0, 0, 0, 2);
    add_rtParam(Ecat_State_name, "ECAT.state", "ECAT.state", Ecat_State, 0, 0, 0, 2);
    add_rtParam(Num_Motors_name, "ECAT.num_motors", "ECAT.num_motors", Num_Motors, init_num_motors, 0, 0, 2);
    initEcatCyclicModules();
}


int moduleEcatHandler(char *arg1, double arg2)
{
    int post_state = 0;
    if (strcmp(arg1, "up") == 0)
        post_state = UP;
    else if (strcmp(arg1, "on") == 0)
        post_state = ON;
    else if (strcmp(arg1, "off") == 0)
        post_state = OFF;
    else if (strcmp(arg1, "down") == 0)
        post_state = DOWN;
    else
        return -1;

    set_rtParam("ECAT.state", "value", post_state);
    return operateHA_ecat();
}

int EthercatManageHandler(char *module, char *arg1, double arg2)
{
    int ret_val = 0;
    int count = 0;
    if (strcmp(module, "Ecat") == 0)
    {
        ret_val = moduleEcatHandler(arg1, arg2);
        while(1)
        {
            if(ret_val == -1)
            {
                moduleEcatHandler("down", arg2);
                ret_val = moduleEcatHandler("up", arg2);
                count ++;
                sleep(1);
                if(count >= 10) return -1;
            }
            else return ret_val;
        }
        return ret_val;
    }
    else if (strcmp(module, "Param") == 0)
    {
        return paramHandler(arg1,arg2);
    }
    else if (strcmp(module, "Motion") == 0)
    {
        return EthercatCyclicHandler(arg1,arg2);
    }
    else if (strcmp(module, "Controller") == 0)
    {
        return EthercatCyclicHandler(arg1,arg2);
    }

    return -1;
}
