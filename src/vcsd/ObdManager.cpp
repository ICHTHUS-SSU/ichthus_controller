#include <stdio.h>
#include <string>
#include <map>
#include <cstring>
#include <iostream>

using namespace std;
extern void initHA_obd();
extern int operateHA_obd();
extern void add_rtParam(pair<string, string> p, char *name, char *nickname,
                   map<string, double> m, int value, int timestamp, int version, int property);
extern int set_rtParam(char *name, char* field, double val);


map<string, double> Obd_State;
pair<string, string> Obd_State_name;
map<string, double> Obd_avel;
pair<string, string> Obd_avel_name;




enum 
{
  START,
  UP,
  ON,
  OFF,
  DOWN,
  FINISH
};

void initObdModule()
{
    initHA_obd();
    add_rtParam(Obd_State_name, "OBD.state", "OBD.state", Obd_State,0,0,0,2);
    add_rtParam(Obd_avel_name, "OBD.actual_velocity", "obd.avelo",Obd_avel,0,0,0,2);
    
}

int ObdManageHandler(char* value)
{
    int post_state =0;
    static int timestamp=0;
    if(strcmp(value,"up") == 0)
    {
        post_state = UP;
    }
    else if(strcmp(value,"on") == 0)
    {
        post_state = ON;
    }
    else if(strcmp(value,"off") == 0)
    {
        post_state = OFF;
    }
    else if(strcmp(value,"down") == 0)
    {
        post_state = DOWN;
    }
    else 
    {
        
        return -1;
    }

    set_rtParam("OBD.state","value",post_state);
    return operateHA_obd();
}