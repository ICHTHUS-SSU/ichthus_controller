#include <stdio.h>
#include <string>
#include <map>
#include <cstring>
#include <iostream>
using namespace std;

extern void initHA_hvi();
extern int operateHA_hvi();
extern void add_rtParam(pair<string, string> p, char *name, char *nickname,
                   map<string, double> m, int value, int timestamp, int version, int property);
extern int set_rtParam(char *name, char* field, double val);

map<string, double> Hvi_State;
pair<string, string> Hvi_State_name;
map<string, double> Hvi_Mode;
pair<string, string> Hvi_Mode_name;
map<string, double> Hvi_Signtower;
pair<string, string> Hvi_Signtower_name;
map<string, double> Hvi_Buzzer;
pair<string, string> Hvi_Buzzer_name;

enum {
  START,
  UP,
  ON,
  OFF,
  DOWN,
  FINISH
};

void initHviModule() {
    initHA_hvi();
    add_rtParam(Hvi_State_name, "HVI.state", "HVI.state", Hvi_State,0,0,0,2);
    add_rtParam(Hvi_Mode_name, "HVI.mode", "HVI.mode", Hvi_Mode,0,0,0,2);
    add_rtParam(Hvi_Signtower_name, "HVI.signtower", "HVI.signtower", Hvi_Signtower,0,0,0,2);
    add_rtParam(Hvi_Buzzer_name, "HVI.buzzer", "HVI.buzzer", Hvi_Buzzer,0,0,0,2);
}

int HviManageHandler(char* value) {
    int post_state = 0;
    static int timestamp = 0;
    if(strcmp(value,"up") == 0){
        post_state = UP;
    }
    else if(strcmp(value,"on") == 0){
        post_state = ON;
    }
    else if(strcmp(value,"off") == 0){
        post_state = OFF;
    }
    else if(strcmp(value,"down") == 0){
        post_state = DOWN;
    }
    else {
        return -1;
    }

    set_rtParam("HVI.state", "value", post_state);
    return operateHA_hvi();
}
