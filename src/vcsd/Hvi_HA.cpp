#include "hybridautomata.h"

extern int hviUp();
extern int hviOn();
extern int hviOff();
extern int hviDown();

extern int get_rtParam_int(char* name, char* field);

HybridAutomata *hvi_HA;

enum {
    START,
    UP,
    ON,
    OFF,
    DOWN,
    FINISH
};

class HVI_UP2ON : public Condition {
public:
    bool check(HybridAutomata *HA) {
        if (get_rtParam_int("HVI.state", "value") == ON) {
            cout << "HVI_UP2ON" << endl;
            return true;
        } 
        else return false;
    }
};

class HVI_ON2OFF : public Condition {
public:
    bool check(HybridAutomata *HA) {
        if (get_rtParam_int("HVI.state", "value") == OFF) {
            cout << "HVI_ON2OFF" << endl;
            return true;
        } 
        else return false;
    }
};

class HVI_OFF2ON : public Condition {
public:
    bool check(HybridAutomata *HA) {
        if (get_rtParam_int("HVI.state", "value") == ON) {
            cout << "HVI_OFF2ON" << endl; 
            return true;
        }
        else return false;
    }
};

class HVI_OFF2DOWN : public Condition {
public:
    bool check(HybridAutomata *HA) {
        if (get_rtParam_int("HVI.state", "value") == DOWN) {
            cout << "HVI_OFF2DOWN" << endl;
            return true;
        }
        else return false;
    }
};

void initHA_hvi() {
    hvi_HA = new HybridAutomata(START, FINISH);
    hvi_HA->setState(UP, hviUp);
    hvi_HA->setState(ON, hviOn);
    hvi_HA->setState(OFF, hviOff);
    hvi_HA->setState(DOWN, hviDown);

    HVI_UP2ON* hvi_up2on = new HVI_UP2ON();
    HVI_ON2OFF* hvi_on2off = new HVI_ON2OFF();
    HVI_OFF2ON* hvi_off2on = new HVI_OFF2ON();
    HVI_OFF2DOWN* hvi_off2down = new HVI_OFF2DOWN();

    hvi_HA->setCondition(START, NULL, UP);
    hvi_HA->setCondition(UP, hvi_up2on, ON);
    hvi_HA->setCondition(ON, hvi_on2off, OFF);
    hvi_HA->setCondition(OFF, hvi_off2on, ON);
    hvi_HA->setCondition(OFF, hvi_off2down, DOWN);
}

int operateHA_hvi() {
    return hvi_HA->operate();
}