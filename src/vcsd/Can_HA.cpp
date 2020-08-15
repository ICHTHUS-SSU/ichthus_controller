#include "hybridautomata.h"

extern int canUp();
extern int canOn();
extern int canOff();
extern int canDown();

extern int get_rtParam_int(char* name, char* field);

HybridAutomata* can_HA;

enum {
    START,
    UP,
    ON,
    OFF,
    DOWN,
    FINISH
};
extern void sendDebug(char* ret_msg);
class CAN_UP2ON : public Condition {
public:
    bool check(HybridAutomata* HA) {
        if (get_rtParam_int("CAN.state", "value") == ON) {
            printf("CAN_UP2ON\n");
	        sendDebug("CAN_UP2ON\n");
            return true;
        }

        else
            return false;
    }
};

class CAN_ON2OFF : public Condition {
public:
    bool check(HybridAutomata* HA) {
        if (get_rtParam_int("CAN.state", "value") == OFF) {
            printf("CAN_ON2OFF\n");
	        sendDebug("CAN_ON2OFF\n");
            return true;
        }
        else
            return false;      
    }
};

class CAN_OFF2ON : public Condition {
public:
    bool check(HybridAutomata* HA) {
        if (get_rtParam_int("CAN.state", "value") == ON) {
            printf("CAN_OFF2ON\n");
	        sendDebug("CAN_OFF2ON\n");
            return true;
        }
        else
            return false;        
    }
};

class CAN_OFF2DOWN : public Condition {
public:
    bool check(HybridAutomata* HA) {
        if (get_rtParam_int("CAN.state", "value") == DOWN) {
            printf("CAN_OFF2DOWN\n");
	        sendDebug("CAN_OFF2DOWN\n");
            return true;
        }
        else
            return false;        
    }
};

void initHybridAutomata_can() {
    can_HA = new HybridAutomata(START, FINISH);
    can_HA->setState(UP, canUp);
    can_HA->setState(ON, canOn);
    can_HA->setState(OFF, canOff);
    can_HA->setState(DOWN, canDown);

    CAN_UP2ON* can_up2on = new CAN_UP2ON();
    CAN_ON2OFF* can_on2off = new CAN_ON2OFF();
    CAN_OFF2ON* can_off2on = new CAN_OFF2ON();
    CAN_OFF2DOWN* can_off2down = new CAN_OFF2DOWN();

    can_HA->setCondition(START, NULL, UP);
    can_HA->setCondition(UP, can_up2on, ON);
    can_HA->setCondition(ON, can_on2off, OFF);
    can_HA->setCondition(OFF, can_off2on, ON);
    can_HA->setCondition(OFF, can_off2down, DOWN);
}

int operateHybridAutomata_can() 
{
    return can_HA->operate();
}
