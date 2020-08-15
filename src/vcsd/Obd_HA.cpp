#include "hybridautomata.h"


/////////////////////////////////////////////
// Imported definitions from obd2.cpp
/////////////////////////////////////////////


extern int obdUp();
extern int obdOn();
extern int obdOff();
extern int obdDown();

extern int get_rtParam_int(char *name, char* field);
/////////////////////////////////////////////
// Exported definitions
/////////////////////////////////////////////

HybridAutomata *obd_HA;

/////////////////////////////////////////////
// Local definitions
/////////////////////////////////////////////

enum    // for ObdMgr.cpp
{
    START,
    UP,
    ON,
    OFF,
    DOWN,
    FINISH
};



class OBD_UP2ON : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    if (get_rtParam_int("OBD.state","value") == ON)
    {
      cout << "OBD_UP2ON" << endl;
      return true;
    }
    else
      return false;
  }
};

class OBD_ON2OFF : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    if (get_rtParam_int("OBD.state","value") == OFF)
    {
      cout << "OBD_ON2OFF" << endl;
      return true;
    }
    else
      return false;
  }
};

class OBD_OFF2ON : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    if (get_rtParam_int("OBD.state","value") == ON)
    {
      cout << "OBD_OFF2ON" << endl;
      return true;
    }
    else
      return false;
  }
};

class OBD_OFF2DOWN : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    if (get_rtParam_int("OBD.state","value") == DOWN)
    {
      cout << "OBD_OFF2DOWN" << endl;
      return true;
    }
    else
      return false;
  }
};

/////////////////////////////////////////////
// Exported definitions
/////////////////////////////////////////////

void initHA_obd()
{
  obd_HA = new HybridAutomata(START, FINISH);
  obd_HA->setState(UP, obdUp);
  obd_HA->setState(ON, obdOn);
  obd_HA->setState(OFF, obdOff);
  obd_HA->setState(DOWN, obdDown);

  OBD_UP2ON *obd_up2on = new OBD_UP2ON();
  OBD_ON2OFF *obd_on2off = new OBD_ON2OFF();
  OBD_OFF2ON *obd_off2on = new OBD_OFF2ON();
  OBD_OFF2DOWN *obd_off2down = new OBD_OFF2DOWN();

  obd_HA->setCondition(START, NULL, UP);
  obd_HA->setCondition(UP, obd_up2on, ON);
  obd_HA->setCondition(ON, obd_on2off, OFF);
  obd_HA->setCondition(OFF, obd_off2on, ON);
  obd_HA->setCondition(OFF, obd_off2down, DOWN);
  //obd_HA->setCondition(,,);
}

int operateHA_obd()
{
  return obd_HA->operate();
}