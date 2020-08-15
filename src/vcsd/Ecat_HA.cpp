#include <stdio.h>
#include <string>
#include <map>
#include <iostream>
#include "hybridautomata.h"

HybridAutomata *ecat_HA;

extern int ecatUp(); //from ecat.c
extern int ecatOn();
extern int ecatOff();
extern int ecatDown();
extern int get_rtParam_int(char *name, char *field); //from GlobalParam.cpp

enum 
{
  START,
  UP,
  ON,
  OFF,
  DOWN,
  FINISH
};


class ECAT_UP2ON : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    if (get_rtParam_int("ECAT.state", "value") == ON)
    {
      return true;
    }
    else
      return false;
  }
};
class ECAT_UP2DOWN : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    if (get_rtParam_int("ECAT.state", "value") == DOWN)
    {
      return true;
    }
    else
      return false;
  }
};
class ECAT_ON2OFF : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    if (get_rtParam_int("ECAT.state", "value") == OFF)
      return true;
    else
      return false;
  }
};

class ECAT_OFF2DOWN : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    if (get_rtParam_int("ECAT.state", "value") == DOWN)
      return true;
    else
      return false;
  }
};

class ECAT_DOWN2UP : public Condition
{
public:
  bool check(HybridAutomata *HA)
  {
    if (get_rtParam_int("ECAT.state","value") == UP)
      return true;
    else
      return false;
  }
};


void initHA_ecat()
{
  ecat_HA = new HybridAutomata(START, FINISH);

  ecat_HA->setState(UP, ecatUp);
  ecat_HA->setState(ON, ecatOn);
  ecat_HA->setState(OFF, ecatOff);
  ecat_HA->setState(DOWN, ecatDown);

  ECAT_UP2ON *ecat_up2on = new ECAT_UP2ON();
  ECAT_UP2DOWN *ecat_up2down = new ECAT_UP2DOWN();
  ECAT_ON2OFF *ecat_on2off = new ECAT_ON2OFF();
  ECAT_OFF2DOWN *ecat_off2down = new ECAT_OFF2DOWN();
  ECAT_DOWN2UP *ecat_down2up = new ECAT_DOWN2UP();

  ecat_HA->setCondition(START, NULL, UP);
  ecat_HA->setCondition(UP, ecat_up2on, ON);
  ecat_HA->setCondition(UP, ecat_up2down, DOWN);
  ecat_HA->setCondition(ON, ecat_on2off, OFF);
  ecat_HA->setCondition(OFF, ecat_off2down, DOWN);
  ecat_HA->setCondition(DOWN, ecat_down2up, UP);
}
int operateHA_ecat()
{
  return ecat_HA->operate();
}