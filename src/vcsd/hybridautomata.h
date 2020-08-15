
#pragma once
#include <iostream>

#define MAX_STATES 100

using namespace std;

class HybridAutomata;
class Condition
{
public:
  virtual bool check(HybridAutomata *HA) = 0;
};

class HybridAutomata
{
private:
  class State
  {
  public:
    unsigned int stateId;
    int (*aDo)();
    State(unsigned int id, int (*ah)())
    {
      stateId = id;
      aDo = ah;
    }
  };
  unsigned int initState, exitState;
  unsigned int nStates;
  bool stateMachine[MAX_STATES][MAX_STATES];
  State *states[MAX_STATES];
  Condition *conditions[MAX_STATES][MAX_STATES];
  void initStateMachineArr();
  void initConditionsArr();
  void initStateArr();
  bool checkStateMachine(unsigned int pre, unsigned int post);
  int checkConditions();

public:
  unsigned int curState;
  void setState(unsigned int id, int (*ah)());                                     // user sets states
  void setCondition(unsigned int preState, Condition *cDo, unsigned int postState); //user sets conditions
  int operate(); // check conditions and if condition has satisfied move to another state
  HybridAutomata(const unsigned int init, const unsigned int exit);
  HybridAutomata();
  ~HybridAutomata();
};
