/*
 * Copyright 2019 Soongsil University. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ICHTHUS_CONTROLLER_H
#define ICHTHUS_CONTROLLER_H

#include <ros/ros.h>
#include "ichthus_controller/con_msg.h"   
#include "ecrt.h"
#include <sys/stat.h>
#include <time.h>   
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "autoware_msgs/VehicleCmd.h"
#include "ichthus_can/can_msg.h"

#define MAX_STATE_VARS 100
#define MAX_MOTORS 7

using namespace std;
using namespace ros;

namespace ichthus_controller
{
  enum var_prop { NO_PROP, STATE_PROP }; // state variable's property
  enum var_field { SV_ORIGIN, SV_STEP, SV_VALUE }; // state variable's data field
  enum motor_var_kind { MV_STATE, MV_POS, MV_VPOS, MV_APOS }; // motor state variable's kind
  enum ECAT_STATE {DOWN, UP, ON};
  enum MOTOR_STATE{DEACTIVATE, ACTIVATE, ENABLE};

  class state_var {
  public:
    double lower;  // sometimes called _left,  _de_step, _Kp, _min_radius, _park
    double upper;  // sometimes called _right, _ac_step, _Ki, _wheel_base, _drive
    double origin; // sometimes called _front, _jerk,    _Kd, _pos_per_deg
    double step;   // sometimes called _rear,  _margin,
    double value;
    // lidar pose parameter names
#define _left  lower
#define _right upper
#define _front origin
#define _rear  step
    // motion-related parameter names
#define _de_step lower
#define _ac_step upper
#define _jerk    origin
#define _margin  step
    // gearstick pose parameter names 
#define _park    lower
    //#define _reverse step
    //#define _neutral origin
#define _drive   upper
    // vehicle geometry parameter names
#define _min_radius  lower
#define _wheel_base  upper
#define _pos_per_deg origin
    // pid gain parameter names
#define _Kp lower
#define _Ki upper
#define _Kd origin

    int timestamp;
    var_prop property; // state variable's property
    int index; // state variable's index
    state_var(): lower(0), upper(0), origin(0), step(0), value(0), timestamp(0), property(NO_PROP), index(-1) {}
    void reset() {
      lower = 0;
      upper = 0;
      origin = 0;
      step = 0;
      value = 0;
      timestamp = 0;
      property = NO_PROP;
      index = -1;
    }
    string to_string() {
      ostringstream ss;
      ss << *this;
      return ss.str();
    }
    friend std::ostream& operator<<(std::ostream &out, state_var& var) {
      out << "[lower=" << var.lower
	  << ", upper=" << var.upper
	  << ", origin=" << var.origin
	  << ", step=" << var.step
	  << ", value=" << var.value
	  << ", timestamp=" << var.timestamp
	  << ", property=" << var.property
	  << ", index=" << var.index << "]";
      return out;
    }
  };

  extern void print_state_vars();
  extern bool get_state_var(string name, state_var& var);
  extern bool set_state_var(string name, double value);
  extern string get_state_varlist();
  extern string get_state_varlist_all();

#define VALID_MOTOR_ID(motor_id)  (((motor_id) >= 0) && ((motor_id) < num_motors))
  extern int num_motors;
  extern int motor_id_pedal_accel;
  extern int motor_id_pedal_decel;
  extern int motor_id_lidar_front;
  extern int motor_id_lidar_left;
  extern int motor_id_lidar_right;
  extern int motor_id_steer_wheel;
  extern int motor_id_gear_stick;
  extern map<string, state_var> stateVars;

  class IchthusController
  {
  private:
    ros::NodeHandle nh;

    ros::Publisher pub_current_accel_pos;
    ros::Publisher pub_target_accel_pos;
    ros::Publisher pub_current_decel_pos;
    ros::Publisher pub_target_decel_pos;
    ros::Publisher pub_current_steer_wheel_pos;
    ros::Publisher pub_target_steer_wheel_pos;
    ros::Publisher pub_controller_target_velocity;
    ros::Publisher pub_controller_current_velocity;
  public:
    IchthusController(void);
    ~IchthusController(void);
    void load_config_vars(void);
    void load_state_vars(void);
    void publish_topics(void);
    void callback_twist_cmd(const geometry_msgs::TwistStampedConstPtr &msg);
    void callback_vehicle_cmd(const autoware_msgs::VehicleCmdConstPtr& msg);
    void callback_CAN(const ichthus_can::can_msgConstPtr& msg);
    //void callback_OBD(const ichthus_controller::obd_msgConstPtr& msg);
    void callback_motion_state(const std_msgs::Float32ConstPtr& msg);
    void callback_brake_one_step(const std_msgs::BoolConstPtr& msg);
    void callback_mission_complete(const std_msgs::BoolConstPtr& msg);
    bool serviceController(con_msg::Request &req, con_msg::Response &res);
    void doLoop(void);
  };
  
}

#endif // ICHTHUS_CONTROLLER_H
