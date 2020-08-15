#include "ichthus_controller/ichthus_controller.h"
#include <stdlib.h>
#include <cmath>  /* for std::abs(double) */

namespace ichthus_controller {

#define SET_STATE_VAR(var,val)	\
  do { \
    var.value = (val); \
    var.timestamp++; \
  } while (0)

  int num_motors;
  int motor_id_pedal_accel;
  int motor_id_pedal_decel;
  int motor_id_lidar_front;
  int motor_id_lidar_left;
  int motor_id_lidar_right;
  int motor_id_steer_wheel;
  int motor_id_gear_stick;
  
  bool update_vehicle_velocity = true; 

  map<string, state_var> state_vars;

  string state_var_list[MAX_STATE_VARS] = {
    // variables to be loaded from .yaml file
    "pedal_accel_pos", 
    "pedal_decel_pos", 
    "steer_wheel_pos", 
    "gear_stick_pos",  
    "lidar_front_pos", 
    "lidar_left_pos",  
    "lidar_right_pos", 
    "pedal_accel_vpos", 
    "pedal_decel_vpos", 
    "steer_wheel_vpos", 
    "gear_stick_vpos",  
    "lidar_front_vpos", 
    "lidar_left_vpos",  
    "lidar_right_vpos",
    "pedal_accel_apos", 
    "pedal_decel_apos", 
    "steer_wheel_apos", 
    "gear_stick_apos",  
    "lidar_front_apos", 
    "lidar_left_apos",  
    "lidar_right_apos", 
    "motion_pull_over",
    "cc_gains_accel_obd",
    "cc_gains_decel_obd",
    "cc_gains_accel_can",
    "cc_gains_decel_can",
    "cc_switch_margins",
    "cc_target_velocity",
    "cc_integral_base",
    "sc_vehicle_geometry",
    "motion_pedal_testing",
    "motion_lidar_homing",
    "motion_lidar_mapping",
    "motion_lidar_driving",
    "motion_lidar_parking",
    // variables not to be loaded from .yaml file
    "ecat_state",
    "pedal_accel_state", 
    "pedal_decel_state", 
    "steer_wheel_state", 
    "gear_stick_state",  
    "lidar_front_state", 
    "lidar_left_state",  
    "lidar_right_state", 
    "lidar_pose_mode",
    "motion_state",
    "vehicle_velocity_obd",
    "vehicle_velocity_can",
    "vehicle_steering_wheel_angle_can",
    "cc_target_velocity",
    "sc_target_anglvelo",
    "sc_target_angle",
    "velocity_update",
    "brake_one_step",
    ""
  };
  
  string motor_id_to_var_name(int motor_id, motor_var_kind kind) {
    string name;
    
    if (motor_id == motor_id_pedal_accel)
      name = "pedal_accel";
    else if (motor_id == motor_id_pedal_decel)
      name = "pedal_decel";
    else if (motor_id == motor_id_steer_wheel)
      name = "steer_wheel";
    else if (motor_id == motor_id_gear_stick)
      name = "gear_stick";
    else if (motor_id == motor_id_lidar_front)
      name = "lidar_front";
    else if (motor_id == motor_id_lidar_left)
      name = "lidar_left";
    else if (motor_id == motor_id_lidar_right)
      name = "lidar_right";
    else
      name = "unknown_motor";

    if (kind == MV_STATE)
      name += "_state";
    else if (kind == MV_POS)
      name += "_pos";
    else if (kind == MV_VPOS)
      name += "_vpos";
    else if (kind == MV_APOS)
      name += "_apos";

    return name;
  }
  
  string motor_id_to_pos_name(int motor_id) {
    return motor_id_to_var_name(motor_id, MV_POS);
  }

  string motor_id_to_vpos_name(int motor_id) {
    return motor_id_to_var_name(motor_id, MV_VPOS);
  }

  string motor_id_to_apos_name(int motor_id) {
    return motor_id_to_var_name(motor_id, MV_APOS);
  }

  string motor_id_to_state_name(int motor_id) {
    return motor_id_to_var_name(motor_id, MV_STATE);
  }
  
  double adjust_value_to_range(state_var& var, double val) {
    if (var.property != STATE_PROP || var.lower == var.upper)
      return val;

    if (val > var.upper) {
      cout << "set: value(" << val << ") adjusted to upper limit(" << var.upper << ")!!!" << endl;
      cout<<"var name : "<<var<<endl;
      cout<<var.index<<endl;
      val = var.upper;
    }

    if (val < var.lower) {
      cout << "set: value(" << val << ") adjusted to lower limit(" << var.lower << ")!!!" << endl;
      cout<<"var name : "<<var<<endl;
      val = var.lower;
    }

    return val;
  }

  bool sethand_no_write(state_var& var, double val) {
    cout << "set: read-only state variable!!!" << endl;
    return false;
  }
  
  bool sethand_default(state_var& var, double val) {
    SET_STATE_VAR(var, adjust_value_to_range(var, val));
    return true;
  }
  
  bool force_to_set_state_var(string name, double val, var_field field_id) {
    map<string, state_var>::iterator it;

    it = state_vars.find(name.c_str());
    if (it == state_vars.end())
      return false;

    if (field_id == SV_VALUE)
      sethand_default(it->second, val);
    else if (field_id == SV_ORIGIN)
      it->second.origin = val;
    else if (field_id == SV_STEP)
      it->second.step = val;
      
    //cout << "set: " << it->first.c_str() << it->second << " (handler returns true)" << endl;
    return true;
  }

  void print_state_vars(void)  {
    map<string, state_var>::iterator it;
    int n = 0;

    for (n = 0, it = state_vars.begin(); it != state_vars.end(); ++it, n++) {
      cout << "state_var[" << n << "]: "
	   << it->first.c_str()
	   << it->second << endl;
    }
  }

  //////////////////////////////////////////////////////////////////
  // Functions to get state variables
  //////////////////////////////////////////////////////////////////  
  double gethand_default(state_var& var) { return var.value;  }
  typedef double get_handler_type(state_var&);

  double __gethand_motor_apos(state_var& var, int motor_id) {
    extern bool get_actual_pos(int slave_no, int& value);
    extern bool is_motor_on(int motor_id);
    int value;

    if (!is_motor_on(motor_id))
      return var.value;

    if (get_actual_pos(motor_id, value)) {
      var.value = (double) value;
      return var.value;
    }

    return var.value;
  }
  
  double gethand_pedal_accel_apos(state_var& var) {
    return __gethand_motor_apos(var, motor_id_pedal_accel);    
  }

  double gethand_pedal_decel_apos(state_var& var) {
    return __gethand_motor_apos(var, motor_id_pedal_decel);    
  }

  double gethand_steer_wheel_apos(state_var& var) {
    return __gethand_motor_apos(var, motor_id_steer_wheel);    
  }

  double gethand_gear_stick_apos(state_var& var) {
    return __gethand_motor_apos(var, motor_id_gear_stick);    
  }

  double gethand_lidar_front_apos(state_var& var) {
    return __gethand_motor_apos(var, motor_id_lidar_front);    
  }

  double gethand_lidar_left_apos(state_var& var) {
    return __gethand_motor_apos(var, motor_id_lidar_left);    
  }

  double gethand_lidar_right_apos(state_var& var) {
    return __gethand_motor_apos(var, motor_id_lidar_right);    
  }
  
  get_handler_type* get_handler[MAX_STATE_VARS] = {
    // variables to be loaded from .yaml file
    gethand_default, // "pedal_accel_pos", 
    gethand_default, // "pedal_decel_pos", 
    gethand_default, // "steer_wheel_pos", 
    gethand_default, // "gear_stick_pos",  
    gethand_default, // "lidar_front_pos", 
    gethand_default, // "lidar_left_pos",  
    gethand_default, // "lidar_right_pos", 
    gethand_default, // "pedal_accel_vpos", 
    gethand_default, // "pedal_decel_vpos", 
    gethand_default, // "steer_wheel_vpos", 
    gethand_default, // "gear_stick_vpos",  
    gethand_default, // "lidar_front_vpos", 
    gethand_default, // "lidar_left_vpos",  
    gethand_default, // "lidar_right_vpos", 
    gethand_pedal_accel_apos, // "pedal_accel_apos", 
    gethand_pedal_decel_apos, // "pedal_decel_apos", 
    gethand_steer_wheel_apos, // "steer_wheel_apos", 
    gethand_gear_stick_apos,  // "gear_stick_apos",  
    gethand_lidar_front_apos, // "lidar_front_apos", 
    gethand_lidar_left_apos,  // "lidar_left_apos",  
    gethand_lidar_right_apos, // "lidar_right_apos", 
    gethand_default, // "motion_pull_over",
    gethand_default, // "cc_gains_accel_obd",
    gethand_default, // "cc_gains_decel_obd",
    gethand_default, // "cc_gains_accel_can",
    gethand_default, // "cc_gains_decel_can",
    gethand_default, // "cc_switch_margins",
    gethand_default, // "cc_target_velocity",
    gethand_default, // "cc_integral_base",
    gethand_default, // "sc_vehicle_geometry",
    gethand_default, // "motion_pedal_testing",
    gethand_default, // "motion_lidar_homing",
    gethand_default, // "motion_lidar_mapping",
    gethand_default, // "motion_lidar_driving",
    gethand_default, // "motion_lidar_parking",
    // variables not to be loaded from .yaml file
    gethand_default, // "ecat_state",
    gethand_default, // "pedal_accel_state",
    gethand_default, // "pedal_decel_state",
    gethand_default, // "steer_wheel_state",    
    gethand_default, // "gear_stick_state",
    gethand_default, // "lidar_front_state",
    gethand_default, // "lidar_left_state",
    gethand_default, // "lidar_right_state",
    gethand_default, // "lidar_pose_mode",
    gethand_default, // "motion_state",
    gethand_default, // "vehicle_velocity_obd",
    gethand_default, // "vehicle_velocity_can",
    gethand_default, // "vehicle_steering_angle_can",
    gethand_default, // "cc_target_velocity",
    gethand_default, // "sc_target_anglvelo",
    gethand_default, // "sc_target_angle"
    gethand_default, // "velocity_update" 20200121
    gethand_default,
    NULL
  };

  bool get_state_var(string name, state_var& var);
  
  bool get_state_var_timestamp(string name, int& timestamp) {
    state_var var;

    if (!get_state_var(name, var)) // no such variable
      return false;

    timestamp = var.timestamp;
    return true;
  }

  bool get_state_var_upper(string name, double& value) {
    state_var var;

    if (!get_state_var(name, var)) // no such variable
      return false;

    value = var.upper;
    return true;
  }

  bool get_state_var_lower(string name, double& value) {
    state_var var;

    if (!get_state_var(name, var)) // no such variable
      return false;

    value = var.lower;
    return true;
  }

  bool get_state_var_origin(string name, double& value) {
    state_var var;

    if (!get_state_var(name, var)) // no such variable
      return false;

    value = var.origin;
    return true;
  }

  bool get_state_var_step(string name, double& value) {
    state_var var;

    if (!get_state_var(name, var)) // no such variable
      return false;

    value = var.step;
    return true;
  }

  bool get_state_var_value(string name, double& value) {
    state_var var;

    if (!get_state_var(name, var)) // no such variable
      return false;

    value = var.value;
    return true;
  }
  
  bool does_state_var_reach_goal(string name, double ref) {
    state_var var;

    if (!get_state_var(name, var)) // no such variable
      return false;

    if (var.value < ref - var.step || var.value > ref + var.step)
      return false;
    
    return true; // ref - var.step <= var.value <= ref + var.step
  }
  
  bool is_motor_on(int motor_id) {
    double value;
    string name;

    if (!VALID_MOTOR_ID(motor_id)) return false;

    get_state_var_value("ecat_state", value);
    if (value != 2) {
      cout << "ecat is not on!!!" << endl;
      return false;
    }
    
    name = motor_id_to_state_name(motor_id);
    get_state_var_value(name, value);
    if (value != 2) {
      cout << "motor is not on!!!" << endl;
      return false;
    }

    return true;
  }
  
  //////////////////////////////////////////////////////////////////
  // Functions to set state variables
  //////////////////////////////////////////////////////////////////
  typedef bool set_handler_type(state_var&, double);
  extern set_handler_type* set_handler[];
  
  bool set_state_var(string name, double value);
  
  bool __sethand_motor_pos(state_var& var, double val, int motor_id) {
    extern bool set_target_pos(int slave_no, int value);
    map<string, state_var>::iterator it;
    string name = motor_id_to_vpos_name(motor_id);

    if (!is_motor_on(motor_id))
      return false;
    
    // get an iterator to virtual position variable
    it = state_vars.find(name.c_str());
    if (it == state_vars.end())
      return false;

    // set val to physical position variable
    SET_STATE_VAR(var, adjust_value_to_range(var, val));
    set_target_pos(motor_id, var.value);
    
    // virtual position = (physical position - physical lower bound)/physical_step
    if(var.lower < 0)
      val = var.value / var.step;
    else
      val = (var.value - var.lower) / var.step; 
  
    // set val to virtual position variable
    it->second.value = val; // in this case, we do not update the timestamp
    
    //cout << "set: " << it->first.c_str() << it->second << endl;
    return true;
  }
  
  bool sethand_pedal_accel_pos(state_var& var, double val) {
    return __sethand_motor_pos(var, val, motor_id_pedal_accel);    
  }

  bool sethand_pedal_decel_pos(state_var& var, double val) {
    return __sethand_motor_pos(var, val, motor_id_pedal_decel);    
  }

  bool sethand_steer_wheel_pos(state_var& var, double val) {
    return __sethand_motor_pos(var, val, motor_id_steer_wheel);    
  }

  bool sethand_gear_stick_pos(state_var& var, double val) {
    return __sethand_motor_pos(var, val, motor_id_gear_stick);    
  }

  bool sethand_lidar_front_pos(state_var& var, double val) {
    return __sethand_motor_pos(var, val, motor_id_lidar_front);    
  }

  bool sethand_lidar_left_pos(state_var& var, double val) {
    return __sethand_motor_pos(var, val, motor_id_lidar_left);    
  }

  bool sethand_lidar_right_pos(state_var& var, double val) {
    return __sethand_motor_pos(var, val, motor_id_lidar_right);    
  }

  bool __sethand_motor_vpos(state_var& var, double val, int motor_id) {
    map<string, state_var>::iterator it;
    set_handler_type *handler;
    string name = motor_id_to_pos_name(motor_id);

    if (!is_motor_on(motor_id))
      return false;
    
    // get an iterator to physical position variable
    it = state_vars.find(name.c_str());
    if (it == state_vars.end())
      return false;

    // set val to virtual position variable
    SET_STATE_VAR(var, adjust_value_to_range(var, val));

    // physical position = virtual position * physical step
    val = var.value * it->second.step; 

    // set val to physical position variable
    handler = set_handler[it->second.index];
    if (!handler(it->second, val)) {
      cout << "set: " << it->first.c_str() << it->second << " (handler returns false)" << endl;
      return true; // return true even if the handler returns false
    }

    //cout << "set: " << it->first.c_str() << it->second << " (handler returns true)" << endl;
    return true;
  }
  
  bool sethand_pedal_accel_vpos(state_var& var, double val) {
    return __sethand_motor_vpos(var, val, motor_id_pedal_accel);
  }

  bool sethand_pedal_decel_vpos(state_var& var, double val) {
    return __sethand_motor_vpos(var, val, motor_id_pedal_decel);
  }

  bool sethand_steer_wheel_vpos(state_var& var, double val) {
    return __sethand_motor_vpos(var, val, motor_id_steer_wheel);
  }

  bool sethand_gear_stick_vpos(state_var& var, double val) {
    return __sethand_motor_vpos(var, val, motor_id_gear_stick);
  }

  bool sethand_lidar_front_vpos(state_var& var, double val) {
    return __sethand_motor_vpos(var, val, motor_id_lidar_front);
  }

  bool sethand_lidar_left_vpos(state_var& var, double val) {
    return __sethand_motor_vpos(var, val, motor_id_lidar_left);
  }

  bool sethand_lidar_right_vpos(state_var& var, double val) {
    return __sethand_motor_vpos(var, val, motor_id_lidar_right);
  }
  
  bool force_to_set_all_motor_states(double val) {
    if (VALID_MOTOR_ID(motor_id_pedal_accel)) force_to_set_state_var("pedal_accel_state", val, SV_VALUE);
    if (VALID_MOTOR_ID(motor_id_pedal_decel)) force_to_set_state_var("pedal_decel_state", val, SV_VALUE);
    if (VALID_MOTOR_ID(motor_id_steer_wheel)) force_to_set_state_var("steer_wheel_state", val, SV_VALUE);
    if (VALID_MOTOR_ID(motor_id_gear_stick))  force_to_set_state_var("gear_stick_state",  val, SV_VALUE);
    if (VALID_MOTOR_ID(motor_id_lidar_front)) force_to_set_state_var("lidar_front_state", val, SV_VALUE);
    if (VALID_MOTOR_ID(motor_id_lidar_left))  force_to_set_state_var("lidar_left_state",  val, SV_VALUE);
    if (VALID_MOTOR_ID(motor_id_lidar_right)) force_to_set_state_var("lidar_right_state", val, SV_VALUE);
    return true;
  }
  
  bool sethand_ecat_state(state_var& var, double val) {
    extern bool ecat_up();   // master up,  all slaves(=motors) activated
    extern bool ecat_on();   // thread on,  all slaves(=motors) deactivated, activated and enabled
    extern bool ecat_off();  // thread off, all slaves(=motors) disabled, i.e., activated
    extern bool ecat_down(); // master down
    
    if (var.value == 0) { // down state
      if (val == 1) { // transition to up state
        ecat_up();
	force_to_set_all_motor_states(val);
	SET_STATE_VAR(var, val);
        return true;
      }
      else
        return false;
    }
    else if (var.value == 1) { // up state
      if (val == 0) {  // transition to down state
        ecat_down();
	force_to_set_all_motor_states(val);
	SET_STATE_VAR(var, val);
        return true;
      }
      else if (val == 2) {  // transition to on state
        ecat_on();
	force_to_set_all_motor_states(val);	
	SET_STATE_VAR(var, val);
        return true;
      }
      else
        return false;
    }
    else if (var.value == 2) { // on state
      if (val == 1) { // transition to up state
        ecat_off();
	force_to_set_all_motor_states(val);
	SET_STATE_VAR(var, val);
        return true;
      }
      else
        return false;
    }
    else
      return false;
  }

  bool __sethand_motor_state(state_var& var, double val, int motor_id) {
    extern bool activate_slave(int slave_no);
    extern bool deactivate_slave(int slave_no);
    extern bool enable_slave(int slave_no);
    extern bool disable_slave(int slave_no);

    if (!VALID_MOTOR_ID(motor_id))
      return false;

    if (var.value == 0) { // deactivated state
      if (val == 1) { // transtion to activated state
        activate_slave(motor_id);
	SET_STATE_VAR(var, val);
        return true;
      }
      else
        return false;
    }
    else if (var.value == 1) { // activated state
      if (val == 0) { // transition to deactivated state
        deactivate_slave(motor_id);
	SET_STATE_VAR(var, val);
        return true;
      }
      else if (val == 2) { // transition to enabled state
        enable_slave(motor_id);
	SET_STATE_VAR(var, val);
        return true;
      }
      else
        return false;
    }
    else if (var.value == 2) { // enabled state
      if (val == 1) { // transition to disabled state == activated state
        disable_slave(motor_id);
	SET_STATE_VAR(var, val);
        return true;
      }
      else
        return false;
    }
    else
      return false;
  }
  
  bool sethand_pedal_accel_state(state_var& var, double val) {
    return __sethand_motor_state(var, val, motor_id_pedal_accel);
  }

  bool sethand_pedal_decel_state(state_var& var, double val) {
    return __sethand_motor_state(var, val, motor_id_pedal_decel);
  }

  bool sethand_steer_wheel_state(state_var& var, double val) {
    return __sethand_motor_state(var, val, motor_id_steer_wheel);
  }

  bool sethand_gear_stick_state(state_var& var, double val) {
    return __sethand_motor_state(var, val, motor_id_gear_stick);
  }

  bool sethand_lidar_front_state(state_var& var, double val) {
    return __sethand_motor_state(var, val, motor_id_lidar_front);
  }

  bool sethand_lidar_left_state(state_var& var, double val) {
    return __sethand_motor_state(var, val, motor_id_lidar_left);
  }

  bool sethand_lidar_right_state(state_var& var, double val) {
    return __sethand_motor_state(var, val, motor_id_lidar_right);
  }

  bool sethand_motion_state(state_var& var, double val) {
    extern bool motion_in_progress;

    int state = var.value;
    int new_state = val;
    
    if (state == 0) { // initial state
      if (new_state == 1) { // transition to standby state
	SET_STATE_VAR(var, new_state);
        return true;
      }
      else
        return false;
    }
    else if (state == 1) { // standby state
      if (new_state == 0) {  // transition to initial state
	SET_STATE_VAR(var, new_state);
        return true;
      }
      else if (new_state >= 2) {  // transition to any motion state
	SET_STATE_VAR(var, new_state);
        return true;
      }
      else
        return false;
    }
    else if (state >= 2) { 
      // if (motion_in_progress){ // motion in progress
      //   return false;
      // }

      if (new_state < 1)
	return false;
      else {
	SET_STATE_VAR(var, new_state);
	motion_in_progress = true;
	// it automatically transitions to standby state
	// by run_controller() when the motion is done
	return true;
      }
    }
    else
      return false;
  }

  bool sethand_velocity_update(state_var& var, double val){
    if((bool)val == false)
      update_vehicle_velocity = false;
    else
      update_vehicle_velocity = true;
    var.value = val;
    return true;
  }

  set_handler_type* set_handler[MAX_STATE_VARS] = {
    // variables to be loaded from .yaml file
    sethand_pedal_accel_pos, // "pedal_accel_pos", 
    sethand_pedal_decel_pos, // "pedal_decel_pos", 
    sethand_steer_wheel_pos, // "steer_wheel_pos", 
    sethand_gear_stick_pos,  // "gear_stick_pos",  
    sethand_lidar_front_pos, // "lidar_front_pos", 
    sethand_lidar_left_pos,  // "lidar_left_pos",  
    sethand_lidar_right_pos, // "lidar_right_pos", 
    sethand_pedal_accel_vpos, // "pedal_accel_vpos", 
    sethand_pedal_decel_vpos, // "pedal_decel_vpos", 
    sethand_steer_wheel_vpos, // "steer_wheel_vpos", 
    sethand_gear_stick_vpos,  // "gear_stick_vpos",  
    sethand_lidar_front_vpos, // "lidar_front_vpos", 
    sethand_lidar_left_vpos,  // "lidar_left_vpos",  
    sethand_lidar_right_vpos, // "lidar_right_vpos", 
    sethand_no_write, // "pedal_accel_apos", 
    sethand_no_write, // "pedal_decel_apos", 
    sethand_no_write, // "steer_wheel_apos", 
    sethand_no_write, // "gear_stick_apos",  
    sethand_no_write, // "lidar_front_apos", 
    sethand_no_write, // "lidar_left_apos",  
    sethand_no_write, // "lidar_right_apos", 
    sethand_default, // "motion_pull_over",
    sethand_default, // "cc_gains_accel_obd",
    sethand_default, // "cc_gains_decel_obd",
    sethand_default, // "cc_gains_accel_can",
    sethand_default, // "cc_gains_decel_can",
    sethand_default, // "cc_switch_margins",
    sethand_default, // "cc_target_velocity",
    sethand_default, // "cc_integral_base",
    sethand_default, // "sc_vehicle_geometry",
    sethand_default, // "motion_pedal_testing",
    sethand_default, // "motion_lidar_homing",
    sethand_default, // "motion_lidar_mapping",
    sethand_default, // "motion_lidar_driving",
    sethand_default, // "motion_lidar_parking",
    // variables not to be loaded from .yaml file
    sethand_ecat_state, // "ecat_state",
    sethand_pedal_accel_state, // "pedal_accel_state",
    sethand_pedal_decel_state, // "pedal_decel_state",
    sethand_steer_wheel_state, // "steer_wheel_state",    
    sethand_gear_stick_state,  // "gear_stick_state",
    sethand_lidar_front_state, // "lidar_front_state",
    sethand_lidar_left_state,  // "lidar_left_state",
    sethand_lidar_right_state, // "lidar_right_state",
    sethand_default, // "lidar_pose_mode",
    sethand_motion_state, // "motion_state",
    sethand_default, // "vehicle_velocity_obd",
    sethand_default, // "vehicle_velocity_can",
    sethand_default, // "vehicle_steering_angle_can",
    sethand_default, // "cc_target_velocity",
    sethand_default, // "sc_target_anglvelo",
    sethand_default, // "sc_target_angle"
    sethand_velocity_update, // "velocity_update" //20200121
    sethand_default,
    NULL
  };

  /////////////////////////////////////////////////////////////////////
  // State variable related functions used by IchthusController class
  /////////////////////////////////////////////////////////////////////
  
  bool get_state_var(string name, state_var& var) {
    map<string, state_var>::iterator it;
    get_handler_type *handler;

    it = state_vars.find(name.c_str());
    if (it == state_vars.end())
      return false;

    handler = get_handler[it->second.index];
    it->second.value = handler(it->second);
    
    //cout << "get: " << it->first.c_str() << it->second << endl;
    var = it->second;
    return true;
  }
  
  bool set_state_var(string name, double value) {
    map<string, state_var>::iterator it;
    set_handler_type *handler;
  
    it = state_vars.find(name.c_str());
    if (it == state_vars.end())
      return false;
    
    handler = set_handler[it->second.index];
    if (!handler(it->second, value)) {
      cout << "set: " << it->first.c_str() << it->second << " (handler returns false)" << endl;
      return true; // return true even if the handler returns false
    }

    //cout << "set: " << it->first.c_str() << it->second << " (handler returns true)" << endl;
    return true;
  }
  
  string get_state_varlist(void) {
    ostringstream ss;
    map<string, state_var>::iterator it;

    it = state_vars.begin();
    ss << "[ " << it->first.c_str();
    ++it;
    for (; it != state_vars.end(); ++it)
      ss << ", " << it->first.c_str();
    ss << " ]";

    return ss.str();
  }

  string get_state_varlist_all(void)  {
    ostringstream ss;
    map<string, state_var>::iterator it;

    it = state_vars.begin();
    ss << "[ " << endl;
    ss << it->first.c_str() << it->second << endl;
    ++it;
    for (; it != state_vars.end(); ++it)
      ss << it->first.c_str() << it->second << endl;
    ss << " ]";

    return ss.str();
  }

  bool normalize_position_variables(const string& vpos_var_name, const string& pos_var_name) {
    state_var vpos_var, pos_var;
    double vrange, range;

    if (!get_state_var(vpos_var_name, vpos_var)) // no such variable
      return false;

    if (!get_state_var(pos_var_name, pos_var)) // no such variable
      return false;
    
    vrange = vpos_var.upper - vpos_var.lower;
    range  = pos_var.upper - pos_var.lower;

    if (vrange == 0 || range == 0) { // do not allow divide-by-zero
      cout << pos_var_name << pos_var << endl;
      cout << vpos_var_name << vpos_var << endl;
      return false;
    }
    
    pos_var.step    = range / vrange;

    if(vpos_var.lower < 0)
      vpos_var.origin = vrange * (pos_var.upper + pos_var.lower) / range;
    else
      vpos_var.origin = vrange * (pos_var.origin - pos_var.lower) / range;
    
    // update the state variables in terms of step or origin
    force_to_set_state_var(pos_var_name,  pos_var.step,    SV_STEP);
    force_to_set_state_var(vpos_var_name, vpos_var.origin, SV_ORIGIN);
    return true;
  }
  
  //////////////////////////////////////////////////////////////////
  // Member functions of IchthusController class
  //////////////////////////////////////////////////////////////////  
  
  IchthusController::IchthusController() {
    pub_current_accel_pos = nh.advertise<std_msgs::Float32>("/current_accel_pos",1);
    pub_target_accel_pos = nh.advertise<std_msgs::Float32>("/target_accel_pos",1);
    pub_current_decel_pos = nh.advertise<std_msgs::Float32>("/current_decel_pos",1);
    pub_target_decel_pos = nh.advertise<std_msgs::Float32>("/target_decel_pos",1);
    pub_current_steer_wheel_pos = nh.advertise<std_msgs::Float32>("/current_steer_wheel_pos",1);
    pub_target_steer_wheel_pos =nh.advertise<std_msgs::Float32>("/target_steer_wheel_pos",1);
    pub_controller_target_velocity = nh.advertise<std_msgs::Float32>("/controller_target_velocity",1);
    pub_controller_current_velocity = nh.advertise<std_msgs::Float32>("/controller_current_velocity",1);
  }
  IchthusController::~IchthusController() {}

  void IchthusController::load_state_vars()  {
    nh.getParam("num_motors", num_motors);
    nh.getParam("motor_id_pedal_accel", motor_id_pedal_accel);
    nh.getParam("motor_id_pedal_decel", motor_id_pedal_decel);
    nh.getParam("motor_id_steer_wheel", motor_id_steer_wheel);
    nh.getParam("motor_id_lidar_front", motor_id_lidar_front);
    nh.getParam("motor_id_gear_stick",  motor_id_gear_stick);
    nh.getParam("motor_id_lidar_left",  motor_id_lidar_left);
    nh.getParam("motor_id_lidar_right", motor_id_lidar_right);

    for (int i = 0; state_var_list[i] != ""; i++) {
      string name = state_var_list[i];
      state_var var;
      var.index = i;
      var.property = STATE_PROP;

      // variable with default ranging parameters
      nh.getParam((name + "/lower").c_str(),  var.lower);
      nh.getParam((name + "/upper").c_str(),  var.upper);
      nh.getParam((name + "/origin").c_str(), var.origin);
      nh.getParam((name + "/step").c_str(),   var.step);

      // variable with lidar pose parameters
      nh.getParam((name + "/left").c_str(),  var._left);
      nh.getParam((name + "/right").c_str(), var._right);
      nh.getParam((name + "/front").c_str(), var._front);
      nh.getParam((name + "/rear").c_str(),  var._rear);

      // variable with motion-related parameters
      nh.getParam((name + "/jerk").c_str(),    var._jerk);
      nh.getParam((name + "/margin").c_str(),  var._margin);
      nh.getParam((name + "/ac_step").c_str(), var._ac_step);
      nh.getParam((name + "/de_step").c_str(), var._de_step);

#if 0      
      // variable with gearstick pose parameters      
      nh.getParam((name + "/park").c_str(),    var._park);
      nh.getParam((name + "/reverse").c_str(), var._reverse);
      nh.getParam((name + "/neutral").c_str(), var._neutral);
      nh.getParam((name + "/drive").c_str(),   var._drive);
#endif
      
      // variable with pid gain parameters      
      nh.getParam((name + "/Kp").c_str(), var._Kp);
      nh.getParam((name + "/Ki").c_str(), var._Ki);
      nh.getParam((name + "/Kd").c_str(), var._Kd);

      // variable with vehicle geometry parameters      
      nh.getParam((name + "/min_radius").c_str(), var._min_radius);
      nh.getParam((name + "/wheel_base").c_str(), var._wheel_base);
      nh.getParam((name + "/pos_per_deg").c_str(), var._pos_per_deg);

      // variable with no parameters
      state_vars.insert(pair<string,state_var>(state_var_list[i], var));
    }

    // normalize some position variables so that controller algorithms can manipulate them.
    if (!normalize_position_variables("pedal_accel_vpos", "pedal_accel_pos")) {
      cout << "cannot normalize pedal_accel_pos" << endl;
      exit(1);
    }
    if (!normalize_position_variables("pedal_decel_vpos", "pedal_decel_pos")) {
      cout << "cannot normalize pedal_decel_pos" << endl;
      exit(1);
    }
    if (!normalize_position_variables("steer_wheel_vpos", "steer_wheel_pos")) {
      cout << "cannot normalize steer_wheel_pos" << endl;
      exit(1);
    }
    if (!normalize_position_variables("gear_stick_vpos", "gear_stick_pos")) {
      cout << "cannot normalize gear_stick_pos" << endl;
      exit(1);
    }
    if (!normalize_position_variables("lidar_front_vpos", "lidar_front_pos")) {
      cout << "cannot normalize lidar_front_pos" << endl;
      exit(1);
    }
    if (!normalize_position_variables("lidar_left_vpos", "lidar_left_pos")) {
      cout << "cannot normalize lidar_left_pos" << endl;
      exit(1);
    }
    if (!normalize_position_variables("lidar_right_vpos", "lidar_right_pos")) {
      cout << "cannot normalize lidar_right_pos" << endl;
      exit(1);
    }
  }
  
  bool IchthusController::serviceController(con_msg::Request &req, con_msg::Response &res) {
    state_var var;
    string cmd = req.cmd;
    string arg1 = req.arg1;
    string arg2 = req.arg2;
    
    if (cmd == "get") {
      if (arg1 == "varlist") {
	if (arg2 == "")
	  res.res = get_state_varlist(); // cmdstring: "get varlist"
	else if (arg2 == "all")
	  res.res = get_state_varlist_all(); // cmdstring: "get varlist all"
      }
      else if (get_state_var(arg1, var)) {
	if (arg2 == "lower")          res.res = "[lower=" + to_string(var.lower) + "]";
	else if (arg2 == "upper")     res.res = "[upper=" + to_string(var.upper) + "]";
	else if (arg2 == "origin")    res.res = "[origin=" + to_string(var.origin) + "]";
	else if (arg2 == "step")      res.res = "[step=" + to_string(var.step) + "]";
	else if (arg2 == "value")     res.res = "[value=" + to_string(var.value) + "]";
	else if (arg2 == "timestamp") res.res = "[timestamp=" + to_string(var.timestamp) + "]";
	else if (arg2 == "property")  res.res = "[property=" + to_string(var.property) + "]";
	else if (arg2 == "")          res.res = var.to_string();
      }
      else
	res.res = "[no such variable: " + arg1 + "]";
    }
    else if (cmd == "set") {
      if (set_state_var(arg1, atof(arg2.c_str()))) {
	get_state_var(arg1, var);
	res.res = var.to_string(); 
      }
      else
	res.res = "[no such variable: " + arg1 + "]";
    }
    return true;
  }

  extern bool receiving_from_can_gateway;
  
  void IchthusController::callback_CAN(const ichthus_can::can_msgConstPtr& msg){
    receiving_from_can_gateway = true;
    if(update_vehicle_velocity)
      set_state_var("vehicle_velocity_can",msg->velocity);
    set_state_var("vehicle_steering_wheel_angle_can",msg->steering_wheel_angle_can);
    //publish_topics();
  }

  // void IchthusController::callback_OBD(const ichthus_controller::obd_msgConstPtr& msg){
  //   receiving_from_can_gateway = false;
  //   if(update_vehicle_velocity)
  //     set_state_var("vehicle_velocity_obd",(double)msg->velocity);
  // }

  void IchthusController::publish_topics(void){
    double accel_apos, decel_apos, steer_apos, accel_pos, decel_pos, steer_pos, target_velocity,current_velocity;
    get_state_var_value("pedal_accel_apos", accel_apos);
    get_state_var_value("pedal_decel_apos", decel_apos);
    get_state_var_value("steer_wheel_apos", steer_apos);
    get_state_var_value("pedal_accel_pos",  accel_pos);
    get_state_var_value("pedal_decel_pos",  decel_pos);
    get_state_var_value("steer_wheel_pos",  steer_pos);
    get_state_var_value("cc_target_velocity", target_velocity);
    get_state_var_value("vehicle_velocity_can", current_velocity);

    std_msgs::Float32 msg;

    msg.data = accel_apos;
    pub_current_accel_pos.publish(msg);

    msg.data = accel_pos; 
    pub_target_accel_pos.publish(msg);

    msg.data = decel_apos;
    pub_current_decel_pos.publish(msg);

    msg.data = decel_pos;
    pub_target_decel_pos.publish(msg);

    // msg.data = steer_apos;
    // pub_current_steer_wheel_pos.publish(msg);

    // msg.data = steer_pos;
    // pub_target_steer_wheel_pos.publish(msg);

    msg.data = target_velocity;
    pub_controller_target_velocity.publish(msg);

    msg.data = current_velocity;
    pub_controller_current_velocity.publish(msg);
  }

  void IchthusController::callback_twist_cmd(const geometry_msgs::TwistStampedConstPtr &msg){
    set_state_var("cc_target_velocity", msg->twist.linear.x * 3.6);
    set_state_var("sc_target_anglvelo", msg->twist.angular.z);

    //publish_topics(); // go to vehicle_status_can (cruise test)
  }

  void IchthusController::callback_motion_state(const std_msgs::Float32ConstPtr& msg){
    set_state_var("motion_state",msg->data);
  }

  void IchthusController::callback_brake_one_step(const std_msgs::BoolConstPtr& msg){
    set_state_var("brake_one_step",msg->data);
  }

  bool mission_complete = false;
  void IchthusController::callback_mission_complete(const std_msgs::BoolConstPtr& msg){
    mission_complete = msg->data;
  }

  void IchthusController::doLoop() {
    state_var var;

    load_state_vars();
    //print_state_vars();
    ros::Subscriber can_sub = nh.subscribe("/vehicle_status_can",1, &IchthusController::callback_CAN, this);
    ros::Subscriber sub_motion_state = nh.subscribe("/motion_state",1,&IchthusController::callback_motion_state,this);
    ros::Subscriber sub_mission_complete = nh.subscribe("/mission_complete",1,&IchthusController::callback_mission_complete,this);
    //ros::Subscriber obd_sub = nh.subscribe("/vehicle_status_obd",100, &IchthusController::callback_OBD, this);
    ros::Subscriber twist_filter_sub = nh.subscribe("/twist_cmd",1, &IchthusController::callback_twist_cmd,this);
    ros::Subscriber sub_brake_step = nh.subscribe("/brake_one_step",1,&IchthusController::callback_brake_one_step,this);
    ros::ServiceServer service = nh.advertiseService("ichthus_controller", &IchthusController::serviceController, this);
    ros::spin();
  }
}

// 
