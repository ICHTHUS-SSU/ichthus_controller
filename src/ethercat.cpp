#include "ichthus_controller/ichthus_controller.h"
#include "ichthus_controller/ecrt.h"

namespace ichthus_controller {

  string motor_id_to_pos_name(int motor_id);
  int get_state_var_timestamp(string name, int& timestamp);
  void update_motor_timestamp(int slave_no);
  bool activate_slave(int slave_no);
  bool deactivate_slave(int slave_no);
  bool disable_slave(int slave_no);
  bool enable_slave(int slave_no);
  
  struct motor_state {
    int t_pos;
    int a_pos;
    unsigned short status;
    int seqno;
    int timestamp;
    bool write_allowed;
  } motor[MAX_MOTORS];
  
  bool domain_state_complete = false;
  pthread_t ecat_cyclic_thread;
  
#define ECAT_CYCLIC_THREAD_PERIOD          5  // in milliseconds
#define ECAT_DOMAIN_STATE_CHECK_PERIOD  1000  // in milliseconds
#define ECAT_DOMAIN_STATE_CHECK_PERIOD_IN_TICKS				\
  ((ECAT_DOMAIN_STATE_CHECK_PERIOD)/(ECAT_CYCLIC_THREAD_PERIOD)) // in 10-millisecond ticks
  
  ///////////////////////////////////////////////////////////
  // ichthus i30 specific constants
  ///////////////////////////////////////////////////////////
  // we need fast reaction for the brake pedal
#define PPM_BRAKE_PEDAL_MOTOR_ACCEL 1000000000 
#define PPM_BRAKE_PEDAL_MOTOR_DECEL 1000000000
#define PPM_BRAKE_PEDAL_MOTOR_SPEED 1428500

  // we need slow reaction for the steering wheel
  // in order to give enough time for the brake pedal to respond
#define PPM_STEER_WHEEL_MOTOR_ACCEL 5000
#define PPM_STEER_WHEEL_MOTOR_DECEL 5000
#define PPM_STEER_WHEEL_MOTOR_SPEED 3571

  // default velocity profile setting for other motors
#define PPM_DEFAULT_MOTOR_ACCEL 150000
#define PPM_DEFAULT_MOTOR_DECEL 150000
#define PPM_DEFAULT_MOTOR_SPEED 3571
  
  ///////////////////////////////////////////////////////////
  // EtherCAT specific constants (EPOS3 70/10 411146 model)
  ///////////////////////////////////////////////////////////
#define MAX_SLAVE_LOOKUPS     100
#define SLAVE_LOOKUP_INTERVAL 100 // in millisecond      
#define MAXON_EPOS3   0x000000fb, 0x64400000

  ///////////////////////////////////////////////////////////////////////////
  // State of the Drive
  // (p15, Section 3.2.1, "EPOS3-EtherCAT-Firmware-Specification-En.pdf")
  ///////////////////////////////////////////////////////////////////////////
#define STATUS_CODE(status)             ((status)&(0xFF))  
#define STATE_NOT_READY_TO_SWITCH_ON    0x0000
#define STATE_SWITCH_ON_DISABLED        0x0040
#define STATE_READY_TO_SWITCH_ON        0x0021
#define STATE_SWITCH_ON                 0x0023
#define STATE_OPERATION_ENABLED         0x0037 
#define STATE_QUICK_STOP_ACTIVE         0x0017 //0x0017 ? 0x0007
#define STATE_FAULT_REACTION_ACTIVE_D   0x000F
#define STATE_FAULT_REACTION_ACTIVE_E   0x001F
#define STATE_FAULT                     0x0008
#define STATE_FAULT_CODE(status)        ((status)&(0x08))  
  
  ///////////////////////////////////////////////////////////////////////////
  // Device Control Commands
  // (p17, Section 3.2.3, "EPOS3-EtherCAT-Firmware-Specification-En.pdf")
  ///////////////////////////////////////////////////////////////////////////
#define COMMAND_SHUTDOWN          0x0006
#define COMMAND_SWITCH_ON         0x0007
#define COMMAND_SWITCH_ON_ENABLE  0x000F
#define COMMAND_DISABLE_VOLTAGE   0x0000
#define COMMAND_QUICK_STOP        0x0002
#define COMMAND_DISABLE_OPERATION 0x0007
#define COMMAND_ENABLE_OPERATION  0x000F  
#define COMMAND_FAULT_RESET       0x0080

  ///////////////////////////////////////////////////////////////////////////
  // Profile Position Mode
  // (p59, Section 6.4, "EPOS3-EtherCAT-Application-Notes-Collection-En.pdf")
  ///////////////////////////////////////////////////////////////////////////
#define COMMAND_PPM_SWITCH_ON_ENABLE  0x000F
#define COMMAND_PPM_ABS_POS_START     0x003F  
  
  ///////////////////////////////////////////////////////////////////////////
  // Communication Errors
  // (p29, Section 4.2, "EPOS3-EtherCAT-Firmware-Specification-En.pdf")
  ///////////////////////////////////////////////////////////////////////////
#define ABORT_CODE(abort)    ((abort)&(0xFFFF0000))
#define SDO_ERROR            0x05040000
#define ACCESS_ERROR         0x06010000
#define OBJECT_ERROR         0x06020000
#define PDO_ERROR            0x06040000
#define HARDWARE_ERROR       0x06060000
#define PARAMETER_ERROR      0x06070000
#define VALUE_ERROR          0x06090000
#define GENERAL_ERROR        0x08000000
#define ECAT_STATE_ERROR     0x0F000000

  ///////////////////////////////////////////////////////////////////////////
  // Error Register
  // (p85, Section 8.2.2, "EPOS3-EtherCAT-Firmware-Specification-En.pdf")
  ///////////////////////////////////////////////////////////////////////////
#define RX_ERROR_REGISTER      0x1001, 0
#define ERROR_CODE(status)     ((status)&(0xFF))
#define MOTION_ERROR           0x80
#define RESERVED_ERROR         0x40 // always 0
#define DEVPROF_SPECIFIC_ERROR 0x20
#define COMMUNICATION_ERROR    0x10
#define TEMPERATURE_ERROR      0x08
#define VOLTAGE_ERROR          0x04
#define CURRENT_ERROR          0x02
#define GENERIC_ERROR          0x01

  ///////////////////////////////////////////////////////////////////////////
  // Receive PDO 5 Mapping
  // (p92, Section 8.2.9, "EPOS3-EtherCAT-Firmware-Specification-En.pdf")
  ///////////////////////////////////////////////////////////////////////////
#define RXPDO_INDEX             0x1604
#define TX_CONTROL_WORD         0x6040, 0 // uint16
#define TX_TARGET_POSITION      0x607A, 0 // int32
#define TX_TARGET_VELOCITY      0x60FF, 0 // int32
#define TX_PROFILE_ACCELERATION 0x6083, 0 // uint32
#define TX_PROFILE_DECELERATION 0x6084, 0 // uint32
#define TX_PROFILE_VELOCITY     0x6081, 0 // uint32
#define TX_MODES_OF_OPERATION   0x6060, 0 // int8
#define TX_DIG_OUT_FUNC_STATE   0x2078, 1 // uint16
  
  ec_pdo_entry_info_t ppm_pdo_rx_entries[] = {
    {TX_CONTROL_WORD, 16},	   // control word
    {TX_TARGET_POSITION, 32},	   // target position
    {TX_TARGET_VELOCITY, 32},	   // target velocity
    {TX_PROFILE_ACCELERATION, 32}, // profile acceleration
    {TX_PROFILE_DECELERATION, 32}, // profile deceleration
    {TX_PROFILE_VELOCITY, 32},	   // profile velocity
    {TX_MODES_OF_OPERATION, 8},	   // modes of operation
    {TX_DIG_OUT_FUNC_STATE, 16},   // digital output functionalities state
  };

  ///////////////////////////////////////////////////////////////////////////
  // Transmit PDO 5 Mapping
  // (p97, Section 8.2.14, "EPOS3-EtherCAT-Firmware-Specification-En.pdf")
  ///////////////////////////////////////////////////////////////////////////
#define TXPDO_INDEX             0x1A04
#define RX_STATUS_WORD          0x6041, 0 // uint16
#define RX_ACTUAL_POSITION      0x6064, 0 // int32
#define RX_ACTUAL_VELOCITY      0x606C, 0 // int32
#define RX_ACTUAL_CURRENT       0x6078, 0 // int16
#define RX_FOLLOWING_ERR_VALUE  0x20F4, 0 // int16
#define RX_MODES_OF_OP_DISPLAY  0x6061, 0 // int8
#define RX_DIG_IN_FUNC_STATE    0x2071, 1 // uint16
#define RX_INTERPOLATION_BUFFER 0x20C4, 1 // <-- must be 1, not 0, different from description of the above document!
  
  ec_pdo_entry_info_t ppm_pdo_tx_entries[] = { 
    {RX_STATUS_WORD, 16},	   // status word
    {RX_ACTUAL_POSITION, 32},	   // position actual value
    {RX_ACTUAL_VELOCITY, 32},	   // velocity actual value
    {RX_ACTUAL_CURRENT, 16},	   // current actual value
    {RX_FOLLOWING_ERR_VALUE, 16},  // following error actual value
    {RX_MODES_OF_OP_DISPLAY, 8},   // modes of operation display
    {RX_DIG_IN_FUNC_STATE, 16},    // digital input functionalities state
    {RX_INTERPOLATION_BUFFER, 16}, // interpolation buffer status
  };

  ec_pdo_info_t ppm_pdo_info[] = {
    {RXPDO_INDEX, 8, ppm_pdo_rx_entries},
    {TXPDO_INDEX, 8, ppm_pdo_tx_entries},
  };

  ec_sync_info_t ppm_sync_info[] = {
    {2, EC_DIR_OUTPUT, 1, ppm_pdo_info + 0},
    {3, EC_DIR_INPUT,  1, ppm_pdo_info + 1},
    {0xff}
  };

  ////////////////////////////////////////////////////
  // Data structures to store offsets for PPM PDOs
  ////////////////////////////////////////////////////
  struct ppm_pdo_offset {
    // offsets for PDOs of Receive PDO 5 Mapping
    unsigned int cntrl; // offset 0 -> 2 bytes
    unsigned int t_pos; // offset 2 -> 4 bytes
    unsigned int t_vel; // offset 6 -> 4 bytes
    unsigned int p_acc; // offset 10 -> 4 bytes
    unsigned int p_dec; // offset 14 -> 4 bytes
    unsigned int p_vel; // offset 18 -> 4 bytes
    unsigned int opmod; // offset 22 -> 1 bytes
    unsigned int dofst; // offset 23 -> 2 bytes
    // offsets for PDOs of Transmit PDO 5 Mapping
    unsigned int state; // offset 25 -> 2 bytes
    unsigned int a_pos; // offset 27 -> 4 bytes
    unsigned int a_vel; // offset 31 -> 4 bytes
    unsigned int a_cur; // offset 35 -> 2 bytes
    unsigned int error; // offset 37 -> 2 bytes
    unsigned int odmod; // offset 39 -> 1 bytes
    unsigned int difst; // offset 40 -> 2 bytes
    unsigned int inbuf; // offset 42
  } offset_of;
  
  /////////////////////////////////////////////////////////
  // Data structures for master, domains, slaves, motors
  /////////////////////////////////////////////////////////
  struct motion_system {
    ec_master_t      *master;
    ec_master_state_t master_state;
    ec_domain_t      *domain[MAX_MOTORS];
    ec_domain_state_t domain_state[MAX_MOTORS];
    uint8_t          *domain_data[MAX_MOTORS];
    ec_slave_config_t      *slave[MAX_MOTORS];
    ec_slave_config_state_t slave_state[MAX_MOTORS];
  } msys;

  ec_pdo_entry_reg_t domain_regs[MAX_MOTORS][17] = {
    { // ec_pdo_entry_reg_t regs_pedal_accel[] = {
      {0, 0, MAXON_EPOS3, TX_CONTROL_WORD, &offset_of.cntrl},
      {0, 0, MAXON_EPOS3, TX_TARGET_POSITION, &offset_of.t_pos},
      {0, 0, MAXON_EPOS3, TX_TARGET_VELOCITY, &offset_of.t_vel},
      {0, 0, MAXON_EPOS3, TX_PROFILE_ACCELERATION, &offset_of.p_acc},
      {0, 0, MAXON_EPOS3, TX_PROFILE_DECELERATION, &offset_of.p_dec},
      {0, 0, MAXON_EPOS3, TX_PROFILE_VELOCITY, &offset_of.p_vel},
      {0, 0, MAXON_EPOS3, TX_MODES_OF_OPERATION, &offset_of.opmod},
      {0, 0, MAXON_EPOS3, TX_DIG_OUT_FUNC_STATE, &offset_of.dofst},

      {0, 0, MAXON_EPOS3, RX_STATUS_WORD, &offset_of.state},
      {0, 0, MAXON_EPOS3, RX_ACTUAL_POSITION, &offset_of.a_pos},
      {0, 0, MAXON_EPOS3, RX_ACTUAL_VELOCITY, &offset_of.a_vel},
      {0, 0, MAXON_EPOS3, RX_ACTUAL_CURRENT, &offset_of.a_cur},
      {0, 0, MAXON_EPOS3, RX_FOLLOWING_ERR_VALUE, &offset_of.error},
      {0, 0, MAXON_EPOS3, RX_MODES_OF_OP_DISPLAY, &offset_of.odmod},
      {0, 0, MAXON_EPOS3, RX_DIG_IN_FUNC_STATE, &offset_of.difst},
      {0, 0, MAXON_EPOS3, RX_INTERPOLATION_BUFFER, &offset_of.inbuf},
      {}
    },

    { // ec_pdo_entry_reg_t regs_pedal_decel[] = {
      {0, 1, MAXON_EPOS3, TX_CONTROL_WORD, &offset_of.cntrl},
      {0, 1, MAXON_EPOS3, TX_TARGET_POSITION, &offset_of.t_pos},
      {0, 1, MAXON_EPOS3, TX_TARGET_VELOCITY, &offset_of.t_vel},
      {0, 1, MAXON_EPOS3, TX_PROFILE_ACCELERATION, &offset_of.p_acc},
      {0, 1, MAXON_EPOS3, TX_PROFILE_DECELERATION, &offset_of.p_dec},
      {0, 1, MAXON_EPOS3, TX_PROFILE_VELOCITY, &offset_of.p_vel},
      {0, 1, MAXON_EPOS3, TX_MODES_OF_OPERATION, &offset_of.opmod},
      {0, 1, MAXON_EPOS3, TX_DIG_OUT_FUNC_STATE, &offset_of.dofst},

      {0, 1, MAXON_EPOS3, RX_STATUS_WORD, &offset_of.state},
      {0, 1, MAXON_EPOS3, RX_ACTUAL_POSITION, &offset_of.a_pos},
      {0, 1, MAXON_EPOS3, RX_ACTUAL_VELOCITY, &offset_of.a_vel},
      {0, 1, MAXON_EPOS3, RX_ACTUAL_CURRENT, &offset_of.a_cur},
      {0, 1, MAXON_EPOS3, RX_FOLLOWING_ERR_VALUE, &offset_of.error},
      {0, 1, MAXON_EPOS3, RX_MODES_OF_OP_DISPLAY, &offset_of.odmod},
      {0, 1, MAXON_EPOS3, RX_DIG_IN_FUNC_STATE, &offset_of.difst},
      {0, 1, MAXON_EPOS3, RX_INTERPOLATION_BUFFER, &offset_of.inbuf},
      {}
    },

    { // ec_pdo_entry_reg_t regs_lidar_front[] = {
      {0, 2, MAXON_EPOS3, TX_CONTROL_WORD, &offset_of.cntrl},
      {0, 2, MAXON_EPOS3, TX_TARGET_POSITION, &offset_of.t_pos},
      {0, 2, MAXON_EPOS3, TX_TARGET_VELOCITY, &offset_of.t_vel},
      {0, 2, MAXON_EPOS3, TX_PROFILE_ACCELERATION, &offset_of.p_acc},
      {0, 2, MAXON_EPOS3, TX_PROFILE_DECELERATION, &offset_of.p_dec},
      {0, 2, MAXON_EPOS3, TX_PROFILE_VELOCITY, &offset_of.p_vel},
      {0, 2, MAXON_EPOS3, TX_MODES_OF_OPERATION, &offset_of.opmod},
      {0, 2, MAXON_EPOS3, TX_DIG_OUT_FUNC_STATE, &offset_of.dofst},

      {0, 2, MAXON_EPOS3, RX_STATUS_WORD, &offset_of.state},
      {0, 2, MAXON_EPOS3, RX_ACTUAL_POSITION, &offset_of.a_pos},
      {0, 2, MAXON_EPOS3, RX_ACTUAL_VELOCITY, &offset_of.a_vel},
      {0, 2, MAXON_EPOS3, RX_ACTUAL_CURRENT, &offset_of.a_cur},
      {0, 2, MAXON_EPOS3, RX_FOLLOWING_ERR_VALUE, &offset_of.error},
      {0, 2, MAXON_EPOS3, RX_MODES_OF_OP_DISPLAY, &offset_of.odmod},
      {0, 2, MAXON_EPOS3, RX_DIG_IN_FUNC_STATE, &offset_of.difst},
      {0, 2, MAXON_EPOS3, RX_INTERPOLATION_BUFFER, &offset_of.inbuf},
      {}
    },

    { // ec_pdo_entry_reg_t regs_lidar_left[] = {
      {0, 3, MAXON_EPOS3, TX_CONTROL_WORD, &offset_of.cntrl},
      {0, 3, MAXON_EPOS3, TX_TARGET_POSITION, &offset_of.t_pos},
      {0, 3, MAXON_EPOS3, TX_TARGET_VELOCITY, &offset_of.t_vel},
      {0, 3, MAXON_EPOS3, TX_PROFILE_ACCELERATION, &offset_of.p_acc},
      {0, 3, MAXON_EPOS3, TX_PROFILE_DECELERATION, &offset_of.p_dec},
      {0, 3, MAXON_EPOS3, TX_PROFILE_VELOCITY, &offset_of.p_vel},
      {0, 3, MAXON_EPOS3, TX_MODES_OF_OPERATION, &offset_of.opmod},
      {0, 3, MAXON_EPOS3, TX_DIG_OUT_FUNC_STATE, &offset_of.dofst},

      {0, 3, MAXON_EPOS3, RX_STATUS_WORD, &offset_of.state},
      {0, 3, MAXON_EPOS3, RX_ACTUAL_POSITION, &offset_of.a_pos},
      {0, 3, MAXON_EPOS3, RX_ACTUAL_VELOCITY, &offset_of.a_vel},
      {0, 3, MAXON_EPOS3, RX_ACTUAL_CURRENT, &offset_of.a_cur},
      {0, 3, MAXON_EPOS3, RX_FOLLOWING_ERR_VALUE, &offset_of.error},
      {0, 3, MAXON_EPOS3, RX_MODES_OF_OP_DISPLAY, &offset_of.odmod},
      {0, 3, MAXON_EPOS3, RX_DIG_IN_FUNC_STATE, &offset_of.difst},
      {0, 3, MAXON_EPOS3, RX_INTERPOLATION_BUFFER, &offset_of.inbuf},
      {}
    },

    { // ec_pdo_entry_reg_t regs_lidar_right[] = {
      {0, 4, MAXON_EPOS3, TX_CONTROL_WORD, &offset_of.cntrl},
      {0, 4, MAXON_EPOS3, TX_TARGET_POSITION, &offset_of.t_pos},
      {0, 4, MAXON_EPOS3, TX_TARGET_VELOCITY, &offset_of.t_vel},
      {0, 4, MAXON_EPOS3, TX_PROFILE_ACCELERATION, &offset_of.p_acc},
      {0, 4, MAXON_EPOS3, TX_PROFILE_DECELERATION, &offset_of.p_dec},
      {0, 4, MAXON_EPOS3, TX_PROFILE_VELOCITY, &offset_of.p_vel},
      {0, 4, MAXON_EPOS3, TX_MODES_OF_OPERATION, &offset_of.opmod},
      {0, 4, MAXON_EPOS3, TX_DIG_OUT_FUNC_STATE, &offset_of.dofst},

      {0, 4, MAXON_EPOS3, RX_STATUS_WORD, &offset_of.state},
      {0, 4, MAXON_EPOS3, RX_ACTUAL_POSITION, &offset_of.a_pos},
      {0, 4, MAXON_EPOS3, RX_ACTUAL_VELOCITY, &offset_of.a_vel},
      {0, 4, MAXON_EPOS3, RX_ACTUAL_CURRENT, &offset_of.a_cur},
      {0, 4, MAXON_EPOS3, RX_FOLLOWING_ERR_VALUE, &offset_of.error},
      {0, 4, MAXON_EPOS3, RX_MODES_OF_OP_DISPLAY, &offset_of.odmod},
      {0, 4, MAXON_EPOS3, RX_DIG_IN_FUNC_STATE, &offset_of.difst},
      {0, 4, MAXON_EPOS3, RX_INTERPOLATION_BUFFER, &offset_of.inbuf},
      {}
    },

    { // ec_pdo_entry_reg_t regs_steer_wheel[] = {
      {0, 5, MAXON_EPOS3, TX_CONTROL_WORD, &offset_of.cntrl},
      {0, 5, MAXON_EPOS3, TX_TARGET_POSITION, &offset_of.t_pos},
      {0, 5, MAXON_EPOS3, TX_TARGET_VELOCITY, &offset_of.t_vel},
      {0, 5, MAXON_EPOS3, TX_PROFILE_ACCELERATION, &offset_of.p_acc},
      {0, 5, MAXON_EPOS3, TX_PROFILE_DECELERATION, &offset_of.p_dec},
      {0, 5, MAXON_EPOS3, TX_PROFILE_VELOCITY, &offset_of.p_vel},
      {0, 5, MAXON_EPOS3, TX_MODES_OF_OPERATION, &offset_of.opmod},
      {0, 5, MAXON_EPOS3, TX_DIG_OUT_FUNC_STATE, &offset_of.dofst},

      {0, 5, MAXON_EPOS3, RX_STATUS_WORD, &offset_of.state},
      {0, 5, MAXON_EPOS3, RX_ACTUAL_POSITION, &offset_of.a_pos},
      {0, 5, MAXON_EPOS3, RX_ACTUAL_VELOCITY, &offset_of.a_vel},
      {0, 5, MAXON_EPOS3, RX_ACTUAL_CURRENT, &offset_of.a_cur},
      {0, 5, MAXON_EPOS3, RX_FOLLOWING_ERR_VALUE, &offset_of.error},
      {0, 5, MAXON_EPOS3, RX_MODES_OF_OP_DISPLAY, &offset_of.odmod},
      {0, 5, MAXON_EPOS3, RX_DIG_IN_FUNC_STATE, &offset_of.difst},
      {0, 5, MAXON_EPOS3, RX_INTERPOLATION_BUFFER, &offset_of.inbuf},
      {}
    }, 

    { // ec_pdo_entry_reg_t regs_gear_stick[] = {
      {0, 6, MAXON_EPOS3, TX_CONTROL_WORD, &offset_of.cntrl},
      {0, 6, MAXON_EPOS3, TX_TARGET_POSITION, &offset_of.t_pos},
      {0, 6, MAXON_EPOS3, TX_TARGET_VELOCITY, &offset_of.t_vel},
      {0, 6, MAXON_EPOS3, TX_PROFILE_ACCELERATION, &offset_of.p_acc},
      {0, 6, MAXON_EPOS3, TX_PROFILE_DECELERATION, &offset_of.p_dec},
      {0, 6, MAXON_EPOS3, TX_PROFILE_VELOCITY, &offset_of.p_vel},
      {0, 6, MAXON_EPOS3, TX_MODES_OF_OPERATION, &offset_of.opmod},
      {0, 6, MAXON_EPOS3, TX_DIG_OUT_FUNC_STATE, &offset_of.dofst},

      {0, 6, MAXON_EPOS3, RX_STATUS_WORD, &offset_of.state},
      {0, 6, MAXON_EPOS3, RX_ACTUAL_POSITION, &offset_of.a_pos},
      {0, 6, MAXON_EPOS3, RX_ACTUAL_VELOCITY, &offset_of.a_vel},
      {0, 6, MAXON_EPOS3, RX_ACTUAL_CURRENT, &offset_of.a_cur},
      {0, 6, MAXON_EPOS3, RX_FOLLOWING_ERR_VALUE, &offset_of.error},
      {0, 6, MAXON_EPOS3, RX_MODES_OF_OP_DISPLAY, &offset_of.odmod},
      {0, 6, MAXON_EPOS3, RX_DIG_IN_FUNC_STATE, &offset_of.difst},
      {0, 6, MAXON_EPOS3, RX_INTERPOLATION_BUFFER, &offset_of.inbuf},
      {}
    }
  };

  ////////////////////////////////////////////////////
  // ichthus i30 specific functions
  ////////////////////////////////////////////////////
  void init_domain_regs(void) {
    int num_regs = sizeof(domain_regs[0])/sizeof(ec_pdo_entry_reg_t);
    cout << "ECAT: num_regs per domain = " << num_regs << endl;

    // initialize the slave positions of the domain registers
    for (int i = 0; i < num_motors; i++)
      for (int j = 0; j < num_regs - 1; j++)
	domain_regs[i][j].position = (uint16_t) i;
  }

  void init_motor_state(void) {
    for (int i = 0; i < num_motors; i++) {
      motor[i].t_pos = 0;
      motor[i].a_pos = 0;
      motor[i].status = 0;
      motor[i].seqno = 3;
      motor[i].timestamp = -1;
      motor[i].write_allowed = false;
    }
  }

  void write_motor_speed(int slave, int vel, int accel, int decel)  {
    EC_WRITE_U32(msys.domain_data[slave] + offset_of.p_vel, vel);
    EC_WRITE_U32(msys.domain_data[slave] + offset_of.p_acc, accel);
    EC_WRITE_U32(msys.domain_data[slave] + offset_of.p_dec, decel);
  }

  void init_velocity_profiles(void) {
    cout << "ECAT: set_velocity_profiles called... " << endl;
    for (int i = 0; i < num_motors; i++){
      if (i == motor_id_pedal_decel) 
        write_motor_speed(i, PPM_BRAKE_PEDAL_MOTOR_SPEED, PPM_BRAKE_PEDAL_MOTOR_ACCEL, PPM_BRAKE_PEDAL_MOTOR_DECEL);
      else if (i == motor_id_steer_wheel) 
        write_motor_speed(i, PPM_STEER_WHEEL_MOTOR_SPEED, PPM_STEER_WHEEL_MOTOR_ACCEL, PPM_STEER_WHEEL_MOTOR_DECEL);
      else 
        write_motor_speed(i, PPM_DEFAULT_MOTOR_SPEED, PPM_DEFAULT_MOTOR_ACCEL, PPM_DEFAULT_MOTOR_DECEL);
    }
  }


  ////////////////////////////////////////////////////
  // EtherCAT and PPM specific functions
  ////////////////////////////////////////////////////
  
  void print_error_code(uint8_t error) { // device error
    string msg;
    switch (ERROR_CODE(error)){
      case MOTION_ERROR:           msg = "ECAT: motion error"; break;
      case DEVPROF_SPECIFIC_ERROR: msg = "ECAT: device profile-specific error"; break;
      case COMMUNICATION_ERROR:    msg = "ECAT: communication error"; break; 
      case TEMPERATURE_ERROR:      msg = "ECAT: temperature error"; break;
      case VOLTAGE_ERROR:          msg = "ECAT: voltage error"; break;
      case CURRENT_ERROR:          msg = "ECAT: current error"; break; 
      case GENERIC_ERROR:          msg = "ECAT: generic error"; break;
      default:                     msg = "ECAT: unknown error"; break;
    }
    cout << msg << endl;
  }

  void print_abort_code(uint32_t abort) { // communication error
    string msg;
    switch(ABORT_CODE(abort)){
      case SDO_ERROR:        msg = "ECAT: abort due to SDO error"; break;
      case ACCESS_ERROR:     msg = "ECAT: abort due to Access error"; break;
      case OBJECT_ERROR:     msg = "ECAT: abort due to Wrong Object error"; break;
      case PDO_ERROR:        msg = "ECAT: abort due to PDO error"; break;
      case HARDWARE_ERROR:   msg = "ECAT: abort due to Hardware error"; break;
      case PARAMETER_ERROR:  msg = "ECAT: abort due to Service Parameter error"; break;
      case VALUE_ERROR:      msg = "ECAT: abort due to Value error"; break;
      case GENERAL_ERROR:    msg = "ECAT: abort due to Gerneral error"; break;
      case ECAT_STATE_ERROR: msg = "ECAT: abort due to EtherCAT state error"; break;
      default:               msg = "ECAT: unknown abort code";  break;
    }
    cout << msg <<endl;
  }

  unsigned short read_slave_status(int slave_no) {
    uint16_t status;
    status = EC_READ_U16(msys.domain_data[slave_no] + offset_of.state);
    status = STATUS_CODE(status);
    motor[slave_no].status = status;
    return status;
  }

  void print_slave_status(uint16_t status) {
    switch(STATUS_CODE(status)){
      case STATE_NOT_READY_TO_SWITCH_ON:
	cout << "not ready to switch on" << endl;
	break;     
      case STATE_SWITCH_ON_DISABLED:
	cout << "switch on disabled" << endl;
	break;
      case STATE_READY_TO_SWITCH_ON:
	cout << "ready to switch on" << endl;
	break;
      case STATE_SWITCH_ON:
	cout << "switch on" << endl;
	break;
      case STATE_OPERATION_ENABLED:
	cout << "operation enabled" << endl;
	break;
      case STATE_QUICK_STOP_ACTIVE:
	cout << "quick stop active" << endl;
	break;
      case STATE_FAULT_REACTION_ACTIVE_D:
	cout << "fault reaction active d" << endl;
	break;
      case STATE_FAULT_REACTION_ACTIVE_E:
	cout << "fault reaction active e" << endl;
	break;
      case STATE_FAULT:
	cout << "fault" << endl;
	break;
      default:
	cout << "unknown status" << endl;
	break;
    }
  }

  void print_pdo_offsets(void) {
    cout << "offset_of.cntrl = " << offset_of.cntrl << endl;
    cout << "offset_of.t_pos = " << offset_of.t_pos << endl;
    cout << "offset_of.t_vel = " << offset_of.t_vel << endl;
    cout << "offset_of.p_acc = " << offset_of.p_acc << endl;
    cout << "offset_of.p_dec = " << offset_of.p_dec << endl;
    cout << "offset_of.p_vel = " << offset_of.p_vel << endl;    
    cout << "offset_of.opmod = " << offset_of.opmod << endl;
    cout << "offset_of.dofst = " << offset_of.dofst << endl;        

    cout << "offset_of.state = " << offset_of.state << endl;
    cout << "offset_of.a_pos = " << offset_of.a_pos << endl;        
    cout << "offset_of.a_vel = " << offset_of.a_vel << endl;
    cout << "offset_of.a_cur = " << offset_of.a_cur << endl;            
    cout << "offset_of.error = " << offset_of.error << endl;
    cout << "offset_of.odmod = " << offset_of.odmod << endl;
    cout << "offset_of.difst = " << offset_of.difst << endl;
    cout << "offset_of.inbuf = " << offset_of.inbuf << endl;            
  }
  
  void print_pdo_values(int slave_no) {

    cout << "slave(" << slave_no << ").cntrl = " << hex << EC_READ_U16(msys.domain_data[slave_no] + offset_of.cntrl) << endl;
    cout << "slave(" << slave_no << ").t_pos = " << dec << EC_READ_S32(msys.domain_data[slave_no] + offset_of.t_pos) << endl;
    cout << "slave(" << slave_no << ").t_vel = " << dec << EC_READ_S32(msys.domain_data[slave_no] + offset_of.t_vel) << endl;
    cout << "slave(" << slave_no << ").p_acc = " << dec << EC_READ_U32(msys.domain_data[slave_no] + offset_of.p_acc) << endl;
    cout << "slave(" << slave_no << ").p_dec = " << dec << EC_READ_U32(msys.domain_data[slave_no] + offset_of.p_dec) << endl;
    cout << "slave(" << slave_no << ").p_vel = " << dec << EC_READ_U32(msys.domain_data[slave_no] + offset_of.p_vel) << endl;    
    cout << "slave(" << slave_no << ").opmod = " << hex << EC_READ_S8(msys.domain_data[slave_no] + offset_of.opmod) << endl;
    cout << "slave(" << slave_no << ").dofst = " << dec << EC_READ_U16(msys.domain_data[slave_no] + offset_of.dofst) << endl;        

    cout << "slave(" << slave_no << ").state = " << hex << EC_READ_U16(msys.domain_data[slave_no] + offset_of.state) << endl;
    cout << "slave(" << slave_no << ").a_pos = " << dec << EC_READ_S32(msys.domain_data[slave_no] + offset_of.a_pos) << endl;        
    cout << "slave(" << slave_no << ").a_vel = " << dec << EC_READ_S32(msys.domain_data[slave_no] + offset_of.a_vel) << endl;
    cout << "slave(" << slave_no << ").a_cur = " << dec << EC_READ_S16(msys.domain_data[slave_no] + offset_of.a_cur) << endl;            
    cout << "slave(" << slave_no << ").error = " << dec << EC_READ_S16(msys.domain_data[slave_no] + offset_of.error) << endl;
    cout << "slave(" << slave_no << ").odmod = " << hex << EC_READ_S8(msys.domain_data[slave_no] + offset_of.odmod) << endl;
    cout << "slave(" << slave_no << ").difst = " << dec << EC_READ_U16(msys.domain_data[slave_no] + offset_of.difst) << endl;
    cout << "slave(" << slave_no << ").inbuf = " << hex << EC_READ_S32(msys.domain_data[slave_no] + offset_of.inbuf) << dec << endl;            
  }
  
  bool upload_sdo_to_slave(uint16_t pos, uint16_t index, uint8_t subindex, uint8_t *target, size_t target_size)  {
    size_t result_size = 0;
    uint32_t abort_code = 0;

    if (ecrt_master_sdo_upload(msys.master, pos, index, subindex, target, target_size, &result_size, &abort_code) < 0) {
      print_abort_code(abort_code);
      cout << "ECAT: upload_sdo_from_slave failed, please check slave!!!" << endl;
      return false;
    }

    return true;
  }

  bool download_sdo_from_slave(uint16_t pos, uint16_t index, uint8_t subindex, uint8_t *data, size_t data_size)  {
    uint32_t abort_code = 0;

    if (ecrt_master_sdo_download(msys.master, pos, index, subindex, data, data_size, &abort_code) < 0) {
      print_abort_code(abort_code);
      cout << "ECAT: download_sdo_from_slave failed, please check slave!!!" << endl;
      return false;
    }

    return true;
  }

  unsigned short read_slave_status_by_sdo(unsigned int slave_no)  {
    uint16_t status = 0xFFFF;
    uint8_t error;

    if (!upload_sdo_to_slave(slave_no, RX_STATUS_WORD, (uint8_t *) &status, sizeof(status)))
      return status; 

    status = STATUS_CODE(status);

    cout << "ECAT: slave(" << slave_no << ") ";
    print_slave_status(status);

    if (STATE_FAULT_CODE(status)) {
      if (upload_sdo_to_slave(slave_no, RX_ERROR_REGISTER, (uint8_t *) &error, sizeof(error)))
        print_error_code(error);
    }

    motor[slave_no].status = status;
    return status;
  }

  void write_target_pos(int slave_no) {

   // cout << "ECAT: write_target_pos(" << slave_no << ", " << motor[slave_no].t_pos << ") called..." << endl;
    
    if (motor[slave_no].seqno == 3) {
      EC_WRITE_U16(msys.domain_data[slave_no] + offset_of.cntrl, COMMAND_PPM_SWITCH_ON_ENABLE);
      motor[slave_no].seqno--;
    }
    else if (motor[slave_no].seqno == 2) {
      EC_WRITE_S32(msys.domain_data[slave_no] + offset_of.t_pos, motor[slave_no].t_pos);
      motor[slave_no].seqno--;
    }
    else if (motor[slave_no].seqno == 1) {
      EC_WRITE_U16(msys.domain_data[slave_no] + offset_of.cntrl, COMMAND_PPM_ABS_POS_START);
      motor[slave_no].seqno = 3;
      update_motor_timestamp(slave_no); // DO NOT FORGET TO UPDATE MOTOR TIMESTAMP!!!
    }
    else{
      cout <<"ECAT: motor[" << slave_no <<  "].seqno error" << endl;
      return;
    }
  }

  int read_actual_pos(int slave_no) {
    motor[slave_no].a_pos = EC_READ_S32(msys.domain_data[slave_no] + offset_of.a_pos);
    return motor[slave_no].a_pos;
  }

  bool init_master_and_slaves(void) {
    int i;
    int slave_count;
    ec_master_info_t master_info;
  
    cout << "ECAT: ecrt_request_master called..." << endl;
    if (!(msys.master = ecrt_request_master(0))){
      cout << "ECAT: ecrt_request_master failed!!!" << endl;
      return false; 
    } 

    ecrt_master(msys.master,&master_info);
    slave_count = master_info.slave_count;
    cout << "ECAT: num_slaves = " << slave_count << ", num_motors = " << num_motors << endl;
    if (slave_count < num_motors){
      cout << "ECAT: some slaves are missing!!!" << endl;
      return false;
    }

    for (i = 0; i < num_motors; i++) {
      cout << "ECAT: ecrt_master_create_domain(" << i << ") called..." << endl;
      if (!(msys.domain[i] = ecrt_master_create_domain(msys.master))){
        cout << "ECAT: ecrt_master_create_domain failed!!!" << endl;
        return false;
      }
    }

    for (i = 0; i < num_motors; i++) {
      cout << "ECAT: ecrt_master_slave_config(" << i << ") called..." << endl;
      if (!(msys.slave[i] = ecrt_master_slave_config(msys.master, 0, i, MAXON_EPOS3))){
        cout << "ECAT: ecrt_master_slave_config failed!!!" << endl;
        return false;
      }
    }

    for (i = 0; i < num_motors; i++){
      if(!deactivate_slave(i)) {
	cout << "ECAT: deactivate_slave failed!!!" << endl;
	return false;
      }
      if(!activate_slave(i)) {
	cout << "ECAT: activate_slave failed!!!" << endl;
	return false;
      }
    }
    
    return true;
  }

  bool init_pdo_mapping_and_clocks(void) {
    int i;

    for (i = 0; i < num_motors; i++) {
      cout << "ECAT: ecrt_slave_config_pdos(" << i << ") called..." << endl;
      if (ecrt_slave_config_pdos(msys.slave[i], EC_END, ppm_sync_info) != 0) {
        cout << "ecrt_slave_config_pdos failed!!!" << endl;
        return false;
      }
    }

    for (i = 0; i < num_motors; i++) {
      cout << "ECAT: ecrt_domain_reg_pdo_entry_list(" << i << ") called... " << endl;
      if (ecrt_domain_reg_pdo_entry_list(msys.domain[i], domain_regs[i]) != 0) {
        cout << "ecrt_domain_reg_pdo_entry_list failed!!!" << endl;
        return false;
      }
    }

    for (i = 0; i < num_motors; i++) {
      cout << "ECAT: ecrt_slave_config_dc(" << i << ") called... " << endl;
      ecrt_slave_config_dc(msys.slave[i], 0x0300, 10000000, 4400000, 0, 0);
    }
    
    return true;
  }

  bool activate_master(void) {
    int i;
  
    cout << "ECAT: ecrt_master_activate called... " << endl;
    if (ecrt_master_activate(msys.master))
      return false;

    for (i = 0; i < num_motors; i++) {
      cout << "ECAT: ecrt_domain_data(" << i << ") called... " << endl;
      if (!(msys.domain_data[i] = ecrt_domain_data(msys.domain[i]))) {
        ecrt_master_deactivate(msys.master);
        return false; 
      }
    }
    return true;
  }

  void update_master_state(void) {
    ec_master_state_t ms;
    
    ecrt_master_state(msys.master, &ms);
    if (ms.slaves_responding != msys.master_state.slaves_responding)
      cout << "ECAT: master: " << ms.slaves_responding << " slaves responding" << endl;

    if (ms.al_states != msys.master_state.al_states)
      cout << "ECAT: master: al_state = " << hex << ms.al_states << dec << endl;

    if (ms.link_up != msys.master_state.link_up)
      cout << "ECAT: master: link is " << (ms.link_up ? "up" : "down") << endl;

    msys.master_state = ms; 
  }

  void update_slaves_state(void) {
    for (int i = 0; i < num_motors; i++) {
      ec_slave_config_state_t scs;
      
      ecrt_slave_config_state(msys.slave[i], &scs);
      if (scs.al_state != msys.slave_state[i].al_state)
	cout << "ECAT: slave(" << i << ") al_state = " << hex << scs.al_state << dec << endl;

      if (scs.online != msys.slave_state[i].online)
	cout << "ECAT: slave(" << i << ") " << (scs.online ? "online" : "offline") << endl;

      if (scs.operational != msys.slave_state[i].operational)
	cout << "ECAT: slave(" << i << ") " << (scs.operational ? "operational" : "not operational") << endl;

      msys.slave_state[i] = scs;
    }
  }

  bool update_domain_state(void) {
    for (int i = 0; i < num_motors; i++) {
      ec_domain_state_t ds;
      
      ecrt_domain_state(msys.domain[i], &ds);

      if (ds.working_counter != msys.domain_state[i].working_counter)
	cout << "ECAT: domain(" << i << ") working_counter = " << ds.working_counter << endl;

      if (ds.wc_state != msys.domain_state[i].wc_state)
	cout << "ECAT: domain(" << i << ") wc_state = " << ds.wc_state << endl;

      msys.domain_state[i] = ds;

      if (msys.domain_state[i].wc_state != EC_WC_COMPLETE) // incomplete state
        return false;
    }
    return true; // complete state
  }

  void update_motor_timestamp(int slave_no) {
    get_state_var_timestamp(motor_id_to_pos_name(slave_no), motor[slave_no].timestamp);
  }

  bool position_timestamp_updated(int slave_no) {
    int timestamp = -1;

    if(get_state_var_timestamp(motor_id_to_pos_name(slave_no), timestamp)
       && timestamp != motor[slave_no].timestamp)
      return true;
    
    return false;
  }
  
  void ecat_cyclic_pre_handler(void) {
    static int counter = 0;
    uint16_t status;
    int i;

    ecrt_master_receive(msys.master);
    for (i = 0; i < num_motors; i++) 
      ecrt_domain_process(msys.domain[i]);

    if (counter) 
      counter--;
    else{
      counter = ECAT_DOMAIN_STATE_CHECK_PERIOD_IN_TICKS;

      if(!update_domain_state()){ // update_domain_state should be called at 1 Hz
        domain_state_complete = false;
        for(i = 0; i < num_motors; i++) {
	  status = read_slave_status(i);
	  cout << "ECAT: slave(" << i << ") ";    
	  print_slave_status(status);
	}
        cout << endl;
      }
      else{
        domain_state_complete = true;
        update_slaves_state();
        update_master_state();
      }
    }
  }
  
  void ecat_cyclic_post_handler(void) {
    struct timespec time;
    static int sync_ref_counter = 0;

    // this command should be used for sync dc    
    clock_gettime(CLOCK_REALTIME, &time);

    #define NSEC_PER_SEC (1000000000L)
    #define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
    // sync master with current time
    ecrt_master_application_time(msys.master, TIMESPEC2NS(time));

    if (sync_ref_counter)
      sync_ref_counter--;
    else {
      sync_ref_counter = 1; // sync every other cycle
      ecrt_master_sync_reference_clock(msys.master);
    }

    ecrt_master_sync_slave_clocks(msys.master);
    
    for (int i = 0; i < num_motors; i++)
      ecrt_domain_queue(msys.domain[i]);  

    ecrt_master_send(msys.master); 
  }

  void* ecat_cyclic_loop(void *arg) {
    extern int init_timer(int usec);
    extern int wait_timer(int timerfd);
    extern void run_controller(void);

    //int timerfd = init_timer(1000 * ECAT_CYCLIC_THREAD_PERIOD); // in microseconds
    int timerfd = init_timer(1000 * 10);

#if 0    
    // we must wait until domain_state_complete becomes true,
    // otherwise ecat_cyclic_pre_handler will never be called in the below loop!!!
    while (!domain_state_complete) {
      ecat_cyclic_pre_handler();
      ecat_cyclic_post_handler();
      wait_timer(timerfd);
    }
#endif
    
    while (true) {
      ecat_cyclic_pre_handler();

      // for (int i = 0; i < num_motors; i++) {
	//ecat_cyclic_pre_handler();

	if(domain_state_complete) {
	  // run controller that generates a sequence of positions
	  run_controller();

    for (int i = 0; i < num_motors; i++) {
	    if (motor[i].write_allowed && position_timestamp_updated(i))
	      write_target_pos(i);
	  //read_actual_pos(i);
	  }


	//ecat_cyclic_post_handler();
	//wait_timer(timerfd);
  }

      ecat_cyclic_post_handler();
      wait_timer(timerfd);
    }
    
  }

  ////////////////////////////////////////////////////
  // Exported functions
  ////////////////////////////////////////////////////

  bool set_target_pos(int slave_no, int value) {
    if (!VALID_MOTOR_ID(slave_no)) return false;
    
    motor[slave_no].t_pos = value;
    return true;
  }

  bool get_actual_pos(int slave_no, int& value)  {
    if (!VALID_MOTOR_ID(slave_no)) return false;
    
    value = read_actual_pos(slave_no); // motor[slave_no].a_pos updated!
    return true;
  }

  ///////////////////////////////////////////////////////////////////////////  
  // Summary of Slave State Machine
  //
  // State Definitions:
  // NSO = Not ready to Switch On --> auto-transitioned to SOD
  // SOD = Switch On Disabled
  // RSO = Ready to Switch On
  // SON = Switched On
  // OEN = Operation Enabled
  // QSA = Quick Stop Active
  // FLT = Fault
  //
  // Command Definitions:
  // DVO = Disable Voltage
  // RST = Fault Reset
  // SHD = Shut Down
  // SON = Switch On
  // ENO = Enable Operation
  // DSO = Disable Operation
  // QST = Quick Stop
  // XXX = Not Defined
  //
  // State Transition Matrix:
  // -------------------------------
  //     | SOD RSO SON OEN QSA FLT |
  // -------------------------------
  // SOD | XXX SHD XXX XXX XXX XXX |
  // RSO | DVO XXX SON XXX XXX XXX |
  // SON | DVO SHD XXX ENO XXX XXX |
  // OEN | DVO SHD DSO XXX QST XXX |
  // QSA | DVO XXX XXX ENO XXX XXX |
  // FLT | RST XXX XXX XXX XXX XXX |
  // -------------------------------
  //
  // Function Descriptions:
  // activate_slave()   : NSO -> SOD -> RSO -> SON
  // enable_slave()     : SON -> OEN (this is achieved by write_target_pos())
  // disable_slave()    : OEN -> SON
  // deactivate_slave() : SON -> SOD
  // ecat_up()   : master up,  all slaves(=motors) activated
  // ecat_on()   : thread on,  all slaves(=motors) deactivated, activated and enabled
  // ecat_off()  : thread off, all slaves(=motors) disabled, i.e., activated
  // ecat_down() : master down
  ///////////////////////////////////////////////////////////////////////////
  
  bool activate_slave(int slave_no) { 
    #define SDO_MAX_FOLLOWING_ERROR 0x6065, 0x00 //20200121 MAX FOLLOWING ERROR
    uint16_t control;
    uint16_t status;
    //uint8_t opmode = 1;
    int trials = 0;
    unsigned long maxFlowingError = 2600000;

    cout << "ECAT: activate_slave(" << slave_no << ") called..." << endl;
    
    if (!download_sdo_from_slave(slave_no, SDO_MAX_FOLLOWING_ERROR, (unsigned char *)&maxFlowingError, sizeof(maxFlowingError)))
			return false;
    //20200121 MAX FOLLOWING ERROR
    
    //if (!download_sdo_from_slave(slave_no, TX_MODES_OF_OPERATION, (uint8_t *) &opmode, sizeof(opmode)))
    //  return false;

    while ((status = read_slave_status_by_sdo(slave_no)) != STATE_SWITCH_ON) {
      if (STATE_FAULT_CODE(status))
	control = COMMAND_FAULT_RESET; // --> STATE_SWITCH_ON_DISABLED
      else if (status == STATE_SWITCH_ON_DISABLED)
	control = COMMAND_SHUTDOWN;    // --> STATE_READY_TO_SWITCH_ON
      else if (status == STATE_READY_TO_SWITCH_ON)
        control = COMMAND_SWITCH_ON;   // --> STATE_SWITCH_ON
      //else if (status == STATE_SWITCH_ON) // The loop condition prevents this case
      //  control = COMMAND_ENABLE_OPERATION; // --> STATE_OPERATION_ENABLED
      else if (status == STATE_OPERATION_ENABLED) 
	return false; // this function is assumed not to be called in this case
      else if (status == STATE_QUICK_STOP_ACTIVE)
	return false; // this function is assumed not to be called in this case

      if (trials > MAX_SLAVE_LOOKUPS) {
	cout << "ECAT: cannot reach STATE_SWITCH_ON!!!" << endl;
	return false;
      }

      if (!download_sdo_from_slave(slave_no, TX_CONTROL_WORD, (uint8_t *) &control, sizeof(control)))
	return false;

      usleep(1000*SLAVE_LOOKUP_INTERVAL);
      trials++;
    }
    return true;
  }

  bool deactivate_slave(int slave_no) { 
    uint16_t control;
    uint16_t status;
    int trials = 0;

    cout << "ECAT: deactivate_slave(" << slave_no << ") called..." << endl;
    
    while ((status = read_slave_status_by_sdo(slave_no)) != STATE_SWITCH_ON_DISABLED) {
      if (STATE_FAULT_CODE(status))
	control = COMMAND_FAULT_RESET;      // --> STATE_SWITCH_ON_DISABLED
      //else if (status == STATE_SWITCH_ON_DISABLED) // The loop condition prevents this case
      //  control = COMMAND_SHUTDOWN;       // --> STATE_READY_TO_SWITCH_ON
      else if (status == STATE_READY_TO_SWITCH_ON)
	control = COMMAND_DISABLE_VOLTAGE;  // --> STATE_SWITCH_ON_DISABLED
      else if (status == STATE_SWITCH_ON)
	control = COMMAND_DISABLE_VOLTAGE;  // --> STATE_SWITCH_ON_DISABLED
      else if (status == STATE_OPERATION_ENABLED)
	control = COMMAND_DISABLE_VOLTAGE;  // --> STATE_SWITCH_ON_DISABLED
      else if (status == STATE_QUICK_STOP_ACTIVE)
	control = COMMAND_DISABLE_VOLTAGE;  // --> STATE_SWITCH_ON_DISABLED

      if (trials > MAX_SLAVE_LOOKUPS) {
	cout << "ECAT: cannot reach STATE_SWITCH_ON_DISABLED!!!" << endl;
	return false;
      }

      if (!download_sdo_from_slave(slave_no, TX_CONTROL_WORD, (uint8_t *) &control, sizeof(control)))
	return false;

      usleep(1000*SLAVE_LOOKUP_INTERVAL);
      trials++;
    }
    return true;
  }

  bool disable_slave(int slave_no) {
    uint16_t control;
    uint16_t status;
    int trials = 0;
    
    cout << "ECAT: disable_slave(" << slave_no << ") called..." << endl;
    motor[slave_no].write_allowed = false;

    while ((status = read_slave_status_by_sdo(slave_no)) != STATE_SWITCH_ON) {
      if (STATE_FAULT_CODE(status))
	return false; // this function is assumed not to be called in this case
      else if (status == STATE_SWITCH_ON_DISABLED)
	return false; // this function is assumed not to be called in this case
      else if (status == STATE_READY_TO_SWITCH_ON)
	return false; // this function is assumed not to be called in this case
      //else if (status == STATE_SWITCH_ON) // The loop condition prevents this case
      //  return false;
      else if (status == STATE_OPERATION_ENABLED)
	control = COMMAND_DISABLE_OPERATION;  // --> STATE_SWITCH_ON
      else if (status == STATE_QUICK_STOP_ACTIVE)
	return false; // this function is assumed not to be called in this case

      if (trials > MAX_SLAVE_LOOKUPS) {
	cout << "ECAT: cannot reach STATE_SWITCH_ON!!!" << endl;
	return false;
      }

      if (!download_sdo_from_slave(slave_no, TX_CONTROL_WORD, (uint8_t *) &control, sizeof(control)))
	return false;

      usleep(1000*SLAVE_LOOKUP_INTERVAL);
      trials++;
    }
    return true;
  }

  bool enable_slave(int slave_no) {
    cout << "ECAT: enable_slave(" << slave_no << ") called..." << endl;
    motor[slave_no].write_allowed = true;
    return true;
  }
  
  bool ecat_up(void) {
    init_motor_state();
    init_domain_regs();

  stage1:
    if (!init_master_and_slaves()) { // stage 1
      cout << "ECAT: init_master_and_slaves failed, so you must relaunch the process" << endl;  
      return false;
    }

  stage2:
    if (!init_pdo_mapping_and_clocks()) { // stage 2
      ecrt_release_master(msys.master);
      goto stage1;
    }

    if(!activate_master()) // stage 3
      goto stage2;

    init_velocity_profiles();

    print_pdo_offsets();
    print_pdo_values(0);
    cout << "ECAT: ecat_up done!!!" << endl;

    //ecat_on();
    return true;
  }

  bool ecat_on(void) {
    if (pthread_create(&ecat_cyclic_thread, NULL, ecat_cyclic_loop, (void *)NULL) < 0) {
      cout << "ECAT: pthread_create error" << endl;
      return false;
    }
    
    pthread_detach(ecat_cyclic_thread);

    while (!domain_state_complete) // waiting for domain_state_complete to be true
      usleep(100 * 1000); 

    for (int i = 0; i < num_motors; i++) {
      if(!deactivate_slave(i)) {
	cout << "ECAT: deactivate_slave failed!!!" << endl;
	return false;
      }
      if(!activate_slave(i)) {
	cout << "ECAT: activate_slave failed!!!" << endl;
	return false;
      }
      if(!enable_slave(i)) {
	cout << "ECAT: deactivate_slave failed!!!" << endl;
	return false;
      }
    }

    cout << "ECAT: ecat_on done!!!" << endl;
    return true;
  }
  
  bool ecat_off(void) {

    for (int i = 0; i < num_motors; i++) {
      if(!disable_slave(i)) {
	cout << "ECAT: disable_slave failed!!!" << endl;
	return false;
      }
    }

    domain_state_complete = false;
    pthread_cancel(ecat_cyclic_thread); // kill ecat_cyclic_thread

    set_state_var("motion.state", 0); // disable all motions
    
    cout << "ECAT: ecat_off done!!!" << endl;

    return true;
  }

  bool ecat_down(void) {
    //ecat_off();
    
    cout << "ECAT: ecrt_master_deactivate called..." << endl;
    ecrt_master_deactivate(msys.master);

    cout << "ECAT: ecrt_release_master called..." << endl;
    ecrt_release_master(msys.master);

    // set_state_var("motion.state", 0);
    cout << "ECAT: ecat_down done!!!" << endl;

    return true;
  }
}
