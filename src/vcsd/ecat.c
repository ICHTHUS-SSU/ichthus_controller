#include "ecrt.h"

#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <errno.h>
#include <sys/resource.h>
#include <time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <sys/times.h>
#include <signal.h>
#include <malloc.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <stdio.h>
#include <sys/timerfd.h>
#include <pthread.h>

#define TXPDO_INDEX 0x1A04
#define TX_CONTROL_WORD 0x6040, 0
#define TX_TARGET_POSITION 0x607A, 0 // RW, int32#define
#define TX_TARGET_VELOCITY 0x60FF, 0
#define TX_PROFILE_ACCELERATION 0x6083, 0
#define TX_PROFILE_DECELERATION 0x6084, 0
#define TX_PROFILE_VELOCITY 0x6081, 0
#define TX_OPERATION_MODE 0x6060, 0
#define TX_DIGITAL_OUTPUT_FUNC 0x2078, 1 //int16, sub ==2~4?

#define RXPDO_INDEX 0x1604
#define RX_ERROR_REGISTER 0x1001, 0
#define RX_STATUS_WORD 0x6041, 0
#define RX_ACTUAL_POSITION 0x6064, 0
#define RX_ACTUAL_VELOCITY 0x606C, 0
#define RX_ACTUAL_CURRENT 0x6078, 0
#define RX_FOLLOWING_ERR_VALUE 0x20F4, 0
#define RX_OPERATION_MODE_DISPLAY 0x6061, 0 //int8
#define RX_DIGITAL_INPUT_FUNC 0x2071, 1		//int16, sub ==2~4?
#define RX_INTERPOLATION_BUFFER 0x20c4, 1   //int16
//#define RX_ACTUAL_CURRENT_AVG 0x2027, 0x00

#define COMMAND_SHUTDOWN 0x0006
#define COMMAND_SWITCH_ON 0x0007
#define COMMAND_DISABLE_VOLTAGE 0x0000
#define COMMAND_QUICK_STOP 0x0002
#define COMMAND_DISABLE_OPERATION 0x0007
#define COMMAND_ENABLE_OPERATION 0x000F
#define COMMAND_FAULT_RESET 0x0080

#define STATE_NOT_READY_TO_SWITCH_ON 0x0000
#define STATE_SWITCH_ON_DISABLED 0x0040
#define STATE_READY_TO_SWITCH_ON 0x0021
#define STATE_SWITCH_ON_ENABLED 0x0023
#define STATE_OPERATION_ENABLED 0x0027
#define STATE_QUICK_STOP_ACTIVE 0x0007
#define STATE_FAULT_REACTION_ACTIVE 0x000F
#define STATE_FAULT 0x0008

#define SDO_MAX_FOLLOWING_ERROR 0x6065, 0x00
#define SDO_INTERPOLATION_TIME_PERIOD 0x60C2, 0x01
#define SDO_PROFILE_POSITION_MODE 0x01
#define SDO_PROFILE_VELOCITY_MODE 0x03
#define SDO_PROFILE_TORQUE_MODE 0x04
#define SDO_CYCLIC_SYNCHRONOUS_POSITION_MODE 0x1000
#define SDO_HOMING_MODE 0x06
#define SDO_INTERPOLATED_POSITION_MODE 0x07
/*
#define DSP_402_POSITION_RANGE 0x607B, 0x00           // R, uint8
#define DSP_402_MIN_POSITION_RANGE_LIMIT 0x607B, 0x01 // RW, int32
#define DSP_402_MAX_POSITION_RANGE_LIMIT 0x607B, 0x02 // RW, int32
#define DSP_402_SOFTWARE_POSITION_LIMIT 0x607D, 0x00
#define DSP_402_MAX_PROFILE_VELOCITY 0x607F, 0x00
#define DSP_402_PROFILE_VELOCITY 0x6081, 0x00
#define DSP_402_PROFILE_ACCELERATION 0x6083, 0x00
#define DSP_402_PROFILE_DECELERATION 0x6084, 0x00
#define COMMAND_QUICK_STOP_DECELERATION 0x6085, 0x00
#define DSP_402_MOTION_PROFILE_TYPE 0x6086, 0x00
*/
#define FREQUENCY 100
#define CLOCK_TO_USE CLOCK_REALTIME
#define MEASURE_TIMING
#define MAXON_EPOS3 0x000000fb, 0x64400000

/*************************************************************************
***/
#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
					   (B).tv_nsec - (A).tv_nsec)
#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec) /*************************************************************************/

extern int get_cfParam(char *name);
extern void operateHA_controller();
extern int get_rtParam_int(char *name, char *field);
pthread_t ecat_cyclic_thread;

extern void sendDebug(char* ret_msg);
typedef struct _master_info
{
	ec_master_t *master;
	ec_master_state_t master_state;
	ec_domain_t **domain;
	ec_domain_state_t *domain_state;
	uint8_t **domain_pd;
	ec_slave_config_t **sc_epos;
	ec_slave_config_state_t *slave_state;
} Master_info;

typedef struct _motor_info
{
	int no;
	int target_pos;
	int act_pos;
	uint16_t status;
} Motor_info;

typedef struct
{
	unsigned int control_word;
	unsigned int target_pos;
	unsigned int target_vel;
	unsigned int prof_accel;
	unsigned int prof_decel;
	unsigned int prof_vel;
	unsigned int mode_op;
	unsigned int digital_out;

	unsigned int status_word;
	unsigned int act_pos;
	unsigned int act_vel;
	unsigned int act_cur;
	unsigned int err_val;
	unsigned int mode_display;
	unsigned int digital_in;
	unsigned int ip_buf;
} offset_PPM;

Master_info master_info;
Motor_info *motor_info;
offset_PPM offset_ppm;

ec_pdo_entry_info_t epos3_pdo_rx_entries[] =
	{
		{TX_CONTROL_WORD, 16},		   // controlword
		{TX_TARGET_POSITION, 32},	  // target position
		{TX_TARGET_VELOCITY, 32},	  // target velocity
		{TX_PROFILE_ACCELERATION, 32}, // profile acceleration
		{TX_PROFILE_DECELERATION, 32}, // profile deceleration
		{TX_PROFILE_VELOCITY, 32},	 // profile velocity
		{TX_OPERATION_MODE, 8},		   // modes of opeartion
		{TX_DIGITAL_OUTPUT_FUNC, 16},  // digital output functionality

};
ec_pdo_entry_info_t epos3_pdo_tx_entries[] =
	{
		{RX_STATUS_WORD, 16},			// status word
		{RX_ACTUAL_POSITION, 32},		// position actual value
		{RX_ACTUAL_VELOCITY, 32},		// velocity actual value
		{RX_ACTUAL_CURRENT, 16},		// current actual value
		{RX_FOLLOWING_ERR_VALUE, 16},   // following error actual value
		{RX_OPERATION_MODE_DISPLAY, 8}, // modes of operation display
		{RX_DIGITAL_INPUT_FUNC, 16},	// digital input functionality
		{RX_INTERPOLATION_BUFFER, 16},  // interpolation buffer status
										//{RX_ACTUAL_CURRENT_AVG, 16},
};

//7 5 4 16
ec_pdo_info_t epos3_pdos_ppm[] =
	{
		{RXPDO_INDEX, 8, epos3_pdo_rx_entries},
		{TXPDO_INDEX, 8, epos3_pdo_tx_entries},
};

ec_sync_info_t epos3_syncs[] =
	{

		{2, EC_DIR_OUTPUT, 1, epos3_pdos_ppm + 0},
		{3, EC_DIR_INPUT, 1, epos3_pdos_ppm + 1},
		{0xff}};

int get_slave_physical_act_pos(unsigned int slave_no);
void set_slave_physical_target_pos(unsigned int slave_no, int value);
int get_slave_physical_target_pos(unsigned int slave_no);

/////////////////////////////////////////////
// Local definitions
/////////////////////////////////////////////
int *control_cnt;
int *tpos_timestamp;
void mallocArray()
{
	control_cnt = (int *)malloc(sizeof(int) * get_rtParam_int("ECAT.num_motors", "value"));
	for (int i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
		control_cnt[i] = 3;
	tpos_timestamp = (int *)malloc(sizeof(int) * get_rtParam_int("ECAT.num_motors", "value"));
	for (int i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
		tpos_timestamp[i] = -1;
	motor_info = (Motor_info *)malloc(sizeof(Motor_info) * get_rtParam_int("ECAT.num_motors", "value"));
	master_info.sc_epos = (ec_slave_config_t **)malloc(sizeof(ec_slave_config_t *) * get_rtParam_int("ECAT.num_motors", "value"));
	master_info.slave_state = (ec_slave_config_state_t *)malloc(sizeof(ec_slave_config_state_t) * get_rtParam_int("ECAT.num_motors", "value"));
	master_info.domain = (ec_domain_t **)malloc(sizeof(ec_domain_t *) * get_rtParam_int("ECAT.num_motors", "value"));
	master_info.domain_state = (ec_domain_state_t *)malloc(sizeof(ec_domain_state_t) * get_rtParam_int("ECAT.num_motors", "value"));
	master_info.domain_pd = (uint8_t **)malloc(sizeof(uint8_t *) * get_rtParam_int("ECAT.num_motors", "value"));
}
void initSlaveInfo() //using in EcatMgr.cpp
{
	int idx;

	for (idx = 0; idx < get_rtParam_int("ECAT.num_motors", "value"); ++idx)
	{
		motor_info[idx].no = idx;
		motor_info[idx].target_pos = 0;
		motor_info[idx].act_pos = 0;
		motor_info[idx].status = 0;
	}
}
int initTimerfd(int msec)
{
	int fd = -1;
	struct itimerspec timeout;

	if ((fd = timerfd_create(CLOCK_REALTIME, 0)) < 0)
	{
		printf("initTimerfd error");
		exit(1);
	}

	timeout.it_interval.tv_sec = 0;
	timeout.it_interval.tv_nsec = msec * 1000 * 1000;
	timeout.it_value.tv_sec = 0;
	timeout.it_value.tv_nsec = msec * 1000 * 1000;
	if (timerfd_settime(fd, 0, &timeout, NULL) != 0)
	{
		printf("timerfd_settime error\n");
		exit(1);
	}

	return fd;
}

//we need to check the state of domain , state of the domain should be equal to 2
// when it exchanged the data
int checkDomainState()
{
	int i = 0;
	int ret_val = 0;
	for (i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
	{
		ec_domain_state_t ds;
		ecrt_domain_state(master_info.domain[i], &ds);
		if (ds.working_counter != master_info.domain_state[i].working_counter)
			printf("Domain %d : WC %u.\n", i, ds.working_counter);
		if (ds.wc_state != master_info.domain_state[i].wc_state)
			printf("Domain %d : State %u.\n", i, ds.wc_state);
		master_info.domain_state[i] = ds;
		if (master_info.domain_state[i].wc_state == 2)
		{
			ret_val = 0;
		}
		else
		{
			ret_val = -1;
		}
	}
	return ret_val;
}
/*************************************************************************
	****/
//check the state of the master that it is in operation, link in up and no of slaves
//responding
void checkMasterState(void)
{
	ec_master_state_t ms;
	ecrt_master_state(master_info.master, &ms);
	if (ms.slaves_responding != master_info.master_state.slaves_responding)
		printf("%u slave(s).\n", ms.slaves_responding);
	if (ms.al_states != master_info.master_state.al_states)
		printf("AL states: 0x%02X.\n", ms.al_states);
	if (ms.link_up != master_info.master_state.link_up)
		printf("Link is %s.\n", ms.link_up ? "up" : "down");
	master_info.master_state = ms;
}
/*************************************************************************
	****/
//check slave state it is in operational state or not, PDO data can not be transfer it
//slave is not in operational state

void checkSlaveStates()
{
	int i = 0;
	for (i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
	{
		ec_slave_config_state_t s;

		ecrt_slave_config_state(master_info.sc_epos[i], &s);
		if (s.al_state != master_info.slave_state[i].al_state)
			printf("EPOS3 slave %d State 0x%02X.\n", i, s.al_state);
		if (s.online != master_info.slave_state[i].online)
			printf("EPOS3 slave %d: %s.\n", i, s.online ? "online" : "offline");
		if (s.operational != master_info.slave_state[i].operational)
			printf("EPOS3 slave %d: %soperational.\n", i,
				   s.operational ? "" : "Not ");
		master_info.slave_state[i] = s;
	}
}
void printErrorReg(unsigned char err_reg)
{
	if ((err_reg & 0xFF) == 0x80)
	{
		printf("ERROR Register : Motion Error\n");
	}
	else if ((err_reg & 0xFF) == 0x20)
	{
		printf("ERROR Register : Device profile-spcific Error\n");
	}
	else if ((err_reg & 0xFF) == 0x10)
	{
		printf("ERROR Register : Communication Error\n");
	}
 	else if ((err_reg & 0xFF) == 0x08)
	{
		printf("ERROR Register : Temperature Error\n");
	}
	else if ((err_reg & 0xFF) == 0x04)
	{
		printf("ERROR Register : Voltage Error\n");
	}
	else if ((err_reg & 0xFF) == 0x02)
	{
		printf("ERROR Register : Current Error\n");
	}
	else if ((err_reg & 0xFF) == 0x01)
	{
		printf("ERROR Register : Generic Error\n");
	}
}
//get the status of ethercat slaves using SDO
unsigned short uploadSlaveStatus(unsigned int position)
{
	unsigned short statusWord = 0xFFFF;
	int errorCode = 0;
	size_t resultSize = 0;
	uint32_t abortCode = 0;
	if ((errorCode = ecrt_master_sdo_upload(master_info.master, position, RX_STATUS_WORD,
											(unsigned char *)&statusWord, 2, &resultSize, &abortCode)) < 0)
	{
		printf("Abort Message : %x\n", abortCode);
		return statusWord;
	}
	if ((statusWord & 0x4F) == 0x00)
	{
		printf("%d : Not ready to switch on.\n", position);
		statusWord = STATE_NOT_READY_TO_SWITCH_ON;
	}
	else if ((statusWord & 0x4F) == 0x40)
	{
		printf("%d : Switch on disabled.\n", position);
		statusWord = STATE_SWITCH_ON_DISABLED;
	}
	else if ((statusWord & 0x6F) == 0x21)
	{
		printf("%d : Ready to switch on.\n", position);
		statusWord = STATE_READY_TO_SWITCH_ON;
	}
	else if ((statusWord & 0x6F) == 0x23)
	{
		printf("%d : Switch on enabled.\n", position);
		statusWord = STATE_SWITCH_ON_ENABLED;
	}
	else if ((statusWord & 0x6F) == 0x27)
	{
		printf("%d : Operation enabled.\n", position);
		statusWord = STATE_OPERATION_ENABLED;
	}
	else if ((statusWord & 0x6F) == 0x07)
	{
		printf("%d : Quick stop active.\n", position);
		statusWord = STATE_QUICK_STOP_ACTIVE;
	}
	else if ((statusWord & 0x4F) == 0x0F)
	{
		unsigned char err_reg;
		printf("%d : Fault reaction active.\n", position);
		ecrt_master_sdo_upload(master_info.master, position, RX_ERROR_REGISTER, &err_reg, 2, &resultSize, &abortCode);
		printErrorReg(err_reg);
		statusWord = STATE_FAULT_REACTION_ACTIVE;
	}
	else if ((statusWord & 0x4F) == 0x08)
	{
		unsigned char err_reg;
		printf("%d : Fault, ", position);
		ecrt_master_sdo_upload(master_info.master, position, RX_ERROR_REGISTER, &err_reg, 2, &resultSize, &abortCode);
		printErrorReg(err_reg);
		statusWord = STATE_FAULT;
	}
	return statusWord;
} // FSM for move the slave in operational state using SDO
int downloadMasterCommand()
{
	unsigned short controlWord;
	unsigned short statusWord;
	int errorCode = 0;
	uint32_t abortCode = 0;
	unsigned char period = 10;
	unsigned char operationMode = 1;
	unsigned long maxFlowingError = 2600000;

	int i = 0;
	for (i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
	{
		if ((errorCode = ecrt_master_sdo_download(master_info.master, i, 0x6065, 0x00, (unsigned char *)&maxFlowingError, 4, &abortCode)) < 0)
		{
			printf("downloadMasterCommand : Abort Message : %x\n", abortCode);
			return -1;
		}
		if ((errorCode = ecrt_master_sdo_download(master_info.master, i, TX_OPERATION_MODE,
												  (unsigned char *)&operationMode, 1, &abortCode)) < 0)
		{
			printf("downloadMasterCommand2 : Abort Message : %x\n", abortCode);
			return -1;
		}
		/*if ((errorCode = ecrt_master_sdo_download(master_info.master, i, 0x60C2, 0x01, &period, 1,
	                                                  &abortCode)) < 0)
	        {
	            printf("Abort Message : %x\n", abortCode);
	            return;
	        }*/
		while ((statusWord = uploadSlaveStatus(i)) != STATE_OPERATION_ENABLED)
		{
			if (statusWord == STATE_FAULT_REACTION_ACTIVE)
			{
				controlWord = COMMAND_ENABLE_OPERATION;
			}
			else if (statusWord == STATE_FAULT)
			{
				controlWord = COMMAND_FAULT_RESET;
			}
			else if (statusWord == STATE_SWITCH_ON_DISABLED)
			{
				controlWord = COMMAND_SHUTDOWN;
			}
			else if (statusWord == STATE_READY_TO_SWITCH_ON)
			{
				controlWord = COMMAND_SWITCH_ON;
			}
			else if (statusWord == STATE_SWITCH_ON_ENABLED)
			{
				controlWord = COMMAND_ENABLE_OPERATION;
			}
			if ((errorCode = ecrt_master_sdo_download(master_info.master, i, TX_CONTROL_WORD,
													  (unsigned char *)&controlWord, 2, &abortCode)) < 0)
			{
				printf("Abort Message : %x\n", abortCode);
				return -1;
			}
			usleep(1000);
		}
	}
}

void profileMotorSpeed(int slave, int vel, int accel, int decel)
{
	EC_WRITE_U32(master_info.domain_pd[slave] + offset_ppm.prof_vel, vel);
	EC_WRITE_U32(master_info.domain_pd[slave] + offset_ppm.prof_accel, accel);
	EC_WRITE_U32(master_info.domain_pd[slave] + offset_ppm.prof_decel, decel);
	
}
//////////////////////////////////////////////////////////////
unsigned short get_slave_status(unsigned int slave_no)
{
	motor_info[slave_no].status = EC_READ_U16(master_info.domain_pd[slave_no] + offset_ppm.status_word);
	return motor_info[slave_no].status;
}
void update_tpos_timestamp(int slave_no)
{
	if (slave_no == get_cfParam("ECAT.motor_id_throttle"))
    {
			tpos_timestamp[slave_no] = get_rtParam_int("Throttle.target_position", "timestamp");
			return ;
    }
    else if (slave_no == get_cfParam("ECAT.motor_id_brake"))
    {
			tpos_timestamp[slave_no] = get_rtParam_int("Brake.target_position", "timestamp");
			return ;
    }
    
    else if (slave_no == get_cfParam("ECAT.motor_id_lidar_center"))
    {
			tpos_timestamp[slave_no] = get_rtParam_int("Lidar.center_target_position", "timestamp");
			return ;
    }
    else if (slave_no == get_cfParam("ECAT.motor_id_lidar_left"))
    {
			tpos_timestamp[slave_no] = get_rtParam_int("Lidar.left_target_position", "timestamp");
			return ;
    }
    else if (slave_no == get_cfParam("ECAT.motor_id_lidar_right"))
    {
			tpos_timestamp[slave_no] = get_rtParam_int("Lidar.right_target_position", "timestamp");
			return ;
    }
    else if (slave_no == get_cfParam("ECAT.motor_id_steer"))
    {
			tpos_timestamp[slave_no] = get_rtParam_int("Steerwheel.target_position", "timestamp");
			return ;
    }
    else if (slave_no == get_cfParam("ECAT.motor_id_gearstick"))
    {
			tpos_timestamp[slave_no] = get_rtParam_int("Gearstick.target_position", "timestamp");
			return ;
    }

    return ;
}
int is_tpos_timestamp_updated(int slave_no)
{
	if (slave_no == get_cfParam("ECAT.motor_id_throttle"))
    {
        if(tpos_timestamp[slave_no] != get_rtParam_int("Throttle.target_position", "timestamp"))
		{
			return 1;
		}
    }
    else if (slave_no == get_cfParam("ECAT.motor_id_brake"))
    {
        if(tpos_timestamp[slave_no] != get_rtParam_int("Brake.target_position", "timestamp"))
		{
			return 1;
		}
    }
    
    else if (slave_no == get_cfParam("ECAT.motor_id_lidar_center"))
    {
        if(tpos_timestamp[slave_no] != get_rtParam_int("Lidar.center_target_position", "timestamp"))
		{
			return 1;
		}
    }
    else if (slave_no == get_cfParam("ECAT.motor_id_lidar_left"))
    {
        if(tpos_timestamp[slave_no] != get_rtParam_int("Lidar.left_target_position", "timestamp"))
		{
			return 1;
		}
    }
    else if (slave_no == get_cfParam("ECAT.motor_id_lidar_right"))
    {
        if(tpos_timestamp[slave_no] != get_rtParam_int("Lidar.right_target_position", "timestamp"))
		{
			return 1;
		}
    }
    else if (slave_no == get_cfParam("ECAT.motor_id_steer"))
    {
        if(tpos_timestamp[slave_no] != get_rtParam_int("Steerwheel.target_position", "timestamp"))
		{
			return 1;
		}
    }
    else if (slave_no == get_cfParam("ECAT.motor_id_gearstick"))
    {
        if(tpos_timestamp[slave_no] != get_rtParam_int("Gearstick.target_position", "timestamp"))
		{
			return 1;
		}
    }

    return -1;
}
int is_ecat_ready()
{
	int i = 0;
	int result = -1;
	unsigned short statusWord;
	for (i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
	{
		statusWord = get_slave_status(i);
		if((statusWord & 0x6F) != 0x27)
		{
			result = -1;
		}
		else 
			result = 1;
	}
	return result;
}
void printSlaveStatus(int slave_no)
{
	unsigned short statusWord;
	statusWord = get_slave_status(slave_no);
	if ((statusWord & 0x4F) == 0x00)
	{
		printf("%d Status : STATE_NOT_READY_TO_SWITCH_ON(%x) ", slave_no, statusWord);
	}
	else if ((statusWord & 0x4F) == 0x40)
	{
		printf("%d Status : STATE_SWITCH_ON_DISABLED(%x) ", slave_no, statusWord);
	}
	else if ((statusWord & 0x6F) == 0x21)
	{
		printf("%d Status : STATE_READY_TO_SWITCH_ON(%x) ", slave_no, statusWord);
	}
	else if ((statusWord & 0x6F) == 0x23)
	{
		printf("%d Status : STATE_SWITCH_ON_ENABLED(%x) ", slave_no, statusWord);
	}
	else if ((statusWord & 0x6F) == 0x27)
	{
		printf("%d Status : STATE_OPERATION_ENABLED(%x) ", slave_no, statusWord);
	}
	else if ((statusWord & 0x6F) == 0x07)
	{
		printf("%d Status : STATE_QUICK_STOP_ACTIVE(%x) ", slave_no, statusWord);
	}
	else if ((statusWord & 0x4F) == 0x0F)
	{
		printf("%d Status : STATE_FAULT_REACTION_ACTIVE(%x) ", slave_no, statusWord);
	}
	else if ((statusWord & 0x4F) == 0x08)
	{
		printf("%d Status : STATE_FAULT(%x) ", checkSlaveStates, statusWord);
	}
}
void printSlaveInfo(int slave)
{
	printSlaveStatus(slave);
	printf("tar_pos = %x, ", EC_READ_S32(master_info.domain_pd[slave] + offset_ppm.target_pos));
	printf("act_pos = %d, ", EC_READ_S32(master_info.domain_pd[slave] + offset_ppm.act_pos));
	printf("act_vel = %d, ", EC_READ_S32(master_info.domain_pd[slave] + offset_ppm.act_vel));
	printf("act_cur = %dmA, ", EC_READ_S16(master_info.domain_pd[slave] + offset_ppm.act_cur));
	printf("err_val = %d, ", EC_READ_S16(master_info.domain_pd[slave] + offset_ppm.err_val));
	printf("mode_display = %x, ", EC_READ_S8(master_info.domain_pd[slave] + offset_ppm.mode_display));

	printf("\nACCEL = %d\n", EC_READ_S32(master_info.domain_pd[slave] + offset_ppm.prof_accel));
	printf("DECEL = %d\n", EC_READ_S32(master_info.domain_pd[slave] + offset_ppm.prof_decel));
	printf("VEL = %d\n", EC_READ_S32(master_info.domain_pd[slave] + offset_ppm.prof_vel));
}
void preprocessingECAT() //using in EcatMgr.cpp
{
	int i;

	static int counter = 0;
	int actualPosition = 0;

	// receive process data
	ecrt_master_receive(master_info.master);
	for (i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
	{
		ecrt_domain_process(master_info.domain[i]);
	}
	// check process data state (optional)

	if (counter)
	{
		counter--;
	}
	else
	{
		// do this at 1 Hz
		counter = FREQUENCY;
		if (checkDomainState() != 0)
		{
			for (i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
			{
				get_slave_status(i);

				printSlaveStatus(i);
				printf("/ Target Position : %d ", EC_READ_S32(master_info.domain_pd[i] + offset_ppm.target_pos));
				actualPosition = EC_READ_S32(master_info.domain_pd[i] + offset_ppm.act_pos);
				printf("/ Actual Position : %d\n", actualPosition);
			}
			printf("\n");
		}
	}

	for (i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
	{
		get_slave_status(i);
	}
	checkDomainState();
	checkMasterState();
	checkSlaveStates();
}

void postprocessingECAT() //using in EcatMgr.cpp
{
	int i;
	struct timespec time;
	static int sync_ref_counter = 0;

	clock_gettime(CLOCK_TO_USE, &time);									 //this command should be use for sync dc,
	ecrt_master_application_time(master_info.master, TIMESPEC2NS(time)); //sync ethrcat master with current time
	if (sync_ref_counter)
	{
		sync_ref_counter--;
	}
	else
	{
		sync_ref_counter = 1; // sync every cycle
		ecrt_master_sync_reference_clock(master_info.master);
	}
	ecrt_master_sync_slave_clocks(master_info.master); //kindly see ecrt.h for function definition
	for (i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
	{
		ecrt_domain_queue(master_info.domain[i]);
	}
	ecrt_master_send(master_info.master); //send all data to ethercat slave
}

void WriteTargetPos_PPM(int slave)
{
	if (control_cnt[slave] == 3)
	{
		EC_WRITE_U16(master_info.domain_pd[slave] + offset_ppm.control_word, 0x0F);
		control_cnt[slave]--;
	}
	else if (control_cnt[slave] == 2)
	{
		EC_WRITE_S32(master_info.domain_pd[slave] + offset_ppm.target_pos, motor_info[slave].target_pos);
		control_cnt[slave]--;
	}
	else if (control_cnt[slave] == 1)
	{
		EC_WRITE_U16(master_info.domain_pd[slave] + offset_ppm.control_word, 0x3F);
		control_cnt[slave] = 3;
		update_tpos_timestamp(slave);
	}
}
int ecatUp()
{
	int i;
	const static ec_pdo_entry_reg_t throttle_domain_regs[] = {
		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, TX_CONTROL_WORD, &offset_ppm.control_word},
		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, TX_TARGET_POSITION, &offset_ppm.target_pos},
		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, TX_TARGET_VELOCITY, &offset_ppm.target_vel},
		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, TX_PROFILE_ACCELERATION, &offset_ppm.prof_accel},
		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, TX_PROFILE_DECELERATION, &offset_ppm.prof_decel},
		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, TX_PROFILE_VELOCITY, &offset_ppm.prof_vel},
		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, TX_OPERATION_MODE, &offset_ppm.mode_op},
		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, TX_DIGITAL_OUTPUT_FUNC, &offset_ppm.digital_out},

		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, RX_STATUS_WORD, &offset_ppm.status_word},
		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, RX_ACTUAL_POSITION, &offset_ppm.act_pos},
		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, RX_ACTUAL_VELOCITY, &offset_ppm.act_vel},
		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, RX_ACTUAL_CURRENT, &offset_ppm.act_cur},
		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, RX_FOLLOWING_ERR_VALUE, &offset_ppm.err_val},
		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, RX_OPERATION_MODE_DISPLAY, &offset_ppm.mode_display},
		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, RX_DIGITAL_INPUT_FUNC, &offset_ppm.digital_in},
		{0, get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, RX_INTERPOLATION_BUFFER, &offset_ppm.ip_buf},
		//{0,get_cfParam("ECAT.motor_id_throttle"), MAXON_EPOS3, RX_ACTUAL_CURRENT_AVG, &offset_ppm.act_cur_avg},
		{}};
	const static ec_pdo_entry_reg_t brake_domain_regs[] = {
		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, TX_CONTROL_WORD, &offset_ppm.control_word},
		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, TX_TARGET_POSITION, &offset_ppm.target_pos},
		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, TX_TARGET_VELOCITY, &offset_ppm.target_vel},
		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, TX_PROFILE_ACCELERATION, &offset_ppm.prof_accel},
		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, TX_PROFILE_DECELERATION, &offset_ppm.prof_decel},
		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, TX_PROFILE_VELOCITY, &offset_ppm.prof_vel},
		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, TX_OPERATION_MODE, &offset_ppm.mode_op},
		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, TX_DIGITAL_OUTPUT_FUNC, &offset_ppm.digital_out},

		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, RX_STATUS_WORD, &offset_ppm.status_word},
		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, RX_ACTUAL_POSITION, &offset_ppm.act_pos},
		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, RX_ACTUAL_VELOCITY, &offset_ppm.act_vel},
		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, RX_ACTUAL_CURRENT, &offset_ppm.act_cur},
		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, RX_FOLLOWING_ERR_VALUE, &offset_ppm.err_val},
		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, RX_OPERATION_MODE_DISPLAY, &offset_ppm.mode_display},
		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, RX_DIGITAL_INPUT_FUNC, &offset_ppm.digital_in},
		{0, get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, RX_INTERPOLATION_BUFFER, &offset_ppm.ip_buf},
		//{0,get_cfParam("ECAT.motor_id_brake"), MAXON_EPOS3, RX_ACTUAL_CURRENT_AVG, &offset_ppm.act_cur_avg},
		{}};

	const static ec_pdo_entry_reg_t clidar_domain_regs[] = {
		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, TX_CONTROL_WORD, &offset_ppm.control_word},
		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, TX_TARGET_POSITION, &offset_ppm.target_pos},
		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, TX_TARGET_VELOCITY, &offset_ppm.target_vel},
		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, TX_PROFILE_ACCELERATION, &offset_ppm.prof_accel},
		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, TX_PROFILE_DECELERATION, &offset_ppm.prof_decel},
		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, TX_PROFILE_VELOCITY, &offset_ppm.prof_vel},
		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, TX_OPERATION_MODE, &offset_ppm.mode_op},
		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, TX_DIGITAL_OUTPUT_FUNC, &offset_ppm.digital_out},

		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, RX_STATUS_WORD, &offset_ppm.status_word},
		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, RX_ACTUAL_POSITION, &offset_ppm.act_pos},
		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, RX_ACTUAL_VELOCITY, &offset_ppm.act_vel},
		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, RX_ACTUAL_CURRENT, &offset_ppm.act_cur},
		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, RX_FOLLOWING_ERR_VALUE, &offset_ppm.err_val},
		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, RX_OPERATION_MODE_DISPLAY, &offset_ppm.mode_display},
		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, RX_DIGITAL_INPUT_FUNC, &offset_ppm.digital_in},
		{0, get_cfParam("ECAT.motor_id_lidar_center"), MAXON_EPOS3, RX_INTERPOLATION_BUFFER, &offset_ppm.ip_buf},
		{}};

	const static ec_pdo_entry_reg_t llidar_domain_regs[] = {
		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, TX_CONTROL_WORD, &offset_ppm.control_word},
		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, TX_TARGET_POSITION, &offset_ppm.target_pos},
		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, TX_TARGET_VELOCITY, &offset_ppm.target_vel},
		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, TX_PROFILE_ACCELERATION, &offset_ppm.prof_accel},
		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, TX_PROFILE_DECELERATION, &offset_ppm.prof_decel},
		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, TX_PROFILE_VELOCITY, &offset_ppm.prof_vel},
		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, TX_OPERATION_MODE, &offset_ppm.mode_op},
		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, TX_DIGITAL_OUTPUT_FUNC, &offset_ppm.digital_out},

		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, RX_STATUS_WORD, &offset_ppm.status_word},
		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, RX_ACTUAL_POSITION, &offset_ppm.act_pos},
		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, RX_ACTUAL_VELOCITY, &offset_ppm.act_vel},
		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, RX_ACTUAL_CURRENT, &offset_ppm.act_cur},
		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, RX_FOLLOWING_ERR_VALUE, &offset_ppm.err_val},
		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, RX_OPERATION_MODE_DISPLAY, &offset_ppm.mode_display},
		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, RX_DIGITAL_INPUT_FUNC, &offset_ppm.digital_in},
		{0, get_cfParam("ECAT.motor_id_lidar_left"), MAXON_EPOS3, RX_INTERPOLATION_BUFFER, &offset_ppm.ip_buf},
		{}};

	const static ec_pdo_entry_reg_t rlidar_domain_regs[] = {
		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, TX_CONTROL_WORD, &offset_ppm.control_word},
		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, TX_TARGET_POSITION, &offset_ppm.target_pos},
		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, TX_TARGET_VELOCITY, &offset_ppm.target_vel},
		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, TX_PROFILE_ACCELERATION, &offset_ppm.prof_accel},
		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, TX_PROFILE_DECELERATION, &offset_ppm.prof_decel},
		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, TX_PROFILE_VELOCITY, &offset_ppm.prof_vel},
		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, TX_OPERATION_MODE, &offset_ppm.mode_op},
		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, TX_DIGITAL_OUTPUT_FUNC, &offset_ppm.digital_out},

		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, RX_STATUS_WORD, &offset_ppm.status_word},
		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, RX_ACTUAL_POSITION, &offset_ppm.act_pos},
		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, RX_ACTUAL_VELOCITY, &offset_ppm.act_vel},
		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, RX_ACTUAL_CURRENT, &offset_ppm.act_cur},
		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, RX_FOLLOWING_ERR_VALUE, &offset_ppm.err_val},
		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, RX_OPERATION_MODE_DISPLAY, &offset_ppm.mode_display},
		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, RX_DIGITAL_INPUT_FUNC, &offset_ppm.digital_in},
		{0, get_cfParam("ECAT.motor_id_lidar_right"), MAXON_EPOS3, RX_INTERPOLATION_BUFFER, &offset_ppm.ip_buf},
		{}};
	const static ec_pdo_entry_reg_t wheel_domain_regs[] = {
		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, TX_CONTROL_WORD, &offset_ppm.control_word},
		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, TX_TARGET_POSITION, &offset_ppm.target_pos},
		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, TX_TARGET_VELOCITY, &offset_ppm.target_vel},
		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, TX_PROFILE_ACCELERATION, &offset_ppm.prof_accel},
		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, TX_PROFILE_DECELERATION, &offset_ppm.prof_decel},
		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, TX_PROFILE_VELOCITY, &offset_ppm.prof_vel},
		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, TX_OPERATION_MODE, &offset_ppm.mode_op},
		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, TX_DIGITAL_OUTPUT_FUNC, &offset_ppm.digital_out},

		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, RX_STATUS_WORD, &offset_ppm.status_word},
		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, RX_ACTUAL_POSITION, &offset_ppm.act_pos},
		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, RX_ACTUAL_VELOCITY, &offset_ppm.act_vel},
		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, RX_ACTUAL_CURRENT, &offset_ppm.act_cur},
		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, RX_FOLLOWING_ERR_VALUE, &offset_ppm.err_val},
		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, RX_OPERATION_MODE_DISPLAY, &offset_ppm.mode_display},
		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, RX_DIGITAL_INPUT_FUNC, &offset_ppm.digital_in},
		{0, get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, RX_INTERPOLATION_BUFFER, &offset_ppm.ip_buf},
		//{0,get_cfParam("ECAT.motor_id_steer"), MAXON_EPOS3, RX_ACTUAL_CURRENT_AVG, &offset_ppm.act_cur_avg},
		{}};
	const static ec_pdo_entry_reg_t gearstick_domain_regs[] = {
		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, TX_CONTROL_WORD, &offset_ppm.control_word},
		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, TX_TARGET_POSITION, &offset_ppm.target_pos},
		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, TX_TARGET_VELOCITY, &offset_ppm.target_vel},
		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, TX_PROFILE_ACCELERATION, &offset_ppm.prof_accel},
		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, TX_PROFILE_DECELERATION, &offset_ppm.prof_decel},
		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, TX_PROFILE_VELOCITY, &offset_ppm.prof_vel},
		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, TX_OPERATION_MODE, &offset_ppm.mode_op},
		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, TX_DIGITAL_OUTPUT_FUNC, &offset_ppm.digital_out},

		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, RX_STATUS_WORD, &offset_ppm.status_word},
		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, RX_ACTUAL_POSITION, &offset_ppm.act_pos},
		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, RX_ACTUAL_VELOCITY, &offset_ppm.act_vel},
		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, RX_ACTUAL_CURRENT, &offset_ppm.act_cur},
		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, RX_FOLLOWING_ERR_VALUE, &offset_ppm.err_val},
		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, RX_OPERATION_MODE_DISPLAY, &offset_ppm.mode_display},
		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, RX_DIGITAL_INPUT_FUNC, &offset_ppm.digital_in},
		{0, get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, RX_INTERPOLATION_BUFFER, &offset_ppm.ip_buf},
		//{0,get_cfParam("ECAT.motor_id_gearstick"), MAXON_EPOS3, RX_ACTUAL_CURRENT_AVG, &offset_ppm.act_cur_avg},
		{}};

	mallocArray();
	initSlaveInfo();
	//ec_slave_config_t *sc;
	//struct itimerval tv;

	master_info.master = ecrt_request_master(0);

	printf("ecrt_request_master is called \n");

	if (!master_info.master)
		return -1;
	for (i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
	{
		master_info.domain[i] = ecrt_master_create_domain(master_info.master);
		if (!master_info.domain[i])
			return -1;
	}

	for (i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
	{
		if (!(master_info.sc_epos[i] = ecrt_master_slave_config(master_info.master, 0, i, MAXON_EPOS3)))
		{
			fprintf(stderr, "Failed to get slave configuration. \n");
			return -1;
		}
	}

	if(downloadMasterCommand() == -1)
		return -1;
	printf("Configuring PDOs...\n");
	sendDebug("Configuring PDOs...\n");
	for (i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
	{
		if (ecrt_slave_config_pdos(master_info.sc_epos[i], EC_END, epos3_syncs))
		{
			fprintf(stderr, "Failed to configure PDOs.\n");
			return -1;
		}
	}

	printf("configureing PDO is completed!\n");
	sendDebug("configureing PDO is completed!\n");
	if (get_rtParam_int("ECAT.num_motors", "value") > 0)
	{
		if (ecrt_domain_reg_pdo_entry_list(master_info.domain[0], throttle_domain_regs))
		{
			fprintf(stderr, "PDO entry registration failed! \n");
			return -1;
		}
	}
	if (get_rtParam_int("ECAT.num_motors", "value") > 1)
	{
		if (ecrt_domain_reg_pdo_entry_list(master_info.domain[1], brake_domain_regs))
		{
			fprintf(stderr, "PDO entry registration failed! \n");
			return -1;
		}
	}

	if (get_rtParam_int("ECAT.num_motors", "value") > 2)
	{
		if (ecrt_domain_reg_pdo_entry_list(master_info.domain[2], clidar_domain_regs))
		{
			fprintf(stderr, "PDO entry registration failed! \n");
			return -1;
		}
	}
	if (get_rtParam_int("ECAT.num_motors", "value") > 3)
	{
		if (ecrt_domain_reg_pdo_entry_list(master_info.domain[3], llidar_domain_regs))
		{
			fprintf(stderr, "PDO entry registration failed! \n");
			return -1;
		}
	}
	if (get_rtParam_int("ECAT.num_motors", "value") > 4)
	{
		if (ecrt_domain_reg_pdo_entry_list(master_info.domain[4], rlidar_domain_regs))
		{
			fprintf(stderr, "PDO entry registration failed! \n");
			return -1;
		}
	}
	if (get_rtParam_int("ECAT.num_motors", "value") > 5)
	{
		if (ecrt_domain_reg_pdo_entry_list(master_info.domain[5], wheel_domain_regs))
		{
			fprintf(stderr, "PDO entry registration failed! \n");
			return -1;
		}
	}
	if (get_rtParam_int("ECAT.num_motors", "value") > 6)
	{
		if (ecrt_domain_reg_pdo_entry_list(master_info.domain[6], gearstick_domain_regs))
		{
			fprintf(stderr, "PDO entry registration failed! \n");
			return -1;
		}
	}

	// configure SYNC signals for this slave
	for (i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
	{
		ecrt_slave_config_dc(master_info.sc_epos[i], 0x0300, 10000000, 4400000, 0, 0);
	}
	printf("Activating master...\n");
	sendDebug("Activating master...\n");

	if (ecrt_master_activate(master_info.master))
		return -1;
	for (i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
	{
		if (!(master_info.domain_pd[i] = ecrt_domain_data(master_info.domain[i])))
		{
			return -1;
		}
	}
	for (i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
	{
		if (i == get_cfParam("ECAT.motor_id_brake"))
		{
			profileMotorSpeed(i, 1428500, 1000000000, 1000000000);
			//profileMotorSpeed(i, 3571, 150000, 150000);
		}
		else if (i == get_cfParam("ECAT.motor_id_steer"))
		{
			profileMotorSpeed(i, 3571, 5000, 5000);
		}
		else 
			profileMotorSpeed(i, 3571, 150000, 150000);
	}
	printf("Ecat UP FINISHED...\n");
	sendDebug("Ecat UP FINISHED...\n");
	return 1;
}
void disableMotor(int slave)
{
	unsigned short controlWord;
	unsigned short statusWord;
	int errorCode = 0;
	uint32_t abortCode = 0;

	controlWord = COMMAND_DISABLE_OPERATION;

    printf("Disabling motor!\n");
	if ((errorCode = ecrt_master_sdo_download(master_info.master, slave, TX_CONTROL_WORD,
											  (unsigned char *)&controlWord, 2, &abortCode)) < 0)
	{
		printf("Abort Message : %x\n", abortCode);
		return -1;
	}
}
extern double get_cur_steer_angle();
extern void sendOdom(double param_val, int bparam);

void ecatCyclic(void *)
{
	int i = 0;
	int timerfd;
	unsigned long long missed;

	timerfd = initTimerfd(10);
	while (1)
	{
		for (i = 0; i < get_rtParam_int("ECAT.num_motors", "value"); i++)
		{
			preprocessingECAT();
            if(i==0 || i==1)
				WriteTargetPos_PPM(i);
            else 
            {
                if(is_tpos_timestamp_updated(i) == 1)			
				    WriteTargetPos_PPM(i);
            }
			get_slave_physical_act_pos(i);
			operateHA_controller();
			postprocessingECAT();
			if(get_rtParam_int("OBD.publish_to_agent","value") == 1 )
			{
				sendOdom(get_cur_steer_angle(), 3);
			} 
			read(timerfd, &missed, sizeof(missed));
		}
	}
}

int ecatOn() //using in EcatMgr.cpp
{
	//if (pthread_create(&thread_id, NULL, &input, (void *)NULL))  ;
	if (pthread_create(&ecat_cyclic_thread, NULL, &ecatCyclic, (void *)NULL))
		pthread_detach(ecat_cyclic_thread);
	//getchar();
	/*if (pthread_create(&thread_id2, NULL, &behavior, (void *)NULL))
        pthread_detach(thread_id2);*/
	printf("Ecat ON FINISHED...\n");
	sendDebug("Ecat ON FINISHED...\n");
	return 1;
}

extern int hviDown();
extern int set_rtParam(char *name, char *field, double val);

int ecatOff()
{
	printf("ecatOff called\n");

	pthread_cancel(ecat_cyclic_thread);
	printf("ecat_cyclic_thread dead\n");
	set_rtParam("Motion.state", "value", 0);
    hviDown();
	return 1;
}
int ecatDown() //using in EcatMgr.cpp
{
	printf("ecatDown called\n");
	ecrt_master_deactivate(master_info.master);
	ecrt_release_master(master_info.master);

	printf("ecatDown finished\n");

    set_rtParam("Motion.state", "value", 0);
	return 1;
}

void set_slave_physical_target_pos(unsigned int slave, int value) // same
{
	//printf("%d , value : %d\n", slave,value);
	motor_info[slave].target_pos = value;
	//printf("clidar physical_target_pos : %d\n", motor_info[2].target_pos);
}
int get_slave_physical_target_pos(unsigned int slave) // same
{
	return motor_info[slave].target_pos;
}
int get_slave_physical_act_pos(unsigned int slave)
{
	motor_info[slave].act_pos = EC_READ_S32(master_info.domain_pd[slave] + offset_ppm.act_pos);
	return motor_info[slave].act_pos;
}

/* qjin ,"not using"
void WriteTargetPos_CSP(int slave_no)
{
    EC_WRITE_S32(master_info.domain_pd[slave_no] + offset_ppm.target_pos, slave_info[slave_no].target_pos);
    EC_WRITE_U16(master_info.domain_pd[slave_no] + offset_ppm.control_word, 0x0F);
}

struct timespec timespecAdd(struct timespec time1, struct timespec time2)
{
    struct timespec result;
    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC)
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    }
    else
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }
    return result;
}

void printElapsedTime(void)
{
    static struct timespec start;
    struct timespec curr;
    static int first_call = 1;
    int secs, nsecs;

    if (first_call)
    {
        first_call = 0;
        if (clock_gettime(CLOCK_MONOTONIC, &start) == -1)
            ;
        //handle_error("clock_gettime");
    }

    if (clock_gettime(CLOCK_MONOTONIC, &curr) == -1)
        ;
    //handle_error("clock_gettime");

    secs = curr.tv_sec - start.tv_sec;
    nsecs = curr.tv_nsec - start.tv_nsec;
    if (nsecs < 0)
    {
        secs--;
        nsecs += 1000000000;
    }
    //printf("%d.%03d: \n", secs, (nsecs + 500 000) / 1 000 000);
}
*/
