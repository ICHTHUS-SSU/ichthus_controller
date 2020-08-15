#define linux

#include <iostream>
#include <fstream>
#include <pthread.h>
#include <mutex>
#include <unistd.h>
#include <string.h>
#include "PCANBasic.h"
#include "CanBasic.h"
#include <math.h>
#include <syslog.h>
extern int get_rtParam_int(char *name, char *field);
extern int set_rtParam(char* name, char* field, double val);
extern double get_rtParam_double(char *name, char *field);
extern void write_log(char* buf);
extern void sendOdom(double param_val, int bparam);//bparam : { actual v = 0 } { steer ang = 1 }
extern double convert_kmh_to_mps(double tvel);
double actual_velocity;
double actual_angular_velocity;
struct GWAY1{
    // double test1 = NULL;
    // double test2 = NULL;
    // double test3 = NULL;
    // double test4 = NULL;
    // double test5 = NULL;

    double Gway_Wheel_Velocity_FR;
    double Gway_Wheel_Velocity_RL;
    double Gway_Wheel_Velocity_RR;
    double Gway_Wheel_Velocity_FL;
    
    void decode(__GWAY1_RAW__t* ptr){
        Gway_Wheel_Velocity_FR = (double) (ptr->Gway_Wheel_Velocity_FR * 0.03125);
        set_rtParam("CAN.wheel_front_right_velocity", "value", Gway_Wheel_Velocity_FR);
        Gway_Wheel_Velocity_RL = (double) (ptr->Gway_Wheel_Velocity_RL * 0.03125);
        set_rtParam("CAN.wheel_rear_left_velocity", "value", Gway_Wheel_Velocity_RL);
        Gway_Wheel_Velocity_RR = (double) (ptr->Gway_Wheel_Velocity_RR * 0.03125);
        set_rtParam("CAN.wheel_rear_right_velocity", "value", Gway_Wheel_Velocity_RR);
        Gway_Wheel_Velocity_FL = (double) (ptr->Gway_Wheel_Velocity_FL * 0.03125);
        set_rtParam("CAN.wheel_front_left_velocity", "value", Gway_Wheel_Velocity_FL);
        
        // test1 = get_rtParam_double("CAN.wheel_front_right_velocity", "value");
        // test2 = get_rtParam_double("CAN.wheel_rear_left_velocity", "value");
        // test3 = get_rtParam_double("CAN.wheel_rear_right_velocity", "value");
        // test4 = get_rtParam_double("CAN.wheel_front_left_velocity", "value");
        // fprintf(stdout, "FR wheel speed : %lf\n", test1);
        // fprintf(stdout, "RL wheel speed : %lf\n", test2);
        // fprintf(stdout, "RR wheel speed : %lf\n", test3);
        // fprintf(stdout, "FL wheel speed : %lf\n", test4);
	    actual_velocity = (Gway_Wheel_Velocity_RL + Gway_Wheel_Velocity_RR) / 2.0;
        set_rtParam("CAN.actual_velocity", "value", actual_velocity);
	    //printf("actual_velocity : %lf\n",actual_velocity);
        if(get_rtParam_int("CAN.publish_to_agent","value") == 1 )
        {
	    //syslog(LOG_ERR,"actual_velocity : %lf\n",actual_velocity);
	    //printf("actual_velocity : %lf\n",actual_velocity);
            sendOdom(convert_kmh_to_mps(actual_velocity),0);
            sendOdom(convert_kmh_to_mps(Gway_Wheel_Velocity_RL),4);
            sendOdom(convert_kmh_to_mps(Gway_Wheel_Velocity_RR),5);
        }
        // test5 = get_rtParam_double("CAN.actual_velocity", "value");
        // fprintf(stdout, "FL wheel speed : %lf\n", test5);

    }
};
struct GWAY2{
    int         Gway_Lateral_Accel_Speed       ;
    int         Gway_AirConditioner_On         ;
    double      Gway_Steering_Angle            ;
    unsigned    Gway_Steering_Speed            ;
    int         Gway_Steering_Tq               ;
    void decode(__GWAY2_RAW__t* ptr){
        Gway_Lateral_Accel_Speed    = (int) (ptr->Gway_Lateral_Accel_Speed * 0.01 - 10.23);
        set_rtParam("CAN.lateral_accel_speed", "value", Gway_Lateral_Accel_Speed);
        Gway_AirConditioner_On      = (int) (ptr->Gway_AirConditioner_On);
        set_rtParam("CAN.airconditioner_on", "value", Gway_AirConditioner_On);
        // steering angle manual is wrong. this is right.
        Gway_Steering_Angle         = (double) (ptr->Gway_Steering_Angle);
        set_rtParam("CAN.steering_angle", "value", Gway_Steering_Angle);
        if(get_rtParam_int("CAN.publish_to_agent","value") == 1 )
        {
        //     actual_angular_velocity = actual_velocity * tan(Gway_Steering_Angle / 950.0) / (double)get_cfParam("SteerControl.wheel_base");
	    // //syslog(LOG_ERR,"actual_angular_velocity : %lf\n",actual_angular_velocity);
	    // //printf("actual_angular_velocity : %lf\n",actual_angular_velocity);
            sendOdom(Gway_Steering_Angle, 1);
        }
        Gway_Steering_Speed         = (double) (ptr->Gway_Steering_Speed * 4);
        set_rtParam("CAN.steering_speed", "value", Gway_Steering_Speed);
        Gway_Steering_Tq            = (int) ((ptr->Gway_Steering_Tq - 0x800) * 0.01);
        set_rtParam("CAN.steering_torque", "value", Gway_Steering_Tq);
    }
};

struct GWAY3{
    int     Gway_Accel_Pedal_Position           ;
    int     Gway_Brake_Active                   ;
    int     Gway_BrakeMasterCylinder_Pressure   ;
    int16_t     Gway_Engine_Speed               ;
    int     Gway_Gear_Target_Change             ;
    int     Gway_GearSelDisp                    ;
    int     Gway_Throttle_Position              ;
    int     Gway_ACC_ACT_Status                 ;
    void decode(__GWAY3_RAW__t* ptr){
        Gway_Accel_Pedal_Position          = (int) (ptr->Gway_Accel_Pedal_Position * 0.3906);
        set_rtParam("CAN.accel_pedal_position", "value", Gway_Accel_Pedal_Position);
        Gway_Brake_Active                  = (int) (ptr->Gway_Brake_Active);
        set_rtParam("CAN.brake_active_state", "value", Gway_Brake_Active); 
        Gway_BrakeMasterCylinder_Pressure  = (int) (ptr->Gway_BrakeMasterCylinder_Pressure * 0.1);
        set_rtParam("CAN.brake_mastercylinder_pressure", "value", Gway_BrakeMasterCylinder_Pressure); 
        Gway_Engine_Speed                  = (int) (ptr->Gway_Engine_Speed * 0.25);
        set_rtParam("CAN.engine_speed", "value", Gway_Engine_Speed);
        Gway_Gear_Target_Change            = (int) (ptr->Gway_Gear_Target_Change);
        set_rtParam("CAN.gear_stage", "value", Gway_Gear_Target_Change); 
        Gway_GearSelDisp                   = (int) (ptr->Gway_GearSelDisp);
        set_rtParam("CAN.gear_state", "value", Gway_GearSelDisp); 
        Gway_Throttle_Position             = (int) ((ptr->Gway_Throttle_Position - 0x20) * 100 / 213);
        set_rtParam("CAN.throttle_position", "value", Gway_Throttle_Position); 
        Gway_ACC_ACT_Status                = (int) (ptr->Gway_ACC_ACT_Status);
        set_rtParam("CAN.ACC_ACT_state", "value", Gway_ACC_ACT_Status);
    }
};

struct GWAY4{
    int Gway_Cluster_Odometer               ;
    int Gway_Longitudinal_Accel_Speed       ;
    int Gway_Vehicle_Speed_Engine           ;
    double Gway_Yaw_Rate_Sensor             ;
    void decode(__GWAY4_RAW__t* ptr){
        Gway_Cluster_Odometer           = (int) (ptr->Gway_Cluster_Odometer * 0.1);
        set_rtParam("CAN.cluster_odometer", "value", Gway_Cluster_Odometer);
        Gway_Longitudinal_Accel_Speed   = (int) (ptr->Gway_Longitudinal_Accel_Speed * 0.01 - 10.23);
        set_rtParam("CAN.longitudinal_accel_speed", "value", Gway_Longitudinal_Accel_Speed);
        Gway_Vehicle_Speed_Engine       = (int) (ptr->Gway_Vehicle_Speed_Engine * 1);
        set_rtParam("CAN.vehicle_speed", "value", Gway_Vehicle_Speed_Engine);
        Gway_Yaw_Rate_Sensor            = (double) (ptr->Gway_Yaw_Rate_Sensor * 0.01 - 40.95);
        set_rtParam("CAN.yaw_rate_sensor", "value", Gway_Yaw_Rate_Sensor);
    }
};

struct GWAY10{
    bool Gway_AV_Main_SW                          ; 
    unsigned int Gway_Driver_Override             ;
    unsigned int Gway_AV_Control_State            ;
    void decode(__GWAY10_RAW__t* ptr){
        Gway_AV_Main_SW         = (bool) ptr->Gway_AV_Main_SW;
        set_rtParam("CAN.AV_main_switch", "value", Gway_AV_Main_SW);
        Gway_Driver_Override    = (unsigned int) ptr->Gway_Driver_Override;
        set_rtParam("CAN.driver_override", "value", Gway_Driver_Override);
        Gway_AV_Control_State   = (unsigned int) ptr->Gway_AV_Control_State;
        set_rtParam("CAN.AV_control_state", "value", Gway_AV_Control_State);
    }
};

GWAY1 gway1;
GWAY2 gway2;
GWAY3 gway3;
GWAY4 gway4;
GWAY10 gway10;

// PCAN library data type
TPCANMsg Message;
TPCANStatus Status;

extern pthread_mutex_t glob_mutex;
pthread_t can_cyclic_thread;

int canUp() {
    fprintf(stdout, "CAN_UP called\n");
    Status = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_500K, 0, 0, 0);
    fprintf(stdout, "Initialize CAN: %i\n", (int)Status);
    return 1;
}

void* canCyclic(void* name) {
    constexpr static int LEN_GWAY_DATA = 8;
    uint8_t gway_buf[LEN_GWAY_DATA] = "";    
    int id_msg;

    while(1) {
        while ((Status = CAN_Read(PCAN_USBBUS1, &Message, NULL)) == PCAN_ERROR_QRCVEMPTY) {
            usleep(5000);
        }

        if (Status != PCAN_ERROR_OK) {
            fprintf(stdout, "Error 0x%x\n", (int)Status);
            break;
        }

        for(int i = 0 ; i < LEN_GWAY_DATA; ++i){
            gway_buf[i] = Message.DATA[i];
        }

        switch((unsigned int)Message.ID){
        case 0x100:
            gway1.decode((__GWAY1_RAW__*)gway_buf); break;
        case 0x101:
            gway2.decode((__GWAY2_RAW__*)gway_buf); break;
        case 0x102:
            gway3.decode((__GWAY3_RAW__*)gway_buf); break;
        case 0x103:
            gway4.decode((__GWAY4_RAW__*)gway_buf); break;
        case 0x110:
            gway10.decode((__GWAY10_RAW__*)gway_buf); break;
        default:
            fprintf(stdout, "gateway output error - invalid id : %d", id_msg);
            break;
        }
    }
}

int canOn() {
    fprintf(stdout, "CAN_ON called\n");
    if (pthread_create(&can_cyclic_thread, 0, canCyclic, NULL)) {
        fprintf(stdout, "CAN thread err\n");
    }
    return 1;
}

int canOff() {
    fprintf(stdout, "CAN_OFF called\n");
    pthread_cancel(can_cyclic_thread);
    pthread_join(can_cyclic_thread, NULL);
    return 1;
}

int canDown() {
    fprintf(stdout, "CAN_DOWN called\n");
    return 1;
}
