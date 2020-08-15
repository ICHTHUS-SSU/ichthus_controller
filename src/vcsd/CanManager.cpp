#include <stdio.h>
#include <string>
#include <map>
#include <cstring>
#include <iostream>
using namespace std;

extern void initHybridAutomata_can();
extern int operateHybridAutomata_can();
extern void add_rtParam(pair<string, string> p, char *name, char *nickname,
                   map<string, double> m, int value, int timestamp, int version, int property);
extern int set_rtParam(char *name, char* field, double val);

pair<string, string> Can_State_name;
map<string, double> Can_State;

pair<string, string> Wheel_Front_Right_Velocity_name;
map<string, double> Wheel_Front_Right_Velocity;
pair<string, string> Wheel_Rear_Left_Velocity_name;
map<string, double> Wheel_Rear_Left_Velocity;
pair<string, string> Wheel_Rear_Right_Velocity_name;
map<string, double> Wheel_Rear_Right_Velocity;
pair<string, string> Wheel_Front_Left_Velocity_name;
map<string, double> Wheel_Front_Left_Velocity;
pair<string, string> Lateral_Accel_Speed_name;
map<string, double> Lateral_Accel_Speed;
pair<string, string> AirConditioner_On_name;
map<string, double> AirConditioner_On;
pair<string, string> Steering_Angle_name;
map<string, double> Steering_Angle;
pair<string, string> Steering_Speed_name;
map<string, double> Steering_Speed;
pair<string, string> Steering_Tq_name;
map<string, double> Steering_Tq;
pair<string, string> Accel_Pedal_Position_name;
map<string, double> Accel_Pedal_Position;
pair<string, string> Brake_Active_name;
map<string, double> Brake_Active;
pair<string, string> BrakeMasterCylinder_Pressure_name;
map<string, double> BrakeMasterCylinder_Pressure;
pair<string, string> Engine_Speed_name;
map<string, double> Engine_Speed;
pair<string, string> Gear_Target_Change_name;
map<string, double> Gear_Target_Change;
pair<string, string> GearSelDisp_name;
map<string, double> GearSelDisp;
pair<string, string> Throttle_Position_name;
map<string, double> Throttle_Position;
pair<string, string> ACC_ACT_Status_name;
map<string, double> ACC_ACT_Status;
pair<string, string> Cluster_Odometer_name;
map<string, double> Cluster_Odometer;
pair<string, string> Longitudinal_Accel_Speed_name;
map<string, double> Longitudinal_Accel_Speed;
pair<string, string> Vehicle_Speed_Engine_name;
map<string, double> Vehicle_Speed_Engine;
pair<string, string> Yaw_Rate_Sensor_name;
map<string, double> Yaw_Rate_Sensor;
pair<string, string> AV_Main_SW_name;
map<string, double> AV_Main_SW;
pair<string, string> Driver_Override_name;
map<string, double> Driver_Override;
pair<string, string> AV_Control_State_name;
map<string, double> AV_Control_State;

pair<string, string> Current_Velocity_name;
map<string, double> Current_Velocity;

pair<string, string> Pub_2_Agent_name;
map<string, double> Pub_2_Agent;

enum {
    START,
    UP,
    ON,
    OFF,
    DOWN,
    FINISH
};

void initCanModule() {
    initHybridAutomata_can();
    
    add_rtParam(Can_State_name, "CAN.state", "CAN.state",Can_State, 0, 0, 0, 2);

    add_rtParam(Wheel_Front_Right_Velocity_name, "CAN.wheel_front_right_velocity", "CAN.wheel_front_right_velocity", Wheel_Front_Right_Velocity, 0, 0, 0, 2);
    add_rtParam(Wheel_Rear_Left_Velocity_name, "CAN.wheel_rear_left_velocity", "CAN.wheel_rear_left_velocity", Wheel_Rear_Left_Velocity, 0, 0, 0, 2);
    add_rtParam(Wheel_Rear_Right_Velocity_name, "CAN.wheel_rear_right_velocity","CAN.wheel_rear_right_velocity", Wheel_Rear_Right_Velocity, 0, 0, 0, 2);
    add_rtParam(Wheel_Front_Left_Velocity_name, "CAN.wheel_front_left_velocity", "CAN.wheel_front_left_velocity", Wheel_Front_Left_Velocity, 0, 0, 0, 2);
    add_rtParam(Lateral_Accel_Speed_name, "CAN.lateral_accel_speed", "CAN.lateral_accel_speed", Lateral_Accel_Speed, 0, 0, 0, 2);
    add_rtParam(AirConditioner_On_name, "CAN.airconditioner_on", "CAN.airconditioner_on", AirConditioner_On, 0, 0, 0, 2);
    add_rtParam(Steering_Angle_name, "CAN.steering_angle", "CAN.steering_angle", Steering_Angle, 0, 0, 0, 2);
    add_rtParam(Steering_Speed_name, "CAN.steering_speed", "CAN.steering_speed", Steering_Speed, 0, 0, 0, 2);
    add_rtParam(Steering_Tq_name, "CAN.steering_torque", "CAN.steering_torque", Steering_Tq, 0, 0, 0, 2);
    add_rtParam(Accel_Pedal_Position_name, "CAN.accel_pedal_position", "CAN.accel_pedal_position",Accel_Pedal_Position, 0, 0, 0, 2);
    add_rtParam(Brake_Active_name, "CAN.brake_active_state",  "CAN.brake_active_state",Brake_Active, 0, 0, 0, 2);
    add_rtParam(BrakeMasterCylinder_Pressure_name, "CAN.brake_mastercylinder_pressure", "CAN.brake_mastercylinder_pressure", BrakeMasterCylinder_Pressure,0, 0, 0, 2);
    add_rtParam(Engine_Speed_name, "CAN.engine_speed", "CAN.engine_speed", Engine_Speed, 0, 0, 0, 2);
    add_rtParam(Gear_Target_Change_name, "CAN.gear_stage", "CAN.gear_stage", Gear_Target_Change, 0, 0, 0, 2);
    add_rtParam(GearSelDisp_name, "CAN.gear_state","CAN.gear_state", GearSelDisp,  0, 0, 0, 2);
    add_rtParam(Throttle_Position_name, "CAN.throttle_position", "CAN.throttle_position", Throttle_Position, 0, 0, 0, 2);
    add_rtParam(ACC_ACT_Status_name, "CAN.ACC_ACT_state", "CAN.ACC_ACT_state", ACC_ACT_Status, 0, 0, 0, 2);
    add_rtParam(Cluster_Odometer_name, "CAN.cluster_odometer", "CAN.cluster_odometer",Cluster_Odometer, 0, 0, 0, 2);
    add_rtParam(Longitudinal_Accel_Speed_name, "CAN.longitudinal_accel_speed", "CAN.longitudinal_accel_speed",Longitudinal_Accel_Speed, 0, 0, 0, 2);
    add_rtParam(Vehicle_Speed_Engine_name, "CAN.vehicle_speed","CAN.vehicle_speed", Vehicle_Speed_Engine, 0, 0, 0, 2);
    add_rtParam(Yaw_Rate_Sensor_name, "CAN.yaw_rate_sensor", "CAN.yaw_rate_sensor", Yaw_Rate_Sensor, 0, 0, 0, 2);
    add_rtParam(AV_Main_SW_name, "CAN.AV_main_switch", "CAN.AV_main_switch", AV_Main_SW, 0, 0, 0, 2);
    add_rtParam(Driver_Override_name, "CAN.driver_override", "CAN.driver_override", Driver_Override, 0, 0, 0, 2);
    add_rtParam(AV_Control_State_name, "CAN.AV_control_state", "CAN.AV_control_state", AV_Control_State, 0, 0, 0, 2);

    add_rtParam(Current_Velocity_name, "CAN.actual_velocity", "can.avelo",Current_Velocity, 0, 0, 0, 2);
    add_rtParam(Pub_2_Agent_name, "CAN.publish_to_agent", "can.pub2agent",Pub_2_Agent, 0, 0, 0, 2);
}

int CanManageHandler(char* value) 
{
    int post_state = 0;
    static int timestamp = 0;

    if (strcmp(value, "up") == 0) {
        post_state = UP;
    }
    else if (strcmp(value, "on") == 0)
        post_state = ON;
    else if (strcmp(value, "off") == 0)
        post_state = OFF;
    else if (strcmp(value, "down") == 0)
        post_state = DOWN;
    else
        return -1;

    set_rtParam("CAN.state", "value", post_state);
    return operateHybridAutomata_can();
}