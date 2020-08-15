#include <stdint.h>

typedef struct __GWAY1_RAW__{ 
    uint16_t     Gway_Wheel_Velocity_FR              : 14    ;
    int          no_use1                             : 2     ;
    uint16_t     Gway_Wheel_Velocity_RL              : 14    ;
    int          no_use2                             : 2     ;
    uint16_t     Gway_Wheel_Velocity_RR              : 14    ;
    int          no_use3                             : 2     ; //63kph
    uint16_t     Gway_Wheel_Velocity_FL              : 14    ;
    int          no_use4                             : 2     ;
}__attribute__((packed)) __GWAY1_RAW__t;        
typedef struct __GWAY2_RAW__{        
    int16_t      Gway_Lateral_Accel_Speed            : 16    ; //0.01 - 10.23(m/s^2)
    int          no_use1                             : 4     ;    
    int          Gway_AirConditioner_On              : 4     ;
    int16_t      Gway_Steering_Angle                 : 16    ;//0.1 * value(Deg)
    int8_t       Gway_Steering_Speed                 : 8     ;
    int16_t      Gway_Steering_Tq                    : 16    ;
}__attribute__((packed)) __GWAY2_RAW__t;        
typedef struct __GWAY3_RAW__{        
    int8_t       Gway_Accel_Pedal_Position           : 8     ;
    int          Gway_Brake_Active                   : 4     ;
    int16_t      Gway_BrakeMasterCylinder_Pressure   : 16    ;
    int16_t      Gway_Engine_Speed                   : 16    ;
    int          Gway_Gear_Target_Change             : 4     ;
    int          Gway_GearSelDisp                    : 4     ;
    int8_t       Gway_Throttle_Position              : 8     ;
    int          Gway_ACC_ACT_Status                 : 4     ;
}__attribute__((packed)) __GWAY3_RAW__t;        
typedef struct __GWAY4_RAW__{        
    int          Gway_Cluster_Odometer               : 24    ;
    int16_t      Gway_Longitudinal_Accel_Speed       : 16    ;
    int8_t       Gway_Vehicle_Speed_Engine           : 8     ;
    int16_t      Gway_Yaw_Rate_Sensor                : 16    ;
}__attribute__((packed)) __GWAY4_RAW__t;        
typedef struct __GWAY5_RAW__{        
    int8_t       Dummy0_01H                          : 8     ;
    int8_t       Dummy1_23H                          : 8     ;
    int8_t       Dummy2_45H                          : 8     ;
    int8_t       Dummy3_67H                          : 8     ;
    int8_t       Dummy4_89H                          : 8     ;
    int8_t       Dummy5_ABH                          : 8     ;
    int8_t       Dummy6_CDH                          : 8     ;
    int8_t       Dummy7_EFH                          : 8     ;
}__attribute__((packed)) __GWAY5_RAW__t;        
typedef struct __GWAY10_RAW__{        
    int          Gway_AV_Main_SW                     : 1     ; 
    int          no_use1                             : 7     ;
    unsigned int Gway_Driver_Override                : 2     ;
    int          no_use2                             : 6     ;
    unsigned int Gway_AV_Control_State               : 4     ;
    int64_t      no_use3                             : 44    ;
}__attribute__((packed)) __GWAY10_RAW__t;        
typedef struct __AVC10_RAW{                                    //write       
    unsigned     AVC_Start                           : 1     ;
    int          no_use1                             : 7     ;
    unsigned     AVC_StopReq                         : 1     ;
    int          no_use2                             : 7     ;
    unsigned     AVC_aReqMax                         : 11    ;
    int64_t      no_use3                             : 37    ;
}__attribute__((packed)) __AVC10_RAW__t;       