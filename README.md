# Ichthus Controller

**Ichthus Controller** is a Vehicle Controller used in Soongsil University's Ichthus vehicle for servo motor based vehicle control written in ROS node. This is a method of controlling the motor by attaching the **Epos3 DC Motor servo motor of Maxon Motor** to the vehicle's accelerator pedal, deceleration pedal, steering wheel, and gear stick, and the motor driver supplies current to the motor. Networking between motor drivers is required to control the motor, and for this purpose, **EtherCAT,** an industrial real-time field bus, is used.

**Pedal Motor**
![pedal_motor](https://user-images.githubusercontent.com/40734644/88827049-6b4afc80-d204-11ea-9430-f495e1d1217d.png)

**Motor Drive**
![motor_drive](https://user-images.githubusercontent.com/40734644/88827060-6e45ed00-d204-11ea-828d-f1888bb2dea6.png)

## Overview
- [Files](#files)
- [IgH  EtherCAT Master Kernel Module Installation](#igh-ethercat-master-kernel-module-installation)
- [EtherCAT-based Motor Control](#ethercat-based-motor-control)
- [Client - Server Architecture using ROS Service](#client---server-architecture-using-ros-service)
- [State Variables Management](#state-variables-management)
- [Command Line Interface](#command-line-interface)
- [Motion Control](#motion-control)
- [Usage](#usage)

## Files
**cfg**
|File|Description|
-------|--------
|`ichthus_command.yaml`|command and macro definition|
|`ichthus_i30.yaml`|state variable definition for i30 vehicle|

**msg**
|File|Description|
-------|--------
|`can_msg.msg`|msg definition for CAN|
|`obd_msg.msg`|msg definition for OBD|
>You can add your message type here.


**srv**
|File|Description|
-------|--------
|`con_msg.srv`|msg definition for client - server service communication|

**src**
|File|Description|
-------|--------
|`ichthus_controller_client.cpp`|service client for command line interface|
|`ichthus_controller_server_core.cpp`|service server, manage all state variables|
|`ichthus_controller_server_node.cpp`|ichthus_controller node|
|`controllers.cpp`|motion control, decide motor target position|
|`ethercat.cpp`|manage motor drive and EtherCAT, communicate with motor drive |


## IgH EtherCAT Master Kernel Module Installation
[EtherCAT Kernel Module Installation](https://sourceforge.net/p/etherlabmaster/code/ci/stable-1.5/tree/)

## EtherCAT-based Motor Control

(**Ethernet for Control Automation Technology**) is an [Ethernet](https://en.wikipedia.org/wiki/Ethernet "Ethernet")-based [fieldbus](https://en.wikipedia.org/wiki/Fieldbus "Fieldbus") system, invented by Beckhoff Automation. The protocol is standardized in [IEC 61158](https://en.wikipedia.org/wiki/IEC_61158 "IEC 61158") and is suitable for both hard and soft [real-time computing](https://en.wikipedia.org/wiki/Real-time_computing "Real-time computing") requirements in automation technology.

- **EtherCAT** guaranteeing the communication delay in microsecond unit.

- **EtherCAT** bus has a **Master-Slave** structure, where the **PC** becomes the **Master** and the **Motor Driver** becomes the **Slave**. Only the Master node can initiate message transmission, and the Slave node transmits data only through piggybacking to the message initiated by the Mastser node

### EtherCAT State Machine
#### offical manual  : [EPOS3-EtherCAT-Firmware-Specification-En.pdf](https://www.maxongroup.com/medias/sys_master/root/8834322825246/EPOS3-EtherCAT-Firmware-Specification-En.pdf)

#### EtherCAT device state
|State short name |State full name|
-------|--------
|`NSO`|Not ready to Switch On|
|`SOD`|Switch On Disable|
|`RSO`|Ready to Swtich On|
|`SON`|Switched On|
|**`OEN`**|**Operation Enable** (goal state)|
|`QSA`|Quick Stop Active|
|`FLT`|Fault|

#### EtherCAT state transition command
|command short name |command full name|
-------|--------
|`DVO`|Disable Voltage|
|`RST`|Fault Reset|
|`SHD`|Shut Down|
|`SON`|Switch On|
|`ENO`|Enable Operation|
|`DSO`|Disable Operation|
|`QST`|Quick Stop|

#### EtherCAT state transition matrix
![EtherCAT State Transition Matrix](https://user-images.githubusercontent.com/40734644/88839865-99393c80-d216-11ea-8af2-7054cfc3e6bf.png)




**Using EtherCAT with stable state machine, Ichthus Controller transmits the target position of the Motor every 10ms  to the Slave to control the Motor. and, if it is not a physical problem, all motors always reach operation enable and normal state recovery is possible without unnecessary power on/off.**
## Client - Server Architecture using ROS Service
Ichthus Controller operates in the **ROS Service**. Client sends a command to the Server in the form of ROS Service, Server performs the operation corresponding to the command.

## State Variables Management
The **State Variable** means all variables managed by the set/get command in the Ichthus Controller.
* The ichthus controller can **get** or **set** state variables.
* **State Variables** related to **motor position** have fields of **minimum, upper, origin, and increment**, and other **State Variables** can use this field in different meanings.
* Also, some **State Variables** associated with the state machine must follow the state machine's state transition graph.

#### ichthus_i30.yaml
``` 

pedal_accel_pos : { lower: 0, upper: 33000, origin: 0 }

pedal_decel_pos : { lower: 0, upper: 110000, origin: 20000 }

steer_wheel_pos : { lower: -225000, upper: 225000, origin: 0 }

gear_stick_pos : { lower: 0, upper: 2450000, origin: 0 } # lower = park, upper = drive

#gear_stick_pos : { park: 0, reverse: 1100000, neutral: 1900000, drive: 2450000 }

pedal_accel_vpos : { lower: 0, upper: 100 } # origin to be calculated from pedal_accel_pos/origin

pedal_decel_vpos : { lower: 0, upper: 100 } # origin to be calculated from pedal_decel_pos/origin

steer_wheel_vpos : { lower: -450, upper: 450 } # origin to be calculated from steer_wheel_pos/origin

gear_stick_vpos : { lower: 0, upper: 100 } # origin to be calculated from gear_stick_pos/origin

pedal_accel_apos : { step: 660 } # used as margin to check if apos is within (pos-margin, pos+margin)

pedal_decel_apos : { step: 3300 } # used as margin to check if apos is within (pos-margin, pos+margin)

steer_wheel_apos : { step: 50} # used as margin to check if apos is within (pos-margin, pos+margin)

gear_stick_apos : { step: 24500 } # used as margin to check if apos is within (pos-margin, pos+margin)

motion_pull_over : { step: 500, jerk: -0.3 }

cc_gains_accel_obd : { Kp: 600, Ki: 25, Kd: 300 }

cc_gains_decel_obd : { Kp: 3000, Ki: 200, Kd: 1000 }

# cc_gains_accel_can : { Kp: 1600, Ki: 4, Kd: 300 }

# cc_gains_decel_can : { Kp: 1900, Ki: 2, Kd: 600 }

cc_gains_accel_can : { Kp: 1400, Ki: 4, Kd: 300 }

cc_gains_decel_can : { Kp: 1600, Ki: 2, Kd: 600 }

cc_switch_margins : { lower: 50, upper: 120 }

cc_target_velocity : { lower: -2, upper: 100 }

cc_integral_base : { origin: 5000, step: 1000 }

sc_vehicle_geometry : { turning_radius : 5.30, wheel_base: 2.65, pos_per_deg: 860 }
```
### Motor Position State Variables
	 pos  : physical target position
	 vpos : virtual target position
	 apos : physical actual position 
	 ex)  : pedal_accel_pos, pedal_accel_vpos, pedal_accel_apos
### Motor Drive State Variables
##### ecat_state
	ecat_state == 0 : master down, slave deactivated -> activated
	ecat_state == 1 : master up, all slave up (all motor drives activated but not enabled)
	ecat_state == 2 : all motor drives enabled
##### pedal_accel_state,  pedal_decel_state,  steer_wheel_state . . . . 
	xxx_state == 0 : motor drive deactivated
	xxx_state == 1 : motor drive activated, but not enabled
	xxx_state == 2 : motor drive enabled
### Motion State Variables
	motion_state == 0 : initial
	motion_state == 1 : standby
	motion_state == 2 : homing
	motion_state == 3 : estop
	motion_state == 4 : calibrate steer wheel
	motion_state == 5 : pull over
	motion_state == 6 : self test
	motion_state == 7 : cruise control
	motion_state == 8 : self-driving
	

## Command Line Interface

The Ichthus Controller works by read or write the **State Variable** by sending commands in the form of **ROS Service** in the form of **basic_fmt** or **macro_fmt** that allows the client to easily type  multiple **basic_fmt** sets

#### basic_fmt  : basic form of command format expressed in regular expression
	- get: command used to get the value set in the status variable
	- set: command used to set the value in the status variable
	- help, quit, history: other utility commands
	
	
```
basic_fmt0: "^(get) ([a-zA-Z0-9_.]{1,40})$" # e.g., get var

basic_fmt1: "^(get) ([a-zA-Z0-9_.]{1,40}) ([a-zA-Z0-9_]{1,40})$" # e.g., get var timestamp

basic_fmt2: "^(set) ([a-zA-Z0-9_.]{1,40}) ([a-zA-Z0-9_.-]{1,19})$" # e.g., set var -123.456

basic_fmt3: "^(help)$"

basic_fmt4: "^(quit)$"

basic_fmt5: "^(history)$"

basic_fmt6: "^(!)([0-9]{1,3})$"
```
#### macro_fmt : macros designed to easily type one or more command sets
#### macro_def : definition of macro written to satisfy basic_fmt
```
macro_fmt6: "^(up)$" # ecat up

macro_def6: [ "set ecat_state 1" ]

macro_fmt7: "^(on)$" # ecat on

macro_def7: [ "set ecat_state 2" ]

macro_fmt8: "^(off)$" # ecat off

macro_def8: [ "set ecat_state 1" ]

macro_fmt9: "^(down)$" # ecat down

macro_def9: [ "set ecat_state 0" ]

macro_fmt10: "^(act)$" # activate motor

macro_def10: [ "set steer_wheel_state 1" ]

macro_fmt11: "^(en)$" # enable motor

macro_def11: [ "set steer_wheel_state 2" ]

macro_fmt12: "^(dis)$" # disable motor

macro_def12: [ "set steer_wheel_state 1" ]

macro_fmt13: "^(de)$" # deactivate motor

macro_def13: [ "set steer_wheel_state 0" ]
```

#### command line example
	#basic_fmt2: "^(set) ([a-zA-Z0-9_.]{1,40}) ([a-zA-Z0-9_.-]{1,19})$"
	###comand  :	set      pedal_accel_pos      100

## Motion Control
Ichthus Controller has various **motion control functions** . The result of the motion control function is the **target position value of the motor**
|Motion|Description|
-------|--------
|`initial`|initial state of motion|
|`standby`|after initializing, all motors are set at orig and waiting for transition to other motion|
|`homing`|set accel pedal motor and decel pedal motor at lower position|
|`estop`|set decel pedal motor at upper position|
|`calibrate_steer_wheel`|set steer wheel motor at orig position|
|`pull_over`|increasing the decel pedal gradually, set the deceleration pedal in the upper position|
|`self_test`|set all motors at lower - higher - orig sequentially for motor test|
|`cruise`|adjust the pedal to converge to the target velocity|
|`auto`|self driving! control accel, decel, steer motor. it requires target veloicty and target steering wheel angle|

## Usage
### initializing EtherCAT and Motor Drive
- 	`roslaunch ichthus_controller ichthus_controller.launch`
-	`set ecat_state 1` *(up)*
-	`set ecat_state 2` *(on)*

### get state variables
- `get pedal_accel_pos` 
- `get cc_target_velocity`
 - `get varlist all` *(show)*

### set state variables
- `set pedal_accel_pos 1000`
- `set ecat_state 1` *(up)*
- `set cc_target_velocity 20` *(20)*
- `set motion_state 2` *(on)*

### others
- `help`
- `history`
- `!(number)`

> There are many macro for easy typing. **check ichthus_command.yaml**
> You can add your own macro in this file
> 
### best way for cruise control
1. `roslaunch ichthus_controller ichthus_controller.launch`
2. `up`
3. `on`
4. `standby`
5. `cruise`
6. `standby`

### best way for self driving
1. `roslaunch ichthus_controller ichthus_controller.launch`
2. `up`
3. `on`
4. `standby`
5. `cal`
6. `auto`
7. `standby`

### uninitializing EtherCAT and Motor Drive
- `set ecat_state 1` *(off)*
- `set ecat_state 0` *(down)*
 

----
