# Ichthus Controller

**Ichthus Controller** is a ROS package that controls a real vehicle driven by servo motors. It assumes that throttle pedal, brake pedal, steering wheel, and even gearstick are controlled by servo motors, which are based on EtherCAT field bus. This way we do not have to connect the Automotive Electronics through CAN gateway. In our vehicle, we use four **Epos3 DC Servo Motors from Maxon Motor** for those control knobs, each of which is attached to a motor drive controlling the current flowing into the motor. The next figures show how we attach the servo motor to the relavant component of our vehicle.

**Throttle Pedal**
![pedal_motor](https://user-images.githubusercontent.com/40734644/88827049-6b4afc80-d204-11ea-9430-f495e1d1217d.png)

**Brake Pedal**
![pedal_motor](https://user-images.githubusercontent.com/40734644/88827049-6b4afc80-d204-11ea-9430-f495e1d1217d.png)

**Steering Wheel**
![pedal_motor](https://user-images.githubusercontent.com/40734644/88827049-6b4afc80-d204-11ea-9430-f495e1d1217d.png)

**Gearstick**
![pedal_motor](https://user-images.githubusercontent.com/40734644/88827049-6b4afc80-d204-11ea-9430-f495e1d1217d.png)

**Motor Drive**
![motor_drive](https://user-images.githubusercontent.com/40734644/88827060-6e45ed00-d204-11ea-828d-f1888bb2dea6.png)

**Ichthus Controller** consists of two ROS nodes: one is a ROS client that interacts between the end user and a ROS service, and the other is a ROS service that interfaces between the ROS client and the hardware. It provides low-level functions to control each motor in position mode and also high-level controllers such as cruise control (= speed control) and steering control. For cruise control, it requres measurement of real speed of the target vehicle, and for this purpose, we may use CAN gateway or OBD2.

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
|`ichthus_command.yaml`|command and macro definitions used by the ichthus controller client in the user terminal |
|`ichthus_i30.yaml`|state variable definitions used by the ichthus controller server for our vehicle based on Hyundai i30|

**msg**
|File|Description|
-------|--------
|`can_msg.msg`|message definition to interface with ROS node reading the current vehicle speed from CAN gateway|
|`obd_msg.msg`|message definition to interface with ROS node reading the current vehicle speed from OBD|
>You can add your message type here.


**srv**
|File|Description|
-------|--------
|`con_msg.srv`|msg definition for client - server communication|

**src**
|File|Description|
-------|--------
|`ichthus_controller_client.cpp`|source file of the client program|
|`ichthus_controller_server_core.cpp`|source file of the server program that interprets client commands|
|`ichthus_controller_server_node.cpp`|source file of the wrapper of the server program|
|`controllers.cpp`|source file of high-level controllers such as cruise control and emergency stop|
|`ethercat.cpp`|source file of low-level functions that controls the position of each motor|


## IgH EtherCAT Master Kernel Module Installation
[EtherCAT Kernel Module Installation](https://sourceforge.net/p/etherlabmaster/code/ci/stable-1.5/tree/)

## EtherCAT-based Motor Control

(**Ethernet for Control Automation Technology**) is an [Ethernet](https://en.wikipedia.org/wiki/Ethernet "Ethernet")-based [fieldbus](https://en.wikipedia.org/wiki/Fieldbus "Fieldbus") system, invented by Beckhoff Automation. The protocol is standardized in [IEC 61158](https://en.wikipedia.org/wiki/IEC_61158 "IEC 61158") and is suitable for both hard and soft [real-time computing](https://en.wikipedia.org/wiki/Real-time_computing "Real-time computing") requirements in automation technology.

- **EtherCAT** guaranteeing the communication delay in microsecond unit.

- **EtherCAT** bus has a **Master-Slave** structure, where the **Main Computer** becomes the **Master** and a **Motor Drive** becomes a **Slave**. Only the master  can initiate message transmission, and the slaves transmit data only through piggybacking to the message initiated by the master.

### EtherCAT State Machine

In our server program, we have implemented a state machine that manages the lifecycle of EtherCAT field bus and individual motor drives. The states used are derived from the EtherCAT standard. You may refer to EPOS3-EtherCAT-Firmware-Specification-En.pdf.

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

In the state of Operation Enable, the server transmits every 10 ms a target position to each motor, maintained in a state variable, whether it is updated or not. Just in case, if we need to override the server control for manually driving the vehicle, e.g., to manually control the steering wheel, we simply turn the motor state from Operation Enable to Disable Operation only for the motor of the steering wheel, instead of shutting down all the motors and the EtherCAT field bus. Later, when we need to recover the server control, we simply turn the motor state from Disable Operation to Operation Enable without affacting the other motors in Operation Enable.

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

## High-level Controllers (Motion Control)
Ichthus Controller has several **high-level controller functions**. They implement their own control logic and issue the target position(s) of one or more motors.

|Motion|Description|
-------|--------
|`initial`|initial state of high-level controller|
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

> There are many macros to replace the basic commands. **check ichthus_command.yaml**
> You can add your own macro to this file
> The following steps are used commonly.

### steps to activate the cruise control
1. `roslaunch ichthus_controller ichthus_controller.launch`
2. `up`
3. `on`
4. `standby`
5. `cruise`
6. `standby`

### steps to activate auto-driving (including both cruise control and steering control)
1. `roslaunch ichthus_controller ichthus_controller.launch`
2. `up`
3. `on`
4. `standby`
5. `cal`
6. `auto`
7. `standby`

### steps to shutdown the EtherCAT field bus and all motor drives simultaneously
- `set ecat_state 1` *(off)*
- `set ecat_state 0` *(down)*
 

----
