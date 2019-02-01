#ifndef CONFIG_H
#define CONFIG_H

//--------------PINS------------------

//------SERVOS--------
#define NUM_SERVOS  6

#define MG996R_MAX_ANGLE 120
#define MG90S_MAX_ANGLE 180
#define MG90S_MIN_ANGLE 0
#define MG996R_MIN_ANGLE 0

//Hip
#define	PIN_SERVO0	3
#define SERVO0MIN  95 // this is the 'minimum' pulse length count (out of 4096)
#define SERVO0MAX  484

//Waist
#define	PIN_SERVO1	5
#define SERVO1MIN  95 
#define SERVO1MAX  380

//Shoulder
#define	PIN_SERVO2	6
#define SERVO2MIN  95 
#define SERVO2MAX  460

//Elbow
#define MG90S_ELB_MIN_ANGLE 30
#define MG90S_ELB_MAX_ANGLE 120
#define	PIN_SERVO3	9
#define SERVO3MIN  165 
#define SERVO3MAX  375

//Wrist
#define	PIN_SERVO4	10
#define SERVO4MIN  95 
#define SERVO4MAX  440

//Gripper-rotation
#define	PIN_SERVO5	11
#define SERVO5MIN  95 
#define SERVO5MAX  484

//------RXEIVER--------
#define EI_ARDUINO_INTERRUPTED_PIN
#define NUM_RX_CHANNELS  6
#define RX_OFFSET   2
#define RX_AVG      1500

#define PIN_RX0     2
#define RX0_MIN  1148
#define RX0_MAX  1888

#define PIN_RX1     3
#define RX1_MIN  1136
#define RX1_MAX  1796

#define PIN_RX2     4
#define RX2_MIN  1156
#define RX2_MAX  1816

#define PIN_RX3     6
#define RX3_MIN  1160
#define RX3_MAX  1908

#define PIN_RX4     6
#define RX4_MIN  996
#define RX4_MAX  2012

#define PIN_RX5   7
#define RX5_MIN   996
#define RX5_MAX  2016


//------SERIAL_RX--------
#define NUM_CHARS   32
#endif