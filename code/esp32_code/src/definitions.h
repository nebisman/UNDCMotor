// Definitions for the DC motor system
// Created by LB on 1/18/24.
#include <stdlib.h>
//

// This is the pwm configuration for the DC motor system
#define  FREQUENCY_PWM     100
#define  RESOLUTION_PWM    12
#define  PIN_AIN1          2
#define  PIN_AIN2          4
#define  CH_PWM_AIN1       0
#define  CH_PWM_AIN2       1


//// This is the configuration of the encoder for sensing position of DC motor

#define CH_ENC_A    25 // Channel A of the motor encoder
#define CH_ENC_B    26 // Channel B of the motor encoder






//
//// This is the configuration for the ws2812 rgb leds that show the temperature and reference
//#define  LIGHT_PIN         19
//#define  NUMPIXELS          2
//
// Hardware definitions
#define CORE_CONTROL 1   // Core 0 is used for control tasks
#define CORE_COMM    0   // Core 1 is used for mqtt communication
//
//
//// This is the pin for the Dallas 18b20 temperature sensor
//#define ONE_WIRE_BUS 4
//
//// System definitions
//#define DEFAULT_REFERENCE 40
//
//
//// IOT topic definitions
//// Topics published by the thermal system
//#define SYS_USER_SIGNALS_CLOSED       "/thermal/thermal_" PLANT_NUMBER "/user/sig_closed"
//#define SYS_USER_SIGNALS_OPEN       "/thermal/thermal_" PLANT_NUMBER "/user/sig_open"
//// Topics received from the user
//#define USER_SYS_SET_PID           "/thermal/user/thermal_" PLANT_NUMBER "/set_pid"             //1
//#define USER_SYS_SET_REF           "/thermal/user/thermal_" PLANT_NUMBER "/set_ref"             //2
//#define USER_SYS_STEP_CLOSED       "/thermal/user/thermal_" PLANT_NUMBER "/step_closed"         //3
//#define USER_SYS_STAIRS_CLOSED     "/thermal/user/thermal_" PLANT_NUMBER "/stairs_closed"       //4
//#define USER_SYS_PRBS_OPEN         "/thermal/user/thermal_" PLANT_NUMBER "/prbs_open"           //5
//#define USER_SYS_STEP_OPEN         "/thermal/user/thermal_" PLANT_NUMBER "/step_open"           //6
//#define USER_SYS_STEP_OPEN         "/thermal/user/thermal_" PLANT_NUMBER "/step_open"           //6
//#define USER_SYS_SET_GENCON        "/thermal/user/thermal_" PLANT_NUMBER "/set_gencon"          //7
//
//// Integer definitions of topics to avoid comparison with strings, which is more expensive
//// in terms of computation
//#define DEFAULT_TOPIC                  0
//#define USER_SYS_SET_PID_INT           1
//#define USER_SYS_SET_REF_INT           2
//#define USER_SYS_STEP_CLOSED_INT       3
//#define USER_SYS_STAIRS_CLOSED_INT     4
//#define USER_SYS_PRBS_OPEN_INT         5
//#define USER_SYS_STEP_OPEN_INT         6
//#define USER_SYS_SET_GENCON_INT        7
//
//// Codes for modes of control
//#define GENERAL_CONTROLLER             0
//#define PID_CONTROLLER                 1
//
///* This matrix is the gamma correction for the leds attached to the plant, which allow to "see" the current
//   temperature and the current reference
//*/
//const uint8_t gamma8[] = {
//        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
//        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
//        1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
//        2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
//        5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
//        10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
//        17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
//        25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
//        37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
//        51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
//        69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
//        90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
//        115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
//        144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
//        177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
//        215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };
//
////  percent of control signal in the operation point
const float percent2pwm = (float) (4095.0/5.0);
const float pulses2degrees = (float) (360.0/600.0);