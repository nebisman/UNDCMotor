// Definitions for the DC motor system
// Created by LB on 1/18/24.
#include <stdlib.h>
#include <ESP32Encoder.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <Adafruit_NeoPixel.h>
#include <OneWire.h>


// This is the pwm configuration for the DC motor system
#define  FREQUENCY_PWM     100
#define  RESOLUTION_PWM    12
#define  PIN_AIN1          2
#define  PIN_AIN2          1 // WEMOS 4  // ESP32S3 1
#define  CH_PWM_AIN1       0
#define  CH_PWM_AIN2       1



// This is the configuration of the encoder for sensing position of DC motor

#define CH_ENC_A  18//  WEMOS 26  ESP32S3 18     Channel A of the motor encoder
#define CH_ENC_B  17 // WEMOS 25  ESP32S3 17     Channel B of the motor encoder

// This is the configuration for the potentiometer-encoder

#define CH_ENC_A_POT  11  //WEMOS 5// ESP32S3 11//
#define CH_ENC_B_POT  10 // WEMOS 23 // ESP32S3 10//


// This is the configuration for the ws2812 rgb leds that show the temperature and reference
#define  LIGHT_PIN         19  // WEMOS 17 ESP32S3 19
#define  NUMPIXELS          1


// This is the definition for the two cores of ESP32
#define CORE_CONTROL 1   // Core 0 is used for control tasks
#define CORE_COMM    0   // Core 1 is used for mqtt communication

// These are the protection pins for the driver
#define DRIVER_ON 21




// System definitions
#define DEFAULT_REFERENCE 0



// IOT topic definitions

// Topics published by the thermal system
#define SYS_USER_SIGNALS_CLOSED       "/motor/motor_" PLANT_NUMBER "/user/sig_closed"
#define SYS_USER_SIGNALS_OPEN       "/motor/motor_" PLANT_NUMBER "/user/sig_open"
// Topics received from the user
#define USER_SYS_SET_PID           "/motor/user/motor_" PLANT_NUMBER "/set_pid"
#define USER_SYS_SET_REF           "/motor/user/motor_" PLANT_NUMBER "/set_ref"

#define USER_SYS_STEP_CLOSED       "/motor/user/motor_" PLANT_NUMBER "/step_closed"
#define USER_SYS_STAIRS_CLOSED     "/motor/user/motor_" PLANT_NUMBER "/stairs_closed"
#define USER_SYS_PRBS_OPEN         "/motor/user/motor_" PLANT_NUMBER "/prbs_open"
#define USER_SYS_STEP_OPEN         "/motor/user/motor_" PLANT_NUMBER "/step_open"
#define USER_SYS_SET_GENCON        "/motor/user/motor_" PLANT_NUMBER "/set_gencon"
#define USER_SYS_PROFILE_CLOSED    "/motor/user/motor_" PLANT_NUMBER "/prof_closed"

/** Integer definitions of topics to avoid comparison with strings, which is more expensive
    in terms of computation */

#define DEFAULT_TOPIC                  0
#define USER_SYS_STEP_CLOSED_INT       1
#define USER_SYS_STAIRS_CLOSED_INT     2
#define USER_SYS_PRBS_OPEN_INT         3
#define USER_SYS_PROFILE_CLOSED_INT    4

/// Codes for modes of control
#define PID_CONTROLLER                 0
#define PID_CONTROLLER_SPEED           1
#define GENERAL_CONTROLLER             2
#define GENERAL_CONTROLLER_SPEED       3
#define GENERAL_CONTROLLER_2P          4
#define GENERAL_CONTROLLER_SPEED_2P    5

#define MAX_ORDER                      10

/// Sampling time
#define SAMPLING_TIME                  0.02
#define BUFFER_SIZE                    25


/**  This matrix is the gamma correction for the leds attached to the plant, which allow to "see" the
 *  control signal
*/
const uint8_t gamma8[] = {
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
        1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
        2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
        5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
        10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
        17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
        25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
        37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
        51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
        69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
        90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
        115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
        144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
        177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
        215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };



// conversion from volts to 12bits-PWM
const float percent2pwm = (float) (4095.0/5.0);

// conversion from pulses to degrees from a 600 ppr encoder
const float pulses2degrees = (float) (360.0/2800.0);


//// Definitions for the PBRS signal

# define PRBS_LENGTH 1023

const uint64_t pbrsSignal[32] = {
        0xfc0, 0xfcf030f3, 0x30300ccf, 0xc033cccf, 0xfcc00033, 0x33300cff, 0x33cfc3c, 0xc330f00c,
        0xc3f0f000, 0xf3333c3c, 0x3ccf0033, 0xc0ff3f0c, 0x3cfccf0c, 0x3033303, 0xc3c0c0c0, 0xf0333c0c,
        0xffc030c0, 0xff0fcf33, 0xcc3c33f3, 0xf30cf303, 0x3c3f30ff, 0xf33cf300, 0x300fc3f0, 0xc0c3fc03,
        0xcf03cc3f, 0x3fc30003, 0xc0fc333c, 0xcff3fcf3, 0xc00cc0f, 0xcc0f3fc0, 0xc3333f3, 0xc3c0f0
};


/// Definitions for dead zone

# define DEAD_ZONE 0


// Definitions for buttons

#define BUTTON_UN   14
#define BUTTON_LIFEFLOWER  7
uint16_t threshold  = 60000; // umbral de deteccion para los botones



// Functions



// Defining encoders
ESP32Encoder encoderMotor;
ESP32Encoder encoderPot;

// Defining RGB  led for visualizing variables
Adafruit_NeoPixel dispLed(NUMPIXELS, LIGHT_PIN, NEO_GRB); // connection to rgb leds that display temperature


// Task handles for resuming and suspending  tasks
TaskHandle_t h_controlPidTask;
TaskHandle_t h_publishStateTask;
TaskHandle_t h_generalControlTask;
TaskHandle_t h_identifyTask;
TaskHandle_t h_stepOpenTask;
TaskHandle_t h_speedControlPidTask;
TaskHandle_t h_speedGenControlTask;
TaskHandle_t h_buttonTask;

// PID control default parameters
float kp = 0.14635290651760308;
float ki = 0.7317645325880153;
float kd  =0.014147246761060576;
float N = 10;
float beta = 0.7;
float h = SAMPLING_TIME; //sampling time
float deadzone = DEAD_ZONE;
const float br = 1/0.99;
bool reset_int = false;







// loop control variables
float reference = 90;   //  Desired temperature of reference
float y = 0;            //  Temperature
float u = 0;            //  Control signal
float usat = 0;         //  Saturated control signal

// These arrays are buffer for sending frames of variables through mqtt
float uBuffer[BUFFER_SIZE];
float yBuffer[BUFFER_SIZE];
float rBuffer[BUFFER_SIZE];


// These parameters allow configuring the different user's commands
float low_val =    0;
float high_val =   0;
uint32_t points_stairs;
uint32_t duration;
uint32_t points_high = 50;
uint32_t points_low = 50;
uint32_t np = 0;
uint32_t total_time = 4294967295;
uint16_t divider = 1;



// Parameters of a general controller for the thermal system
uint8_t order;
float A [MAX_ORDER][MAX_ORDER] = {0};   // Controller's A matrix
float B [MAX_ORDER][2] = {0};    // Controller's B matrix
float C [MAX_ORDER] = {0};       // Controller's C matrix
float D [2] = {0};        // Controller's D matrix
float L [MAX_ORDER] = {0};       // L is the gain matrix for the antiwindup observer



// Vectors of values and times for storing  the edges of
// an user's defined arbitrary signal
float stairs[50];       //  Arbitrary signal
uint32_t timeValues[50];


//Integer code of the command sent by the user
unsigned int codeTopic = DEFAULT_TOPIC;

//Integer code of mode of control, which can be PID (default) or general control
uint8_t typeControl = PID_CONTROLLER;


// Internet connection variables
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);


// These miscellaneous functions allow to receive formatted data (for instance controllers) from user's app
float hex2Float(const char* string){
    // This function converts a 4-digit hexadecimal string received from MQTT to a floating-point number.
    // It enables the reception of precise float numbers without rounding, minimizing the transmitted byte count.
    unsigned long temp;
    sscanf(string, "%8x", &temp);
    float f = *(float *)&temp;
    return f;
}

float hex2Long(const char* string){
    // This function converts a 4-digit hexadecimal string received from MQTT to a long integer.
    // As a result, it enables the reception of precise integer numbers with the minimum byte usage.
    long l = strtol(string, NULL, 16);
    return l;
}

char * float2Hex(float number){
    // This function converts a floating-point number to a 4-digit hexadecimal string. Consequently,
    // it enables the transmission of an exact float number with the minimum byte usage through MQTT.
    static char str[9];
    unsigned long ui;
    memcpy(&ui, &number, sizeof (ui));
    sprintf(str, "%08x", ui);
    return str;
}


char * long2Hex(unsigned long number){
    // This function converts a long integer number to a 4-digit hexadecimal string. Consequently,
    // it enables the transmission of a big number with the minimum byte usage through MQTT.
    static char str[9];
    unsigned long ui;
    memcpy(&ui, &number, sizeof (ui));
    sprintf(str, "%08x", ui);
    return str;
}

void hexStringToFloatArray(float* floatArray, const char* hexString, unsigned int numFloats) {
    // This function translates a user-sent hexadecimal string into an array of floats, enabling the transmission
    // of arbitrary signals and parameters of a controller in a state space form.
    for (uint i = 0; i < numFloats; ++i) {
        unsigned long temp;
        sscanf(hexString + i * 8, "%8x", &temp); // Read 8 hex digits (32 bits)
        memcpy(floatArray + i, &temp, sizeof(float)); // Copy bits into float
    }
}

void hexStringToLongArray(uint32_t *longArray, const char* hexString, unsigned int numLongs) {
    // This function translates a user-sent hexadecimal string into an array of floats, enabling the transmission
    // of arbitrary signals and parameters of a controller in a state space form.
    for (unsigned int  i = 0; i < numLongs; ++i) {
        uint32_t  temp;
        sscanf(hexString + i * 8, "%8x", &temp); // Read 8 hex digits (32 bits)
        memcpy(longArray + i, &temp, sizeof(uint32_t)); // Copy bits into float
    }
}


void hexStringToMatrix(float* matrix, const char* hexString, unsigned int order, unsigned int cols, unsigned int maxcols) {
    // This function translates a user-sent hexadecimal string into a matrix of floats, enabling the transmission
    // of arbitrary signals and parameters of a controller in a state space form.
    for (uint i = 0; i < order; ++i) {
        unsigned long temp;
        for (uint j = 0; j < cols; j++) {
            sscanf(hexString + (i*cols + j) * 8, "%8x", &temp); // Read 8 hex digits (32 bits)
            memcpy(matrix + (i*maxcols + j ), &temp, sizeof(float)); // Copy bits into float
        }
    }

}

String arrayToString(float * array, uint framesize){
    String arrayHex;
    for (uint i = 0; i < framesize; i++ ){
        arrayHex.concat(float2Hex(array[i]));
        if (i < framesize-1){
            arrayHex.concat(',');
        }
    }
    return arrayHex;
}

float linearInterpolation(uint32_t t[], float r[], uint16_t n, uint32_t t_interp) {
    uint16_t i;
    float result;
    float r1;

    for (i = 0; i < n - 1; i++) {
        if (t[i] <= t_interp && t_interp <= t[i + 1]) {
            // Linear interpolation formula: y = y1 + (y2 - y1) * ((x - x1) / (x2 - x1))
            r1 = (float) ( t_interp -  t[i]) / (t[i + 1] - t[i]);
            result = r[i] + (r[i + 1] - r[i]) *  r1;
        }
    }
    return result;
}

void voltsToMotor( float volts){
    // This function convert a voltage value given in variable volts
    // to a bipolar pwm signal for controlling the motor

    unsigned int pwm = abs(volts)*percent2pwm;

    if (volts < 0){
        // if var volts is negative use CH_PWM_AIN2 to output a pwm signal
        // proportional to the input voltage
        ledcWrite(CH_PWM_AIN1, 0);
        ledcWrite(CH_PWM_AIN2, pwm);
    }
    else{
        // if var volts is negative use CH_PWM_AIN1 to output a pwm signal
        // proportional to the input voltage
        ledcWrite(CH_PWM_AIN1, pwm);
        ledcWrite(CH_PWM_AIN2, 0);
    }
}


void sensorToColormap(float x, uint8_t * rgbval) {
    // This function calculates the R, G, and B values of a fifth-degree polynomial approximation of the turbo colormap.
    // It is the colormap displayed by the WS2812 LEDs connected to the thermal plant.
    // The resulting colormap enables the user to visualize both the plant's temperature and the controller's actions.
    // The input 'x' represents a normalized value within the [0, 1] range, corresponding to the analog variable visualized by the LEDs.
    // The input 'rgbval' is a 3-element array designed to store the computed RGB values.

    float r;
    float g;
    float b;
    // colormap turbo
    r = 0.1357 + x * ( 4.5974 - x * ( 42.3277 - x * ( 130.5887 - x * ( 150.5666 - x * 58.1375 ))));
    g = 0.0914 + x * ( 2.1856 + x * ( 4.8052 - x * ( 14.0195 - x * ( 4.2109 + x * 2.7747 ))));
    b = 0.1067 + x * ( 12.5925 - x * ( 60.1097 - x * ( 109.0745 - x * ( 88.5066 - x * 26.8183 ))));


    r = constrain(r,0,1);
    g = constrain(g,0,1);
    b = constrain(b,0,1);

    rgbval[0] = (uint8_t) 255*r;
    rgbval[1] = (uint8_t) 255*g;
    rgbval[2] = (uint8_t) 255*b;
    rgbval[0] = pgm_read_byte(&gamma8[rgbval[0]]);
    rgbval[1] = pgm_read_byte(&gamma8[rgbval[1]]);
    rgbval[2] = pgm_read_byte(&gamma8[rgbval[2]]);
}

void displayLed(float var, float valmin, float valmax, float percent, uint8_t  led){
    // This function takes an analog variable and visualizes it as a colormap using the "sensorToColormap" function.
    // The 'var' parameter represents the analog variable, while 'valmin', 'valmax', and 'percent' serve as adjustment
    // parameters to scale the colormap.
    // The 'led' parameter, with values 0 or 1, denotes each of the LEDs attached to the plant.
    uint8_t rgb[3];
    float change;
    float x;
    float valmin_b;

    float valmax_b;
    float start = 0.025;
    change = valmax - valmin;
    valmin_b = valmin - percent * change;
    valmax_b = valmax + percent * change;
    x = start + constrain((1-start)* (var - valmin_b)/(valmax_b-valmin_b), 0, 1-start);
    sensorToColormap( x,  rgb);
    uint32_t color = dispLed.Color(rgb[0], rgb[1], rgb[2]);
    dispLed.setPixelColor(led, color );
    dispLed.setBrightness(40);
    dispLed.show();
}

void displayWhite() {
uint32_t color = dispLed.Color(255, 255,255);
dispLed.setPixelColor(0, color );
dispLed.setBrightness(20);
dispLed.show();
}



// This function suspend all  controlling tasks
void suspendAllTasks(){
    vTaskSuspend(h_publishStateTask);
    vTaskSuspend(h_generalControlTask);
    vTaskSuspend(h_controlPidTask);
    vTaskSuspend(h_speedControlPidTask);
    vTaskSuspend(h_speedGenControlTask);
    vTaskSuspend(h_identifyTask);
    vTaskSuspend(h_stepOpenTask);
    vTaskSuspend(h_buttonTask);
    voltsToMotor(0);
}


void resumeControl(){
      switch (typeControl) {
        case GENERAL_CONTROLLER_SPEED:
            vTaskResume(h_speedGenControlTask);
            break;
        case GENERAL_CONTROLLER_SPEED_2P:
            vTaskResume(h_speedGenControlTask);
            break;
        case PID_CONTROLLER:
            vTaskResume(h_controlPidTask);
            break;
        case PID_CONTROLLER_SPEED:
            vTaskResume(h_speedControlPidTask);
            break;
        case GENERAL_CONTROLLER:
            vTaskResume(h_generalControlTask);
            break;
        case GENERAL_CONTROLLER_2P:
            vTaskResume(h_generalControlTask);
            break;
    }
}

// This function activate the default controller
void defaultControl(){
    vTaskSuspend(h_publishStateTask);
    codeTopic = DEFAULT_TOPIC;
    reset_int=true;
    resumeControl();
    vTaskSuspend(h_identifyTask);
    vTaskSuspend(h_stepOpenTask);
    vTaskResume(h_buttonTask);
}

// These functions handle the WiFi and mqtt communication

void connectMqtt()
{
    printf("Starting MQTT connection...");
    if (mqttClient.connect(THINGNAME))
    {

        mqttClient.subscribe(USER_SYS_SET_PID);
        mqttClient.subscribe(USER_SYS_SET_GENCON);
        mqttClient.subscribe(USER_SYS_SET_REF);
        mqttClient.subscribe(USER_SYS_STEP_CLOSED);
        mqttClient.subscribe(USER_SYS_STAIRS_CLOSED);
        mqttClient.subscribe(USER_SYS_PRBS_OPEN);
        mqttClient.subscribe(USER_SYS_STEP_OPEN);
        mqttClient.subscribe(USER_SYS_PROFILE_CLOSED);
        printf("now connected to broker %s !\n", BROKER);
    }
    else
    {
        printf("Failed MQTT connection code %d \n try again in 3 seconds\n", mqttClient.state());
    }
}
void connectWiFi(){
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    printf("Plant %s is connecting to %s network , please wait\n", PLANT_NUMBER, WIFI_SSID);

    
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    printf("\n");
    printf("Connected to WIFI through IP: %s \n", WiFi.localIP().toString());

}



void handleConnections(void *pvParameters) {
    for (;;) {
        if (WiFi.status() != WL_CONNECTED) {
            connectWiFi();
        }
        if (!mqttClient.connected()) {
            vTaskDelay(2000);
            connectMqtt();
        }

        mqttClient.loop();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


void IRAM_ATTR onMqttReceived(char* lastTopic, byte* lastPayload, unsigned int lenght) {
    suspendAllTasks();
    JsonDocument doc;
    if (strstr(lastTopic, USER_SYS_SET_PID)) {
        deserializeJson(doc, lastPayload);
        kp = hex2Float((const char *) doc["kp"]);
        ki = hex2Float((const char *) doc["ki"]);
        kd = hex2Float((const char *) doc["kd"]);
        N = hex2Float((const char *) doc["N"]);
        beta = hex2Float((const char *) doc["beta"]);
        typeControl = hex2Long((const char *) doc["typeControl"]);
        beta = hex2Float((const char *) doc["beta"]);
        deadzone =  hex2Float((const char *) doc["deadzone"]);
        printf("PID parameters settled:\n    kp=%0.3f\n    ki=%0.3f\n    kd=%0.3f\n    N=%0.3f\n    Beta=%0.3f\n"
               "    Dead Zone=%0.3f\n", kp, ki, kd, N, beta, deadzone);
        printf("control code: %d\n", typeControl);
        reset_int = true;
        defaultControl();
    }
    else if(strstr(lastTopic, USER_SYS_SET_GENCON)){
        deserializeJson(doc, lastPayload);
        order = hex2Long((const char *) doc["order"]);
        const char *hex_A = doc["A"];
        const char *hex_B = doc["B"];
        const char *hex_C = doc["C"];
        const char *hex_D = doc["D"];
        const char *hex_L = doc["L"];
        typeControl  = hex2Long((const char *) doc["typeControl"]);
        deadzone =  hex2Float((const char *) doc["deadzone"]);
        hexStringToMatrix( *A, hex_A, order, order, MAX_ORDER);
        hexStringToMatrix( *B, hex_B, order, 2, 2);
        hexStringToFloatArray(C, hex_C, order);
        hexStringToFloatArray(D, hex_D, 2);
        hexStringToFloatArray(L, hex_L, order);
        printf("A general controller or order %d has been loaded\n", order);
        printf("A=\n");
        for ( uint8_t  i =0; i < order; i++ ){
            for (size_t j =0; j < order; j++ ) {
                printf("%0.4f    ", A[i][j]);
            }
            printf("\n");
        }
        printf("B=\n");
        for (uint8_t  i =0; i < order; i++ ) {
            for (size_t j =0; j < 2; j++ ) {
                printf("%0.4f    ", B[i][j]);
            }
            printf("\n");
        }
        printf("C=\n");
        for (uint8_t  j =0; j < order; j++ ) {
            printf("%0.4f    ", C[j]);
        }
        printf("\n");
        printf("D = %0.4f   %0.4f \n", D[0], D[1]);
        printf("L=\n");
        for (uint8_t  j =0; j < order; j++ ) {
            printf("%0.4f    ", L[j]);
        }
        printf("\n");
        printf("control code: %d\n", typeControl);
        reset_int = true;
        defaultControl();
    }

    else if (strstr(lastTopic, USER_SYS_SET_REF)) {
        deserializeJson(doc, lastPayload);
        reference = hex2Float((const char *) doc["reference"]);
        printf("Reference has been set to %0.2f degrees \n", reference);
        reset_int = true;
        defaultControl();
    }
    else if (strstr(lastTopic, USER_SYS_STEP_CLOSED )) {
        codeTopic = USER_SYS_STEP_CLOSED_INT;
        deserializeJson(doc, lastPayload);
        low_val = hex2Float((const char *) doc["low_val"]);
        high_val = hex2Float((const char *) doc["high_val"]);
        points_high = hex2Long((const char *) doc["points_high"]);
        points_low = hex2Long((const char *) doc["points_low"]);
        total_time = points_low + points_high;
        printf("Closed loop step response:\n");
        printf("    low value=%0.2f\n    high value=%0.2f\n    time in high =%0.2f\n    time in low=%0.2f\n", low_val,
               high_val, (float) (points_high * h), (float) (points_low * h));
        vTaskResume(h_publishStateTask);
        reset_int = true;
        resumeControl();
    }
    else if (strstr(lastTopic, USER_SYS_STAIRS_CLOSED)){
        codeTopic = USER_SYS_STAIRS_CLOSED_INT;
        deserializeJson(doc, lastPayload);
        low_val = hex2Float((const char *) doc["min_val"]);
        high_val = hex2Float((const char *) doc["max_val"]);
        points_stairs = hex2Long((const char *) doc["points_stairs"]);
        duration = hex2Long((const char *) doc["duration"]);
        const char *hexSignal = doc["signal"];
        hexStringToFloatArray(stairs, hexSignal, points_stairs);
        total_time = points_stairs * duration - 1;
        printf("Stairs signal of %d steps  with a duration of %0.2f secs.\n", points_stairs, h * total_time);
        vTaskResume(h_publishStateTask);
        reset_int = true;
        resumeControl();
    }

    else if(strstr(lastTopic, USER_SYS_PRBS_OPEN )) {
        codeTopic = USER_SYS_PRBS_OPEN_INT;
        deserializeJson(doc, lastPayload);
        low_val  = hex2Float((const char *) doc["low_val"]);
        high_val = hex2Float((const char *) doc["high_val"]);
        divider = hex2Long((const char *) doc["divider"]);
        total_time = PRBS_LENGTH * divider;

        printf("Open loop test with a prbs signal with %d steps with a duration of %0.2f secs.\n",
               total_time, (total_time-1) * h);
        vTaskResume(h_publishStateTask);
        reset_int = true;
        vTaskResume(h_identifyTask);
    }
    else if(strstr(lastTopic, USER_SYS_STEP_OPEN )){
        codeTopic = USER_SYS_STEP_CLOSED_INT;
        deserializeJson(doc, lastPayload);
        low_val = hex2Float((const char *) doc["low_val"]);
        high_val = hex2Float((const char *) doc["high_val"]);
        points_high = hex2Long((const char *) doc["points_high"]);
        points_low = hex2Long((const char *) doc["points_low"]);
        total_time = points_low + points_high;
        
        printf("Open loop step response:\n");
        printf("    low value=%0.2f\n    high value=%0.2f\n    time in high=%0.2f\n    time in low=%0.2f\n", low_val,
               high_val, (float) (points_high * h), (float) (points_low * h));
        vTaskResume(h_publishStateTask);
        reset_int = true;
        vTaskResume(h_stepOpenTask);
    }

    else if(strstr(lastTopic, USER_SYS_PROFILE_CLOSED)){
        codeTopic = USER_SYS_PROFILE_CLOSED_INT;
        deserializeJson(doc, lastPayload);
        points_stairs = hex2Long((const char *) doc["points"]);
        low_val = hex2Float((const char *) doc["min_val"]);
        high_val = hex2Float((const char *) doc["max_val"]);
        const char *timeValuesHex = doc["timevalues"];
        const char *stairsHex = doc["refvalues"];
        hexStringToFloatArray(stairs, stairsHex, points_stairs);
        hexStringToLongArray(timeValues, timeValuesHex, points_stairs);
        total_time = timeValues[points_stairs - 1];
        
        printf("Closed loop profile response with a duration of %0.2f secs.\n", total_time * h);
        printf("   refvalues = [ ");
        for (uint8_t  j =0; j < points_stairs; j++ ) {
            printf("%0.2f ", stairs[j]);
        }
        printf("]\n");
        printf("   timevalues = [ ");
        for (uint8_t  j =0; j < points_stairs; j++ ) {
            printf("%0.2f ", timeValues[j] * h);
        }
        printf("]\n");
        vTaskResume(h_publishStateTask);
        reset_int = true;
        resumeControl();
    }
}


void initMqtt() {
    mqttClient.setServer(BROKER, 1883);
    mqttClient.setCallback(onMqttReceived);
    mqttClient.setBufferSize (4096);
}

void publishStateClosed(uint indexFrame, uint currFrame) {
    JsonDocument doc;
    char jsonBuffer[BUFFER_SIZE * 29];
    doc["r"] = arrayToString(rBuffer, indexFrame + 1);
    doc["u"] = arrayToString(uBuffer, indexFrame + 1);
    doc["y"] = arrayToString(yBuffer, indexFrame + 1);
    doc["frame"] = long2Hex(currFrame);
    serializeJson(doc, jsonBuffer); // print to client
    mqttClient.publish(SYS_USER_SIGNALS_CLOSED, jsonBuffer);
}

void publishStateOpen(uint indexFrame, uint currFrame) {
    JsonDocument doc;
    char jsonBuffer[BUFFER_SIZE * 21];
    doc["u"] = arrayToString(uBuffer, indexFrame+1);
    doc["y"] = arrayToString(yBuffer, indexFrame+1);
    doc["frame"] = long2Hex(currFrame);
    serializeJson(doc, jsonBuffer); // print to client
    mqttClient.publish(SYS_USER_SIGNALS_OPEN, jsonBuffer);
}

static void publishStateTask (void *pvParameters) {
    // local constants
    uint32_t rv;
    uint16_t  indexFrame;
    uint16_t currFrame;

    for (;;) {
        xTaskNotifyWait(0, 0b0011, &rv, portMAX_DELAY);
        if (rv & 0b0001) {
            if (np <= total_time) {
                indexFrame = np % BUFFER_SIZE;
                currFrame = np/BUFFER_SIZE + 1;
                rBuffer[indexFrame] = reference;
                uBuffer[indexFrame] = usat;
                yBuffer[indexFrame] = y;
                if (indexFrame == BUFFER_SIZE - 1) {
                    publishStateClosed(indexFrame, currFrame);
                  }
                else if (np == total_time) {
                    publishStateClosed(indexFrame, currFrame);
                  }

            }
        }
        else if  ( rv & 0b0010 ){
            if (np <= total_time) {
                indexFrame = np % BUFFER_SIZE;
                currFrame = np / BUFFER_SIZE + 1;
                uBuffer[indexFrame] = u;
                yBuffer[indexFrame] = y;
                if (indexFrame == BUFFER_SIZE - 1) {
                    publishStateOpen(indexFrame, currFrame);
                }
                else if (np == total_time) {
                    publishStateOpen(indexFrame, currFrame);
                }

            }

        }

    }
}


float compDeadZone(float var, float dz){
    // This function compensates the dead zone of DC motor
    if (var == 0){
        return 0;
    }
    else {
        float sgnvar = abs(var)/var;
        return var + sgnvar * dz ;
    }

}

void setup_peripherals(void){
    //setting the pwm channels
    
    ledcSetup(CH_PWM_AIN1  ,  FREQUENCY_PWM, RESOLUTION_PWM);
    ledcSetup(CH_PWM_AIN2  ,  FREQUENCY_PWM, RESOLUTION_PWM);
    ledcAttachPin(PIN_AIN1 , CH_PWM_AIN1);
    ledcAttachPin(PIN_AIN2 , CH_PWM_AIN2);

    //open the serial port for revising the board messages
    
    Serial.begin(115200);
    //setting the encoders
    encoderMotor.attachFullQuad (CH_ENC_A, CH_ENC_B);
    encoderMotor.setCount(0);


   // Setting the potentiometer for setting reference

    pinMode( CH_ENC_A_POT, INPUT_PULLUP);
    pinMode( CH_ENC_B_POT, INPUT_PULLUP);
    pinMode(DRIVER_ON, OUTPUT);
    digitalWrite(DRIVER_ON, HIGH);
    encoderPot.attachFullQuad(CH_ENC_A_POT, CH_ENC_B_POT);
    encoderPot.clearCount();
    encoderPot.setFilter(1023);

   // setting rgb led
    dispLed.begin();
    dispLed.clear();
    dispLed.show();
    dispLed.setBrightness(30);
}