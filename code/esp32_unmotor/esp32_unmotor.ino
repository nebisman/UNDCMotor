/********************************************************************************
  FIRMARE for UNDCMOTOR
  LB 2024 
  MIT License
********************************************************************************/

#include "connection_settings.h"
#include "definitions.h"
#include <ESP32Encoder.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <Adafruit_NeoPixel.h>
#include <OneWire.h>




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
}

// These functions handle the WiFi and mqtt communication

void connectMqtt()
{
    printf("Starting MQTT connection...");
    if (mqttClient.connect(THINGNAME, USER, PASSWORD))
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
    #ifdef UNALCONNECTION
        WiFi.begin(WIFI_SSID);
        printf("\nPlant %s is connecting to UNAL network, please wait\n", PLANT_NUMBER);
    #else    
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        printf("Plant %s is connecting to %s network , please wait\n", PLANT_NUMBER, WIFI_SSID);
    #endif
    
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
            if (!mqttClient.connected()) {
                vTaskDelay(500);
                connectMqtt();
            }
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


void computeReference() {

    switch (codeTopic) {

        case DEFAULT_TOPIC:
            reference = encoderPot.getCount() *  3.75 ;
            if (abs(usat)> 0 ){
                 displayLed(usat, -5, 5, 0.1, 0);
            }
            else{
                displayWhite();
            }
            break;
        case USER_SYS_STEP_CLOSED_INT:
            if (np < points_low) {
                reference = low_val  + encoderPot.getCount() *  3.75;
                // sending the indication for publishing
                xTaskNotify(h_publishStateTask, 0b0001, eSetBits);
                displayLed(y, low_val, high_val, 0.5, 0);
            }
            else if (np <= total_time) {
                reference = high_val + encoderPot.getCount() *  3.75;
                // sending the indication for publishing
                xTaskNotify(h_publishStateTask, 0b0001, eSetBits);
                displayLed(y, low_val, high_val, 0.5, 0);
            }
            else if (np == total_time + 1){
                printf("Closed loop step response completed\n");
                encoderMotor.setCount(0);
                encoderPot.setCount(0);
                voltsToMotor(0);
                defaultControl();
            }
            break;

        case USER_SYS_STAIRS_CLOSED_INT:
            if (np <= total_time) {
                reference = stairs[np / duration];
                // sending the indication for publishing
                xTaskNotify(h_publishStateTask, 0b0001, eSetBits);
                displayLed(y, low_val, high_val, 0, 6);

            }
            else if (np == total_time + 1) {
                printf("Stairs closed loop response completed\n");
                encoderMotor.setCount(0);
                encoderPot.setCount(0);
                voltsToMotor(0);
                defaultControl();
            }
            break;

        case USER_SYS_PROFILE_CLOSED_INT:
            if (np <= total_time) {
                reference = linearInterpolation(timeValues, stairs, points_stairs, np);
                displayLed(y, low_val, high_val, 0, 6);
             
            }
            else if (np == total_time + 1) {
                printf("Closed loop profile response completed\n");
                encoderMotor.setCount(0);
                encoderPot.setCount(0);
                voltsToMotor(0);
                defaultControl();
            }
            break;

    }
}


static void speedControlPidTask(void *pvParameters) {
    /* this function computes a two parameter PID control with Antiwindup
     See Astrom ans Murray
    */
    static bool led_status = false;
    const TickType_t taskPeriod = (pdMS_TO_TICKS(1000*h));

    float bi;      //scaled integral constant
    float ad;      //scale derivative constant 1
    float bd;      //scale derivative constant 2
    float P;       //  proportional action
    float D;       //  derivative action

    /** state variables for the PID */
    static float y_ant = 0;      //  past output
    static float I = 0;          // Integral action


    for (;;) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        // at start we reset the integral action and the state variables
        // when receiving a new command
        if (reset_int) {
            np = 0;
            encoderPot.setCount(0);
            encoderMotor.clearCount();
            reset_int = false;
            y_ant = 0;
            I = 0;
            vTaskDelay(1000*h);
            continue;
        }
        // reading the encoder
        y = encoderMotor.getCount() * pulses2degrees / h;
        encoderMotor.clearCount();
        // computing the current reference depending of the current command
        computeReference();
        // updating error

        /** updating controller parameters if they changed */
        // integral action scaled to sampling time
        bi = ki*h;
        // filtered derivative constant
        ad = kd/(N*(kd/N + h));
        bd = kd/(kd/N + h);
        P = kp*(beta * reference - y); // proportional actions
        D =  ad*D - bd*(y - y_ant); // derivative action
        u = P + I + D ; // control signal
        if (abs(reference) <= 10 ){
            u = 0;
        }
        //saturated control signal
        usat = constrain(u, -5, 5);
        // sending the control signal to motor
        voltsToMotor(usat);

        // updating integral action
        if (ki != 0) {
            I = I + bi * (reference - y) + br * (usat - u);
        }
        // updating state for derivative action
        y_ant = y;
        //The task is suspended while awaiting a new sampling time
        vTaskDelayUntil(&xLastWakeTime, taskPeriod);
        np +=1;
    }
}



static void controlPidTask(void *pvParameters) {
    /* this function computes a two parameter PID control with Antiwindup
     See Astrom ans Murray
    */
    static bool led_status = false;
    const TickType_t taskPeriod = (pdMS_TO_TICKS(1000*h));

    float bi;      //scaled integral constant
    float ad;      //scale derivative constant 1
    float bd;      //scale derivative constant 2
    float P;       //  proportional action
    float D;       //  derivative action

    /** state variables for the PID */
    static float y_ant = 0;      //  past output
    static float I = 0;          // Integral action
    float e;
    float v;

    for (;;) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        // at start we reset the integral action and the state variables
        // when receiving a new command
        if (reset_int) {
            I = 0;
            np = 0;
            encoderMotor.clearCount();
            encoderPot.clearCount();
            reset_int = false;
            y_ant = 0;
            continue;
        }
        y = encoderMotor.getCount() * pulses2degrees;
        // computing the current reference depending of the current command
        computeReference();
        // updating error


        /** updating controller parameters if they changed */
        // integral action scaled to sampling time
        bi = ki*h;
        // filtered derivative constant
        ad = kd/(N*(kd/N + h));
        bd = kd/(kd/N + h);
        P = kp*(beta * reference - y); // proportional actions
        D =  ad*D - bd*(y - y_ant); // derivative action
        u = P + I +  D ; // control signal
        e = reference - y;
        v = (y - y_ant)/h;

        // for the default control we allow to relax the system with zero control signal
        if ((codeTopic == DEFAULT_TOPIC ) & (abs(e) <= 0.5) & (abs(v) <= 20)){
            u=0;
             }


        //saturated control signal
        usat =  constrain(u, -5 + deadzone, 5 - deadzone);
        voltsToMotor(compDeadZone(usat, deadzone));

        // updating integral action
        if (ki!= 0) {
            I = I + bi * e + br * (usat - u);
        }
        // updating output
        y_ant = y;

        if (reset_int) {I=0;}
        //The task is suspended while awaiting a new samplig time
        vTaskDelayUntil(&xLastWakeTime, taskPeriod);  
        np +=1;     
        
        
    }
}


float computeController(float limit, bool type){
    /* This function computes a general controlller for the thermal system
       The controller is defined by the state equation
           x[n+1] = (A - L * C) * x[k] + (B - L * D) * [r[n] y[n]]^T + L *u[n]
       where A, B, C ,D are the state equation matrices and L is the observer's gain for antiwindup,
       r[n] and y[n] are, respectively, the current reference and the current temperature
       of the thermal system

       0 - 1par
       1 -
    */

    static float X[10] = {0};
    float Xnew[10] = {0};
    static float control = 0;
    float e;
    static float y_ant;
    float v;

    if (reset_int) {
        for (size_t  i = 0; i < order; i++){
            X[i] = 0;
        }
        control = 0;
        y_ant=0;
       // reset_int = false;
        return 0;
    }
    else {
        e = reference - y;
        for (size_t i = 0; i < order; i++) {
            for (size_t j = 0; j < order; j++) {
                Xnew[i] += A[i][j] * X[j];
            }
            if (type == true) {
               Xnew[i] += B[i][0] * reference + B[i][1] * y + L[i] * control;
                           }
            else{
               Xnew[i] += B[i][0] * e + L[i] * control;
            }
        }
        control = D[0] * reference + D[1] * y;
        for (size_t i = 0; i < order; i++) {
            control += C[i] * X[i];
        }
        for (size_t i = 0; i < order; i++) {
            X[i] = Xnew[i];
        }
        v = (y - y_ant) / h;
        if ((abs(e) <= 0.23) & abs(v) <= 10){
            control = 0;
        }

        y_ant = y;
        control = constrain(control, -limit,  limit);
        return control;
    }

}







static void generalControlTask(void *pvParameters) {
    //this pin is only for vizualizing the correct timing of the control routine
    // Set the sampling time for the task at h=0.02 ms
    const TickType_t taskPeriod = (pdMS_TO_TICKS(1000*h));
    float v;
    float e;

    for (;;) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        if (reset_int) {
            encoderMotor.clearCount();
            encoderPot.clearCount();
            computeController(5-deadzone, false);
            reset_int = false;
            np = 0;
        }
        // reading the encoder
        y = encoderMotor.getCount() * pulses2degrees;

        // computing the current reference depending of the current command
        computeReference();


        // computing the general controlled
        if (typeControl == GENERAL_CONTROLLER){
            u = computeController(5-deadzone, false);

        }
        else if (typeControl == GENERAL_CONTROLLER_2P) {
            u = computeController(5-deadzone, true);
        }



        // Compensation of dead zone for the DC motor
        usat = compDeadZone(u, deadzone);
        // sending the control signal to motor
        voltsToMotor(usat);
        //The task is suspended while awaiting a new sampling time
        vTaskDelayUntil(&xLastWakeTime, taskPeriod);
        np +=1;
    }
}




static void speedGenControlTask(void *pvParameters) {
    //this pin is only for vizualizing the correct timing of the control routine
    // Set the sampling time for the task at h=0.02 ms
    const TickType_t taskPeriod = (pdMS_TO_TICKS(1000*h));


    for (;;) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        if (reset_int) {
            encoderMotor.clearCount();
            np = 0;
            encoderPot.clearCount();
            computeController(5, false);
            reset_int = false;
            continue;
        }
        // reading the encoder
        y = encoderMotor.getCount() * pulses2degrees / h;
        encoderMotor.clearCount();
        // computing the current reference depending of the current command
        computeReference();

        // computing the general controlled
        if (typeControl == GENERAL_CONTROLLER_SPEED){
            usat = computeController(5, false);
        }
        else if (typeControl == GENERAL_CONTROLLER_SPEED_2P) {
            usat = computeController(5, true);
        }


        if (abs(reference) <= 10 ){
            usat = 0;
        }

        // sending the control signal to motor
        voltsToMotor(usat);
        //The task is suspended while awaiting a new sampling time
        vTaskDelayUntil(&xLastWakeTime, taskPeriod);
        np +=1;
    }
}




static void identifyTask(void *pvParameters) {
/*  This task involves identifying the DC motor using
 *  a PRBS signal stored in the prbsSignal array.
 *  */

const TickType_t taskPeriod = (uint32_t) (1000 * h);
    uint byteIndex = 0;
    uint bitShift;
    bool  currBit;

    for (;;) {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        if (reset_int){
            displayLed(0l , -1, 1, 0, 0);
            voltsToMotor(abs(low_val)/low_val);
            vTaskDelay(1000);
            float val = (low_val + high_val)/2;
            voltsToMotor(val);
            displayLed(val , low_val, high_val, 0.5, 0);
            vTaskDelay(3000);
            reset_int = false;
            np = 0;
            encoderMotor.clearCount();
            vTaskDelay(1000*h);
            continue;
        }

        if (np <= total_time) {
            y = encoderMotor.getCount() * pulses2degrees /h;
            encoderMotor.clearCount();
            byteIndex =  (np / (32 * divider));
            bitShift = 31 - ((np/divider) % 32);
            currBit = (pbrsSignal[byteIndex] >> bitShift) & 1;
            if (currBit) {
                u = high_val;
            } else {
                u = low_val;
            }
            voltsToMotor(u);
            xTaskNotify(h_publishStateTask, 0b0010, eSetBits);
            displayLed(u, low_val, high_val, 0.5, 0);

        }
        else if (np == total_time + 1) {
            voltsToMotor(0);
            printf("Open loop PBRS response completed\n");
            encoderMotor.clearCount();
            defaultControl();
        }
        vTaskDelayUntil(&xLastWakeTime, taskPeriod);
        np +=1;
    }
}



static void stepOpenTask(void *pvParameters) {
/*  This task returns the velocity step response
 *  of the  DC motor
 *  */
    const TickType_t taskPeriod = (uint32_t) (1000 * h);
    for (;;) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        if (reset_int){
            //this allow to set steady state for the low step
            voltsToMotor(abs(high_val)/high_val);
            vTaskDelay(pdMS_TO_TICKS(1000));
            if (low_val != 0) {
                voltsToMotor(low_val);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            reset_int = false;
            np = 0;
            encoderMotor.clearCount();
            vTaskDelay(pdMS_TO_TICKS(1000*h));
            continue;
        }
        //currPosition = encoderMotor.getCount() * pulses2degrees;
        y = encoderMotor.getCount() * pulses2degrees/ h;
        encoderMotor.clearCount();

        //lastPosition = currPosition;
        if (np < points_low) {
            u = low_val;
            voltsToMotor(u);
            xTaskNotify(h_publishStateTask, 0b0010, eSetBits);
            displayLed(u, -5 ,5, 0.2, 0);
        }
        else if (np <= total_time) {
            u  = high_val;
            voltsToMotor(u);
            xTaskNotify(h_publishStateTask, 0b0010, eSetBits);
            displayLed(u, -5 ,5, 0.2, 0);
        }
        else if (np == total_time + 1 ){
            voltsToMotor( 0);
            printf("Open loop step response completed\n");
            vTaskSuspend(h_publishStateTask);
            vTaskSuspend(h_stepOpenTask);

        }
        vTaskDelayUntil(&xLastWakeTime, taskPeriod);
        np +=1;
    }
}


void setup() {
    //Setting the maximal priority for setup
    vTaskPrioritySet(nullptr,24);

    //setting the pwm channels
    
    ledcSetup(CH_PWM_AIN1  ,  FREQUENCY_PWM, RESOLUTION_PWM);
    ledcSetup(CH_PWM_AIN2  ,  FREQUENCY_PWM, RESOLUTION_PWM);
    ledcAttachPin(PIN_AIN1 , CH_PWM_AIN1);
    ledcAttachPin(PIN_AIN2 , CH_PWM_AIN2);

    //open the serial port for revising the borad messages
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

    //create the task for communication in core 1
    xTaskCreatePinnedToCore(
            publishStateTask, // This is communication task
            "User command IOT",
            8192,
            NULL,
            10,
            &h_publishStateTask,
            CORE_COMM
    );
    vTaskSuspend(h_publishStateTask);

    // Creating the task for PID control in core 0.
    xTaskCreatePinnedToCore(
            controlPidTask, // This is the control routine
            "PID control",
            8192,
            NULL,
            23,
            &h_controlPidTask,
            CORE_CONTROL
    );
    // Creating the task for speed PID control in core 0.
    xTaskCreatePinnedToCore(
            speedControlPidTask, // This is the control routine
            "Speed PID control",
            8192,
            NULL,
            23,
            &h_speedControlPidTask,
            CORE_CONTROL
    );
    vTaskSuspend(h_speedControlPidTask);

    connectWiFi();
    initMqtt();
    connectMqtt();
    xTaskCreatePinnedToCore(
            handleConnections, // This is communication task
            "handle connections",
            4096,
            nullptr,
            10,
            nullptr,
            CORE_COMM // communications are attached to core 1
    );

    // Create the task for general control in core 0.
    xTaskCreatePinnedToCore(
            generalControlTask, // This is the control routine
            "general control",
            8192,
            NULL,
            20,
            &h_generalControlTask,
            CORE_CONTROL
    );
    vTaskSuspend(h_generalControlTask);

    xTaskCreatePinnedToCore(
            speedGenControlTask, // This is the control routine
            "speed general control",
            8192,
            NULL,
            20,
            &h_speedGenControlTask,
            CORE_CONTROL
    );
    vTaskSuspend(h_speedGenControlTask);


    xTaskCreatePinnedToCore(
            identifyTask,
            "prbs-ident",
            4096,
            NULL,
            20,
            &h_identifyTask,
            CORE_CONTROL
    );
    vTaskSuspend(h_identifyTask);

    xTaskCreatePinnedToCore(
            stepOpenTask,
            "step-open",
            4096,
            NULL,
            20,
            &h_stepOpenTask,
            CORE_CONTROL
    );
    vTaskSuspend(h_stepOpenTask);





}




void loop() {
    vTaskDelete(nullptr);
}