#include "secrets.h"
#include "definitions.h"
#include <ESP32Encoder.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
//#include <Adafruit_NeoPixel.h>


// Defining encoder variables
ESP32Encoder encoderMotor;

// Task handles for resuming and suspending  tasks
TaskHandle_t h_controlPidTask;
TaskHandle_t h_publishStateTask;
TaskHandle_t h_generalControlTask;


// PID control default parameters
float kp = 0.026048;
float ki = 0.018115;
float kd  =0.00074865;
float N = 11.9052;
float beta = 0.90423;
float h = SAMPLING_TIME; //sampling time
const float br = .9;
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
//unsigned int uee_points= (unsigned int) (20/h);
//unsigned int divider = 35;
//unsigned int prbs_points =  63 * divider;
//unsigned int stab_points = (unsigned int) (150/h);


// Parameters of a general controller for the thermal system
uint8_t order;
float A [10][10];   // Controller's A matrix
float B [10][2];    // Controller's B matrix
float C [10];       // Controller's C matrix
float D [2];        // Controller's D matrix
float L [10];       // L is the gain matrix for the antiwindup observer



// Vectors of values and times for storing an the edges of
// an user's defined arbitrary signal
float stairs[50];       //  Arbitrary signal


//Integer code of the command sent by the user
unsigned int codeTopic = DEFAULT_TOPIC;

//Integer code of mode of control, which can be PID or general control (default)
unsigned int typeControl = PID_CONTROLLER;

// Internet connection variables
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);


// These miscellaneous functions allow to receive formatted data (for instance controllers) from user
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

void suspendAllTasks(){
    vTaskSuspend(h_publishStateTask);
    vTaskSuspend(h_generalControlTask);
    vTaskSuspend(h_controlPidTask);
    voltsToMotor(0);
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
       // mqttClient.subscribe(USER_SYS_STEP_OPEN);
//        mqttClient.subscribe(USER_SYS_PRBS_OPEN);
//        mqttClient.subscribe(USER_SYS_SET_GENCON);
        printf("connected to broker\n");
    }
    else
    {
        printf("Failed MQTT connection code %d \n try again in 3 seconds\n", mqttClient.state());
    }
}
void connectWiFi(){
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        vTaskDelay(pdMS_TO_TICKS(500));
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
            vTaskDelay(3000);
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
        codeTopic = USER_SYS_SET_PID_INT;
        typeControl = PID_CONTROLLER;
        deserializeJson(doc, lastPayload);
        kp = hex2Float((const char *) doc["kp"]);
        ki = hex2Float((const char *) doc["ki"]);
        kd = hex2Float((const char *) doc["kd"]);
        N = hex2Float((const char *) doc["N"]);
        reset_int = true;
        beta = hex2Float((const char *) doc["beta"]);
        vTaskResume(h_controlPidTask);
        printf("PID parameters settled:\n\tkp=%0.3f\n\tki=%0.3f\n\tkd=%0.3f\n\tN=%0.3f\n\tBeta=%0.3f\n",
               kp, ki, kd, N, beta);
    }
    else if(strstr(lastTopic, USER_SYS_SET_GENCON)){
        codeTopic = USER_SYS_SET_GENCON_INT;
        typeControl = GENERAL_CONTROLLER;
        deserializeJson(doc, lastPayload);
        order = hex2Long((const char *) doc["order"]);
        const char *hex_A = doc["A"];
        const char *hex_B = doc["B"];
        const char *hex_C = doc["C"];
        const char *hex_D = doc["D"];
        const char *hex_L = doc["L"];
        hexStringToMatrix( *A, hex_A, order, order, 10 );
        hexStringToMatrix( *B, hex_B, order, 2, 2);
        hexStringToFloatArray(C, hex_C, order);
        hexStringToFloatArray(D, hex_D, 2);
        hexStringToFloatArray(L, hex_L, order);
        reset_int = true;
        printf("A general controller or order %d has been loaded\n", order);
        printf("A=\n");
        for ( uint8_t  i =0; i < order; i++ ){
            for (size_t j =0; j < order; j++ ) {
                printf("%0.4f\t", A[i][j]);
            }
            printf("\n");
        }
        printf("B=\n");
        for (uint8_t  i =0; i < order; i++ ) {
            for (size_t j =0; j < 2; j++ ) {
                printf("%0.4f\t", B[i][j]);
            }
            printf("\n");
        }
        printf("C=\n");
        for (uint8_t  j =0; j < order; j++ ) {
            printf("%0.4f\t", C[j]);
        }
        printf("\n");
        printf("D = %0.4f   %0.4f \n", D[0], D[1]);
        printf("L=\n");
        for (uint8_t  j =0; j < order; j++ ) {
            printf("%0.4f\t", L[j]);
        }
        printf("\n");
        vTaskResume(h_generalControlTask);
    }

    else if (strstr(lastTopic, USER_SYS_SET_REF)) {
        codeTopic = USER_SYS_SET_REF_INT;
        deserializeJson(doc, lastPayload);
        reference = hex2Float((const char *) doc["reference"]);
        reset_int = true;
        printf("Reference has been set to %0.2f degrees \n", reference);
        if(typeControl == PID_CONTROLLER) {
            vTaskResume(h_controlPidTask);
        }
        else if (typeControl == GENERAL_CONTROLLER) {
            vTaskResume(h_generalControlTask);
        }

    }
    else if (strstr(lastTopic, USER_SYS_STEP_CLOSED )) {
        codeTopic = USER_SYS_STEP_CLOSED_INT;
        deserializeJson(doc, lastPayload);
        low_val = hex2Float((const char *) doc["low_val"]);
        high_val = hex2Float((const char *) doc["high_val"]);
        points_high = hex2Long((const char *) doc["points_high"]);
        points_low = hex2Long((const char *) doc["points_low"]);
        total_time = points_low + points_high;
        reset_int = true;
        printf("Closed loop step response:\n");
        printf("\tlow value=%0.2f\n\thigh value=%0.2f\n\ttime in high =%0.2f\n\ttime in low=%0.2f\n", low_val,
               high_val, (float) (points_high * h), (float) (points_low * h));
        if (typeControl == PID_CONTROLLER) {
            vTaskResume(h_publishStateTask);
            vTaskResume(h_controlPidTask);
        }
        else if (typeControl == GENERAL_CONTROLLER) {
             vTaskResume(h_publishStateTask);
             vTaskResume(h_generalControlTask);
        }

    }
    else if (strstr(lastTopic, USER_SYS_STAIRS_CLOSED)){
        codeTopic = USER_SYS_STAIRS_CLOSED_INT;
        deserializeJson(doc, lastPayload);
        points_stairs = hex2Long((const char *) doc["points_stairs"]);
        duration = hex2Long((const char *) doc["duration"]);
        const char *hexSignal = doc["signal"];
        hexStringToFloatArray(stairs, hexSignal, points_stairs);
        total_time = points_stairs * duration - 1;
        reset_int = true;
        printf("Stairs signal of %d steps  with a duration of %0.2f secs.\n", points_stairs, h * total_time);
        if(typeControl == PID_CONTROLLER) {
            vTaskResume(h_controlPidTask);
        }
//            else if (typeControl == GENERAL_CONTROLLER) {
//                vTaskSuspend(h_controlPidTask);
//                vTaskResume(h_generalControlTask);
//            }


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

static void publishStateTask (void *pvParameters) {
    // local constants
    uint32_t rv;
    uint16_t  indexFrame;
    uint16_t currFrame;
//    uint32_t t0;
//    uint32_t t1;

    for (;;) {
        xTaskNotifyWait(0, 0b0011, &rv, portMAX_DELAY);
        if (rv & 0b0001) {
            if (np <= total_time) {
                indexFrame = (np-1) % BUFFER_SIZE;
                currFrame = (np-1)/BUFFER_SIZE + 1;
                rBuffer[indexFrame] = reference;
                uBuffer[indexFrame] = usat;
                yBuffer[indexFrame] = y;
                if (indexFrame == BUFFER_SIZE - 1) {
                    //t0 = micros();
                    publishStateClosed(indexFrame, currFrame);
                   // t1 = micros();
                    //printf("elapsed= %d\n ", t1-t0);
                } else if (np == total_time) {
                   // t0 = micros();
                    publishStateClosed(indexFrame, currFrame);
                   // t1 = micros();
                    //printf("elapsed= %d\n ", t1-t0);
                }

            }
        }
    }
}



float compDeadZone(float var, float offset, float zm){
    // This function compensates the dead zone of DC motor
    float udz;
    float sgnvar = abs(var)/var;
    if (abs(var) < zm){
        udz = 0;
        }
    else{
         udz = var + sgnvar* offset;
        }

    return udz;
}


void computeReference() {

    switch (codeTopic) {

        case DEFAULT_TOPIC:
            reference = DEFAULT_REFERENCE;
            vTaskSuspend(h_publishStateTask);
            break;
        case USER_SYS_STEP_CLOSED_INT:
            if (np < points_low) {
                reference = low_val;

            }
            else if (np <= total_time) {
                reference = high_val;
            }
            else if (np == total_time + 1){
                voltsToMotor( 0);
                codeTopic = DEFAULT_TOPIC;
                //typeControl = PID_CONTROLLER;
                printf("Closed loop step response completed\n");
                //vTaskSuspend(h_controlPidTask);
                //vTaskSuspend(h_generalControlTask);
            }
            np += 1;
            break;
//
        case USER_SYS_STAIRS_CLOSED_INT:
            if (np <= total_time) {
                reference = stairs[np / duration];
            }
            else if (np == total_time + 1) {
                voltsToMotor( 0);
                codeTopic = DEFAULT_TOPIC;
                //typeControl = PID_CONTROLLER;
                printf("Stairs closed loop response completed\n");
                //vTaskSuspend(h_controlPidTask);
                //vTaskSuspend(h_generalControlTask);
            }
            np += 1;
            break;

    }
}





static void controlPidTask(void *pvParameters) {
    /* this function computes a two parameter PID control with Antiwindup
     See Astrom
    */
    static bool led_status = false;
    const TickType_t taskPeriod = (pdMS_TO_TICKS(1000*h));

    float bi;      //scaled integral constant
    float ad;      //scale derivative constant 1
    float bd;      //scale derivative constant 2
    float P;       //  proportional action
    float D;       //  derivative action
    float e;       //  current error r[n] - y [n]
    static float y_ant = 0;      //  past output
    static float I = 0;          // Integral action
    for (;;) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        led_status = !led_status;
        digitalWrite(PIN_CLOCK, led_status);

        // at start we reset the integral action and the state variables
        // when receiving a new command
        if (reset_int) {
            I = 0;
            np = 0;
            reset_int = false;
        }
        // reading the encoder
        y = encoderMotor.getCount() * pulses2degrees;
        // computing the current reference depending of the current command
        computeReference();
        // updating error
        e = reference - y;
        // updating controller parameters
        bi = ki*h;         // integral action scaled to sampling time
        // filtered derivative constantd
        ad = kd/(N*(kd/N + h));
        bd = kd/(kd/N + h);
        P = kp*(beta * reference - y); // proportional actions
        D =  ad*D - bd*(y - y_ant); // derivative action
        u = P + I + D ; // control signal
        // Compensation of dead zone for the DC motor
        u  = compDeadZone(u, .4, 0.05);

        //saturated control signal
        usat = constrain(u, -5, 5);

        // sending the control signal to motor
        voltsToMotor(usat);

        // updating integral action
        I = I + bi *(reference - y) + br*(usat - u);

        // updating output
        y_ant = y;
        // sending the indication for publishing
        xTaskNotify(h_publishStateTask, 0b0001, eSetBits);

        //The task is suspended while awaiting a new samplig time
        vTaskDelayUntil(&xLastWakeTime, taskPeriod);
    }
}


float computeController(){
    /* This function computes a general controlller for the thermal system
       The controller is defined by the state equation
           x[n+1] = (A - L * C) * x[k] + (B - L * D) * [r[n] y[n]]^T + L *u[n]
       where A, B, C ,D are the state equation matrices and L is the observer's gain for antiwindup,
       r[n] and y[n] are, respectively, the current reference and the current temperature
       of the thermal system
    */

    static float X[10] = {0,0,0,0,0,0,0,0,0,0};
    float Xnew[10] = {0,0,0,0,0,0,0,0,0,0};
    static float control = 0;

    if (reset_int) {
        for (size_t  i = 0; i < order; i++){
            X[i] = 0;
        }
        control = 0;
        reset_int = false;
    }

    for (size_t i = 0; i < order; i++ ){
        for (size_t j = 0; j < order; j++ ){
            Xnew[i] +=  A[i][j] * X[j];
        }
        Xnew[i] +=  B[i][0]* reference + B[i][1] * y + L[i] * control;
    }
    control = D[0] * reference + D[1] * y;
    for (size_t i = 0; i < order; i++ ){
        control += C[i]*X[i];
    }
    for (size_t i = 0; i < order; i++ ){
        X[i] = Xnew[i];
    }
    control = constrain(control, -5,  5);
    return control;
}


static void generalControlTask(void *pvParameters) {
    //this pin is only for vizualizing the correct timing of the control routine
    static bool led_status = false;
    // Set the sampling time for the task at h=0.02 ms
    const TickType_t taskPeriod = (pdMS_TO_TICKS(1000*h));
    float deadzone = 0.4;

    for (;;) {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        // this pin is only for vizualizing the correct timing of the control routine

        led_status = !led_status;
        digitalWrite(PIN_CLOCK, led_status);
        if (reset_int) {
            np = 0;
        }
        //printf("working..\n");
        // reading the encoder
        y = encoderMotor.getCount() * pulses2degrees;

        // computing the current reference depending of the current command
        computeReference();

        //
         u = computeController();
        // Compensation of dead zone for the DC motor

        u  = compDeadZone(u, 0.4, 0.05);

        //saturated control signal
        usat = constrain(u, -5, 5);

        // sending the control signal to motor
        voltsToMotor(usat);

        // sending the indication for publishing
        xTaskNotify(h_publishStateTask, 0b0001, eSetBits);

        //printf("entre ref = %0.2f    y = %0.2f    u=%0.2f \n", reference, y, usat);
        //The task is suspended while awaiting a new sampling time
        vTaskDelayUntil(&xLastWakeTime, taskPeriod);
    }
}


void setup() {
    //Setting the maximal priority for setup
    vTaskPrioritySet(nullptr,24);
    //This pin is for testing the timing of real time control routines
    pinMode(PIN_CLOCK , OUTPUT);
    //setting the pwm channels
    ledcSetup(CH_PWM_AIN1  ,  FREQUENCY_PWM, RESOLUTION_PWM);
    ledcSetup(CH_PWM_AIN2  ,  FREQUENCY_PWM, RESOLUTION_PWM);
    ledcAttachPin(PIN_AIN1 , CH_PWM_AIN1);
    ledcAttachPin(PIN_AIN2 , CH_PWM_AIN2);

    //open the serial port for revising the borad messages
    Serial.begin(115200);
    //setting the encoders
    encoderMotor.attachFullQuad (CH_ENC_B, CH_ENC_A);
    encoderMotor.setCount(0);


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

    // Create the task for general control in core 0.
    xTaskCreatePinnedToCore(
            generalControlTask, // This is the control routine
            "general control",
            8192,
            NULL,
            23,
            &h_generalControlTask,
            CORE_CONTROL
    );
    vTaskSuspend(h_generalControlTask);
    connectWiFi();
    initMqtt();
    connectMqtt();
    xTaskCreatePinnedToCore(
            handleConnections, // This is communication task
            "handle connections",
            4096,
            nullptr,
            2,
            nullptr,
            CORE_COMM // communications are attached to core 1
    );





}




void loop() {
    vTaskDelete(nullptr);
    vTaskDelay(1000);
    //printf("PID parameters settled:\n\tkp=%0.3f\n\tki=%0.3f\n\tkd=%0.3f\n\tN=%0.3f\n\tBeta=%0.3f\n",
//    //       kp, ki, kd, N, beta);
//    vTaskDelay(1000);
     printf("%s\n", USER_SYS_STAIRS_CLOSED);
//    //Serial.println(USER_SYS_STEP_CLOSED);
//

};