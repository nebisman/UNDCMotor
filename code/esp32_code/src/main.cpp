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

// PID control default parameters
float kp = 0.026048;
float ki = 0.018115;
float kd  =0.00074865;
float N = 11.9052;
float beta = 0.90423;
float h = SAMPLING_TIME; //sampling time
const float br = 1/0.99;







// loop control variables
float reference = 90;   //  Desired temperature of reference
float y = 0;            //  Temperature
float u = 0;            //  Control signal
float usat = 0;         //  Saturated control signal
float u_vector[BUFFER_SIZE];
float y_vector[BUFFER_SIZE];



// These parameters allow configuring the different user's commands
float low_val =    0;
float high_val =   0;
unsigned long duration;
unsigned long points = 60;
unsigned long points_low = 60;
unsigned long np = 0;
unsigned long total_time = 0;
unsigned int uee_points= (unsigned int) (20/h);
unsigned int divider = 35;
unsigned int prbs_points =  63 * divider;
unsigned int stab_points = (unsigned int) (150/h);



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
    for (size_t i = 0; i < numFloats; ++i) {
        unsigned long temp;
        sscanf(hexString + i * 8, "%8x", &temp); // Read 8 hex digits (32 bits)
        memcpy(floatArray + i, &temp, sizeof(float)); // Copy bits into float
    }

}

void hexStringToMatrix(float* matrix, const char* hexString, unsigned int order, unsigned int cols, unsigned int maxcols) {
    // This function translates a user-sent hexadecimal string into a matrix of floats, enabling the transmission
    // of arbitrary signals and parameters of a controller in a state space form.
    for (size_t i = 0; i < order; ++i) {
        unsigned long temp;
        for (size_t j = 0; j < cols; j++) {
            sscanf(hexString + (i*cols + j) * 8, "%8x", &temp); // Read 8 hex digits (32 bits)
            memcpy(matrix + (i*maxcols + j ), &temp, sizeof(float)); // Copy bits into float
        }
    }

}


String arrayToString(float * array){
    String arrayStr;
    for (size_t i = 0; i < BUFFER_SIZE; i++ ){
        arrayStr.concat(float2Hex(array[i]));
        if (i < BUFFER_SIZE-1){
            arrayStr.concat(',');
        }
    }
    return arrayStr;
}




// These functions handle the WiFi and mqtt communication


void connectMqtt()
{
    printf("Starting MQTT connection...");
    if (mqttClient.connect(THINGNAME, USER, PASSWORD))
    {
        // mqttClient.subscribe(USER_SYS_STAIRS_CLOSED);
        mqttClient.subscribe(USER_SYS_SET_PID);
        mqttClient.subscribe(USER_SYS_SET_REF);
       // mqttClient.subscribe(USER_SYS_STEP_CLOSED);
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
void connectWiFi()
{

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED)
    {
        for (char i=0; i <= 12; i++)
        {
            Serial.print(".");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        Serial.println("");
    }
    Serial.print("Connected to WIFI through IP: ");
    Serial.println(WiFi.localIP());
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
    JsonDocument doc;
    if (strstr(lastTopic, USER_SYS_SET_PID)) {
        codeTopic = USER_SYS_SET_PID_INT;
        typeControl = PID_CONTROLLER;
//        vTaskSuspend(h_identifyTask);
        vTaskSuspend(h_controlPidTask);
//        vTaskSuspend(h_generalControlTask);
        deserializeJson(doc, lastPayload);
        kp = hex2Float((const char *) doc["kp"]);
        ki = hex2Float((const char *) doc["ki"]);
        kd = hex2Float((const char *) doc["kd"]);
        N = hex2Float((const char *) doc["N"]);
        beta = hex2Float((const char *) doc["beta"]);
        // vTaskSuspend(h_identifyTask);

        printf("PID parameters settled:\n\tkp=%0.3f\n\tki=%0.3f\n\tkd=%0.3f\n\tN=%0.3f\n\tBeta=%0.3f\n",
               kp, ki, kd, N, beta);


    }
    else if (strstr(lastTopic, USER_SYS_SET_REF)) {
        codeTopic = USER_SYS_SET_REF_INT;
        vTaskSuspend(h_controlPidTask);
        deserializeJson(doc, lastPayload);
        reference = hex2Float((const char *) doc["reference"]);
//        vTaskSuspend(h_identifyTask);
//        if(typeControl == PID_CONTROLLER) {
//            vTaskSuspend(h_generalControlTask);
//            vTaskResume(h_controlPidTask);
//        }
//        else if (typeControl == GENERAL_CONTROLLER) {
//            vTaskSuspend(h_controlPidTask);
//            vTaskResume(h_generalControlTask);
//        }
        printf("Reference has been set to %0.2f degrees \n", reference);
        vTaskResume(h_controlPidTask);
    }
}


void initMqtt() {
    mqttClient.setServer(BROKER, 1883);
    mqttClient.setCallback(onMqttReceived);
    mqttClient.setBufferSize (4096);
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


float computeReference() {
    float ref = low_val;
    float percent = 0.5;

    switch (codeTopic) {
        case DEFAULT_TOPIC:
            ref = DEFAULT_REFERENCE;
            break;

        case USER_SYS_STEP_CLOSED_INT:
            if (np <= points_low) {
                ref = low_val;
            }
            else if (np <= points + points_low) {
                ref = high_val;
            }
            else if (np <= points + points_low + 1){
                voltsToMotor( 0);
                printf("Closed loop step response completed\n");
                //typeControl = PID_CONTROLLER;
                //codeTopic = DEFAULT_TOPIC;

                //vTaskSuspend(h_controlPidTask);
                //vTaskSuspend(h_generalControlTask);

            }
            np += 1;
            break;
//
//        case USER_SYS_STAIRS_CLOSED_INT:
//            if (np <= total_time) {
//                ref = stairs[np / duration];
//                displayLed(ref, 20, 90, 0, 0);
//                displayLed(y, 20, 90, 0, 1);
//            }
//            else if (np == total_time + 1) {
//                ledcWrite(PWM_CHANNEL, 0);
//                printf("Stairs closed loop response completed\n");
//                //typeControl = PID_CONTROLLER;
//                //codeTopic = DEFAULT_TOPIC;
//                ledcWrite(PWM_CHANNEL, 0 * percent2pwm);
//                dispLeds.clear();
//                dispLeds.show();
//                vTaskSuspend(h_controlPidTask);
//                vTaskSuspend(h_generalControlTask);
//            }
//            np += 1;
//            break;

    }
    return ref;
}





static void controlPidTask(void *pvParameters) {
    /* this function computes a two parameter PID control with Antiwindup
     See Astrom
    */
    static bool led_status = false;
    const TickType_t taskPeriod = (pdMS_TO_TICKS(1000*h));
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float bi;
    float ad;
    float bd;
    float P;          //  proportional action
    float D;          //  derivative action
    float e;
    static float y_ant = 0;      //  past output
    static float I = 0;



    for (;;) {
        led_status = !led_status;
        digitalWrite(LED_BUILTIN, led_status);
        y = encoderMotor.getCount() * pulses2degrees;
        reference = computeReference();
        e = reference - y;

        if ((np == 0) & ( codeTopic!= DEFAULT_TOPIC)) {
            I = 0;
            y_ant = y;
        }

        bi = ki*h;
        ad = kd/(N*(kd/N + h));
        bd = kd/(kd/N + h);
        P = kp*(beta * reference - y);
        D = ad*D - bd*(y - y_ant); // derivative action
        u = P + I + D ; // control signal
        if (abs(e) <= 0.6){
            u = 0;
        }
        else{
            u  = compDeadZone(u, 0.3, 0.05);
        }

        usat = constrain(u, -5, 5); //saturated control signal
        voltsToMotor(usat);
        I = I + bi *(reference - y) + br*(usat - u);  // calculo de la accion integral
        y_ant = y;
        //xTaskNotify(h_publishStateTask, 0b0001, eSetBits);
        vTaskDelayUntil(&xLastWakeTime, taskPeriod);
    }
}

void setup() {
    vTaskPrioritySet(nullptr,24);
    Serial.begin(500000);
    pinMode(LED_BUILTIN, OUTPUT);

    //setting the pwm channels
    ledcSetup(CH_PWM_AIN1  ,  FREQUENCY_PWM, RESOLUTION_PWM);
    ledcSetup(CH_PWM_AIN2  ,  FREQUENCY_PWM, RESOLUTION_PWM);
    ledcAttachPin(PIN_AIN1 , CH_PWM_AIN1);
    ledcAttachPin(PIN_AIN2 , CH_PWM_AIN2);

    //setting the encoders
    encoderMotor.attachFullQuad (CH_ENC_B, CH_ENC_A);
    encoderMotor.setCount (0);

    // Create the task for PID control in core 0.
    xTaskCreatePinnedToCore(
            controlPidTask, // This is the control routine
            "PID control",
            8192,
            NULL,
            23,
            &h_controlPidTask,
            CORE_CONTROL
    );
    //vTaskSuspend(h_controlPidTask);
    connectWiFi();
    initMqtt();
    connectMqtt();
    xTaskCreatePinnedToCore(
            handleConnections, // This is communication task
            "handle connections",
            4096,
            nullptr,
            1,
            nullptr,
            CORE_COMM // communications are attached to core 1
    );
    //vTaskSuspend(h_controlPidTask);
}




void loop() {
    float e = reference - y;
    vTaskDelay(1000);
    printf("ref = %0.2f\t y = %0.2f\n",reference, y);

};