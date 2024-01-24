#include "secrets.h"
#include "definitions.h"
#include <ESP32Encoder.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>
//#include <PubSubClient.h>
//#include <ArduinoJson.h>
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
float h = 0.02; //sampling time
const float br = 1/0.99;

// loop control variables
float reference = 90;   //  Desired temperature of reference
float y = 0;            //  Temperature
float u = 0;            //  Control signal
float usat = 0;         //  Saturated control signal



void voltsToMotor( float volts){
    unsigned int pwm = abs(volts)*percent2pwm;

    if (volts < 0){
        ledcWrite(CH_PWM_AIN1, 0);
        ledcWrite(CH_PWM_AIN2, pwm);
    }
    else{
        ledcWrite(CH_PWM_AIN1, pwm);
        ledcWrite(CH_PWM_AIN2, 0);
    }
}

float deadZone(float var, float offset){
    float udz;
    if (var >= 0){
        udz = var + offset;
    }
    else{
        udz = var - offset;
    }
    return udz;
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
       // reference = 90;
        y = encoderMotor.getCount() * pulses2degrees;
        e = reference - y;


//        if ((np == 0) & ( codeTopic!= DEFAULT_TOPIC)) {
//            I = 0;
//            y_ant = y;
//        }

        bi = ki*h;
        ad = kd/(N*(kd/N + h));
        bd = kd/(kd/N + h);
        P = kp*(beta * reference - y);
        D = ad*D - bd*(y - y_ant); // derivative action
        u = P+ I + D ; // control signal
        if (abs(e) <= 0.6){
            u = 0;
        }
        else{
            u  = deadZone(u, 0.2);
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
    // Initialize serial and wait for port to open:
    //    vTaskPrioritySet(nullptr,24);
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
}




void loop() {
//    voltsToMotor(-0.35);
//    vTaskDelay(2000);
//    voltsToMotor(0.35);
//    vTaskDelay(2000);
    reference = 90;
    vTaskDelay(10000);
    printf("ref = %0.2f  y = %0.2f\n", reference, y);
    reference = 90;
    vTaskDelay(10000);
    printf("ref = %0.2f  y = %0.2f\n", reference, y);


};