/********************************************************************************
  FIRMARE for UNDCMOTOR
  LB 2024 
  MIT License
********************************************************************************/

#include "connection_settings.h"
#include "definitions.h"



void computeReference() {

    switch (codeTopic) {

        case DEFAULT_TOPIC:
            reference += encoderPot.getCount() *  3.75 ;
            encoderPot.setCount(0);
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
                // sending the indication for publishing
                xTaskNotify(h_publishStateTask, 0b0001, eSetBits);
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
     See Astrom ans Murray Feedback Systems: an Introduction for Scientists and Engineers
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

        // sending the control signal to motor
        usat =  constrain(u, -5 , 5);
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
        // computing the current reference depending on the current command
        computeReference();
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
        //The task is suspended while awaiting a new sampling time
        vTaskDelayUntil(&xLastWakeTime, taskPeriod);  
        np +=1;     
        
        
    }
}


float computeController(float limit, bool type){
    /* This function computes a general controlller for the motor system
       The controller is defined by the state equation
           x[n+1] = (A - L * C) * x[k] + (B - L * D) * [r[n] y[n]]^T + L *u[n]
       where A, B, C ,D are the state equation matrices and L is the observer's gain for antiwindup,
       r[n] and y[n] are, respectively, the current reference and the current temperature
       of the system

       0 - 1 DOF
       1 - 2 DOF
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
            encoderMotor.clearCount();
            printf("Open loop step response completed\n");
            vTaskSuspend(h_publishStateTask);
            vTaskSuspend(h_stepOpenTask);

        }
        vTaskDelayUntil(&xLastWakeTime, taskPeriod);
        np +=1;
    }
}



static void buttonTask(void *pvParameters) {
    
    for (;;) {
   
       // Si el boton con logo UN supera el umbral pongo un color naranja 
       if (touchRead(BUTTON_UN) > threshold){           
            
            // incrementamos  la referencia en 90°
            reference += 90;  
            vTaskDelay(1000);          
                    
            }

        // Si el boton con el dibujo de flor de la vida supera el umbral pongo un color azul claro 
       if (touchRead(BUTTON_LIFEFLOWER) > threshold){  
   
            // decrementamos  la referencia en 90°           
            reference -= 180;
            vTaskDelay(1000);   
            }
       //   
       vTaskDelay(100); // espero 100 ms antes de hacer nuevamente la tarea
    }

}

void setup() {
    //Setting the maximal priority for setup
    vTaskPrioritySet(nullptr,24);
   
    //Setting peripherals
    setup_peripherals();

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


    // This is a task for the minimal UI of the board
    xTaskCreatePinnedToCore(
        buttonTask, // nombre de la rutina
        "activate motor with the buttons",
        4192,
        NULL,
        10,  // definimos una prioridad baja para esta tarea
        &h_buttonTask,
        CORE_CONTROL // la vamos a ejecutar en el CORE_0 que es comparte tareas con el procesador, baja prioridad
    );

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