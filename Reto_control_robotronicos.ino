#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include "driver/adc.h"
#include "driver/ledc.h"
#include <math.h>

// Definición de pines
//#define variador 34  
#define PWM  27 
#define In1  12 
#define In2  14 
#define EncA  25 
#define EncB  26 

// Configuración PWM
#define freq 5000
#define resolution 8
#define PWM1_Ch 0 

// Variables de control
int32_t tiempo_act = 0, tiempo_ant = 0, delta_tiempo = 2000000;
int pwm = 128; // Valor inicial del PWM
float posicion = 0, velocidad = 0;
float resolucion = 0.4712;  // Conversión de pulsos a ángulo
int pulsos = 764;           // Pulsos por revolución del encoder
int32_t contador = 0;       // Contador de pulsos del encoder
volatile bool BSet = 0;
volatile bool ASet = 0;
volatile bool encoderDirection = false;
double velocidadDeseada = 10.0; // Velocidad deseada en rad/s

double valorEncoderRpm = 0;

// Parámetros del control PID
double Kp = 0.175202;
double Ki = 1.592025;
double Kd = 1.285401e-05;

double error, error_prev = 0;
double P, I, D, U;
double I_prec = 0, D_prec = 0;

// ROS 2
rcl_publisher_t controller;
std_msgs_msg_Float32 msg;
rcl_timer_t timer_1;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Interrupción del encoder para medir pulsos
void IRAM_ATTR Encoder() {
    BSet = digitalRead(EncB);
    ASet = digitalRead(EncA);
    if (BSet == ASet) {
        contador++;
        encoderDirection = true;
    } else {
        contador--;
        encoderDirection = false;
    }
    tiempo_act = micros();
    delta_tiempo = tiempo_act - tiempo_ant;
    tiempo_ant = tiempo_act;
}

// Cálculo de posición y velocidad del motor
void pose() {
    posicion = contador * resolucion;
    if (delta_tiempo > 0) {
        velocidad = 60000000.0 / (pulsos * delta_tiempo);
    } else {
        velocidad = 0;
    }
    error = velocidadDeseada - velocidad; // Cálculo de error para PID
    P = Kp * error;
    I += Ki * error * (delta_tiempo / 1000000.0); // Integra el error sobre el tiempo
    D = Kd * (error - error_prev) / (delta_tiempo / 1000000.0); // Deriva el error
    U = P + I + D; // Cálculo de la salida del control PID
    error_prev = error; // Guarda el error actual para el próximo cálculo

    pwm = constrain((int)(128 + U), 0, 255); // Calcula el nuevo valor de PWM
    ledcWrite(PWM1_Ch, pwm); // Ajusta el PWM según PID
}

// Callback del temporizador en ROS 2
void timer_callback_1(rcl_timer_t * timer_1, int64_t last_call_time) {  
    RCLC_UNUSED(last_call_time);
    if (timer_1 != NULL) {
        pose();  // Actualiza posición y velocidad
        msg.data = velocidad; // Publica la velocidad actual
        RCSOFTCHECK(rcl_publish(&controller, &msg, NULL));
    }
}

void setup() {
    set_microros_transports();
    
    // Configuración de pines
    pinMode(PWM, OUTPUT);
    pinMode(In1, OUTPUT);
    pinMode(In2, OUTPUT);
    pinMode(EncA, INPUT_PULLUP);
    pinMode(EncB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(EncA), Encoder, CHANGE);

    // Configuración de PWM
    ledcSetup(PWM1_Ch, freq, resolution);
    ledcAttachPin(PWM, PWM1_Ch);
    
    Serial.begin(115200);
    
    // Configuración de ROS 2
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "Control_Robotronicos", "", &support));
    RCCHECK(rclc_publisher_init_default(&controller, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "ri_controller"));
    
    // Cambio en la frecuencia del temporizador: 1 ms (1000 Hz)
    const unsigned int timer_timeout = 1; // 1 ms
    RCCHECK(rclc_timer_init_default(&timer_1, &support, RCL_MS_TO_NS(timer_timeout), timer_callback_1));
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_1));

    msg.data = 0;
}

void loop() {
    delay(10);  // Reducimos el delay para mejorar la respuesta
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}