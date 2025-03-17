// ========================= LIBRERÍAS =========================
#include <micro_ros_arduino.h>  // Biblioteca para integración de Micro-ROS en Arduino
#include <stdio.h>              // Biblioteca estándar de entrada y salida
#include <rcl/rcl.h>            // Biblioteca de ROS 2 para manejo de nodos y comunicaciones
#include <rcl/error_handling.h> // Manejo de errores de ROS 2
#include <rclc/rclc.h>          // Cliente ROS 2 para C
#include <rclc/executor.h>      // Executor para manejo de callbacks y timers en ROS 2
#include <std_msgs/msg/float32.h> // Tipo de mensaje de ROS 2 para enviar flotantes
#include "driver/ledc.h"        // Controlador de LED para PWM en ESP32
#include <math.h>               // Biblioteca matemática para operaciones complejas

// ========================= DEFINICIONES =========================
#define In1 27
#define In2 33
#define PWM_PIN 12
#define EncA  25
#define EncB  26
#define freq 5000
#define resolution 8
#define PWM1_Ch 0
#define Umax 14
#define Umin 0
#define T 0.01
#define pi M_PI

// ========================= VARIABLES GLOBALES =========================
volatile int32_t contador = 0;
volatile bool ASet = 0, BSet = 0, encoderDirection = false;
int32_t tiempo_act = 0, tiempo_ant = 0, delta_tiempo = 2e9;
float velocidad = 0;
int pulsos = 764;

const double P = 0.1752;
const double I = 1.5920;
const double D = 1.2854e-5;
const double N = 59325475.3983676;  // Constante del filtro derivativo
double error, error_anterior = 0;
double integral = 0, derivativo = 0, derivativo_filtrado = 0;
double setpoint = 0;
double control_signal = 0;

rcl_publisher_t vel_pub;
std_msgs__msg__Float32 msg_vel;
rcl_subscription_t setpoint_sub;
rcl_timer_t timer_1;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ========================= FUNCIONES =========================
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

void calcular_velocidad() {
    if (encoderDirection) {
        velocidad = (60000000 / (pulsos * delta_tiempo)); // Conversión a RPM
    } else {
        velocidad = -(60000000 / (pulsos * delta_tiempo)); // Conversión a RPM en dirección negativa
    }
}

void controlPID() {
    error = setpoint - velocidad;
    integral += error * T;
    derivativo = (error - error_anterior) / T;
    derivativo_filtrado = (D * N * derivativo) / (1 + N * T);  // Cálculo del término derivativo filtrado

    control_signal = (P * error) + (I * integral) + derivativo_filtrado;  // Cálculo de la señal de control (PID)

    control_signal = constrain(control_signal, Umin, Umax);

    ledcWrite(PWM1_Ch, control_signal);

    error_anterior = error;
}

// ========================= FUNCIONES ROS =========================
void timer_callback_1(rcl_timer_t * timer_1, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer_1 != NULL) {
        calcular_velocidad();
        msg_vel.data = velocidad * 2 * pi / 60;
        rcl_publish(&vel_pub, &msg_vel, NULL);
    }
}

void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    setpoint = abs(msg->data) * 14;
    controlPID();
}

void setup() {
    set_microros_transports();

    pinMode(In1, OUTPUT);
    pinMode(In2, OUTPUT);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(EncA, INPUT_PULLUP);
    pinMode(EncB, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(EncA), Encoder, CHANGE);

    ledcSetup(PWM1_Ch, freq, resolution);
    ledcAttachPin(PWM_PIN, PWM1_Ch);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support);
    rclc_publisher_init_default(&vel_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/motor_output");
    rclc_subscription_init_default(&setpoint_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/set_point");

    rclc_timer_init_default(&timer_1, &support, RCL_MS_TO_NS(10), timer_callback_1);
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_timer(&executor, &timer_1);
    rclc_executor_add_subscription(&executor, &setpoint_sub, &msg_vel, &subscription_callback, ON_NEW_DATA);

    msg_vel.data = 0;
}

void loop() {
    delay(100);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}