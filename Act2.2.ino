#include <micro_ros_arduino.h>

#include <stdio.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>

#define variador 34
#define PWM_PIN 26
#define In1 14
#define In2 27

#define freq 5000
#define resolution 8

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

std_msgs__msg__String msg_cmd;
std_msgs__msg__Float32MultiArray msg_state;

bool motorEnMovimiento = false;
int pot = 0;
int pwm = 0;
float voltaje = 0;
float duty = 0;
float Vcc = 3.3;

// Control de tiempo para publicar
unsigned long last_time = 0; 
const int publish_interval = 100; // Publicar cada 100 milisegundos (10 Hz)

//Callback
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;

  // Validación de seguridad por si llega un mensaje vacío
  if (msg == NULL || msg->data.data == NULL) return;

  if (strcmp(msg->data.data, "D") == 0) {
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
    motorEnMovimiento = true;
  }
  else if (strcmp(msg->data.data, "I") == 0) {
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
    motorEnMovimiento = true;
  }
  else if (strcmp(msg->data.data, "S") == 0) {
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
    ledcWrite(PWM_PIN, 0);
    motorEnMovimiento = false;
  }
}

void setup()
{
  // Transporte serial micro-ROS 
  set_microros_transports();

  pinMode(variador, INPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);

  ledcAttach(PWM_PIN, freq, resolution);

  delay(2000);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_motor_node", "", &support);

  //Subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "motor_cmd");

  // Publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "motor_state");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg_cmd,
    &subscription_callback,
    ON_NEW_DATA);
  msg_cmd.data.data = (char *) malloc(20 * sizeof(char)); 
  msg_cmd.data.capacity = 20;
  msg_cmd.data.size = 0;

  //Inicializar mensaje publicador
  msg_state.data.data = (float*) malloc(2 * sizeof(float));
  msg_state.data.size = 2;
  msg_state.data.capacity = 2;
  
  msg_state.layout.dim.capacity = 0;
  msg_state.layout.dim.size = 0;
  msg_state.layout.dim.data = NULL;
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  if (motorEnMovimiento)
  {
    // Solo publicar si ha pasado el tiempo establecido
    if (millis() - last_time >= publish_interval) {
      last_time = millis();

      pot = analogRead(variador);

      pwm = map(pot, 0, 4095, 0, 255);
      ledcWrite(PWM_PIN, pwm);

      voltaje = pot * (Vcc / 4095.0);
      duty = (voltaje / Vcc) * 100.0;

      msg_state.data.data[0] = voltaje;
      msg_state.data.data[1] = duty;

      rcl_publish(&publisher, &msg_state, NULL);
    }
  }
}
