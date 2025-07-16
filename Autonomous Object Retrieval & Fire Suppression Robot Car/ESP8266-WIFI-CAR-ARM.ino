#include <Servo.h>
#include <PID_v1.h>
#define ENA 8
#define IN1 4
#define IN2 6
#define IN3 13
#define IN4 7
#define ENB 12
#define TRIG 3
#define ECHO 2
#define FIRE_SENSOR_PIN A4
#define FAN_MOTOR_INA A1
#define FAN_MOTOR_INB A0
#define SAFE_DISTANCE 20
#define DISTANCE_TO_TURN 7
#define SERVO_Z_AXIS_PIN 9
#define SERVO_CLAMP_PIN 5
#define SERVO_X_AXIS_PIN 11
#define SERVO_Y_AXIS_PIN 10

int x_axis_degree = 120;
int y_axis_degree = 33;
int z_axis_degree = 145;
int clamp_degree = 90;

Servo servo_z_axis;
Servo servo_clamp;
Servo servo_x_axis;
Servo servo_y_axis;

double setpoint = SAFE_DISTANCE;
double input, output;
double Kp = 2.0, Ki = 0.5, Kd = 1.0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(FIRE_SENSOR_PIN, INPUT);
  pinMode(FAN_MOTOR_INA, OUTPUT);
  pinMode(FAN_MOTOR_INB, OUTPUT);

  servo_z_axis.attach(SERVO_Z_AXIS_PIN);
  servo_clamp.attach(SERVO_CLAMP_PIN);
  servo_x_axis.attach(SERVO_X_AXIS_PIN);
  servo_y_axis.attach(SERVO_Y_AXIS_PIN);

  servo_x_axis.write(x_axis_degree);
  servo_y_axis.write(y_axis_degree);
  servo_z_axis.write(z_axis_degree);
  servo_clamp.write(clamp_degree);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, 255); 
}

int getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}

void moveCar(int speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  Serial.println("Turning Right");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  delay(500); 
  stopCar();
}

void pickUpObject() {
  Serial.println("Picking up object");
  servo_x_axis.write(120);
  servo_y_axis.write(100);
  servo_z_axis.write(100);
  delay(500);

  servo_clamp.write(90);
  delay(500);
  servo_clamp.write(65);
  delay(500);

  servo_y_axis.write(50);
  delay(500);
  servo_x_axis.write(60);
  delay(500);
  servo_clamp.write(90);
  delay(500);

  servo_x_axis.write(x_axis_degree);
  servo_y_axis.write(y_axis_degree);
  servo_z_axis.write(z_axis_degree);
  servo_clamp.write(clamp_degree);
  delay(500);
}

void activateFan() {
  digitalWrite(FAN_MOTOR_INA, LOW);
  digitalWrite(FAN_MOTOR_INB, LOW);
  Serial.println("No fire detected. Fan is OFF.");
}

void deactivateFan() {
  digitalWrite(FAN_MOTOR_INA, HIGH);
  digitalWrite(FAN_MOTOR_INB, LOW);
  Serial.println("Fire detected! Fan is ON.");
}

void loop() {
  int traveledDistance = 0;
  
  while (true) {
    int distance = getDistance();
    int fireDetected = analogRead(FIRE_SENSOR_PIN);
    
    if (fireDetected > 100) {
    activateFan(); 
    } else {
    stopCar();
    deactivateFan();
    Serial.println("Fire detected! car stop");
    }

    input = distance;
    pid.Compute();
    moveCar(output); 
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" | Speed: ");
    Serial.println(output);

    if (distance < SAFE_DISTANCE) {
      stopCar();
      Serial.println("Object detected! Stopping...");
      pickUpObject();
      moveCar(output);
    }

    if (traveledDistance >= DISTANCE_TO_TURN) {
      stopCar();
      turnRight();
      traveledDistance = 0;
    }
    traveledDistance++;
    delay(500);
  }
}