#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <SPI.h>
#include <MPU6050_6Axis_MotionApps20.h>
#define BT_RXD 3 //아두이노의 RX가 12, hc-6의 TX로 이어짐
#define BT_TXD 12
#define LED 13

SoftwareSerial bluetooth(BT_RXD, BT_TXD);

MPU6050 mpu; // mpu interface object
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int v1,v2,v3,v4,v1x,v2x,v3x,v4x,v1y,v2y,v3y,v4y;
char bt='h';
Quaternion q;
VectorFloat gravity;
float ypr[3];
float yprLast[3];
int16_t gyro[3];
volatile bool mpuInterrupt = false;

void dmpDataReady() {
mpuInterrupt = true;
}
///////////below is Yngneers data/////////////////////////////////////////////////////////////////////////
Servo front, back, left, right;
double T = 0.0045; // Loop time.
double Xoffset = 1.74;
double Yoffset = -0.32;
/*throttle*/
int throttle = 0;
int throttle_real = 0;
int front_output = 0, back_output = 0, left_output = 0, right_output = 0;
/*PID gain variables i게인원래 0.7 2016_1_22. outer p는 4.5, 3. inner p는 0.83, 0.8 */
double P_angle_gain_y = 6.5; double P_gain_y = 5; double I_gain_y = 0.7; double D_gain_y = 0.135;
double P_angle_gain_x = 6.5; double P_gain_x = 5; double I_gain_x = 0.7; double D_gain_x = 0.135;
/*No need to change*/
double error_x;
double error_pid_x, error_pid_x1;
double P_angle_pid_x;
double P_x, I_x, D_x, PID_x;
double desired_angle_x = 0;
/*No need to change*/
double error_y;
double error_pid_y, error_pid_y1;
double P_angle_pid_y;
double P_y, I_y, D_y, PID_y;
double desired_angle_y = 0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void dmpsetup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
Wire.begin();
TWBR = 12; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
Fastwire::setup(400, true);
#endif
Serial.begin(115200);

while (!Serial);
//Serial.println(F("Initializing I2C devices..."));
mpu.initialize();
// Serial.println(F("Testing device connections..."));
// Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
// Serial.println(F("Initializing DMP..."));
devStatus = mpu.dmpInitialize();
mpu.setXGyroOffset(33);
mpu.setYGyroOffset(-13);
mpu.setZGyroOffset(8);
mpu.setZAccelOffset(1416);
if (devStatus == 0) {
// Serial.println(F("Enabling DMP..."));
mpu.setDMPEnabled(true);
// Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
attachInterrupt(0, dmpDataReady, RISING);
mpuIntStatus = mpu.getIntStatus();
// Serial.println(F("DMP ready! Waiting for first interrupt..."));
dmpReady = true;
packetSize = mpu.dmpGetFIFOPacketSize();
}
else {
// Serial.print(F("DMP Initialization failed (code "));
// Serial.print(devStatus);
// Serial.println(F(")"));
}
}

void setup() {
delay(5000);
Serial.begin(115200);
bluetooth.begin(9600);
dmpsetup();
servoset(); //initialize settings for each motors.
//Serial.setTimeout(5); //bluetooth parse time set.
pinMode(LED,OUTPUT);
Serial.println("v1,v4");
}

void dmploop() {
if (!dmpReady) return;
while (!mpuInterrupt && fifoCount < packetSize) {
}
mpuInterrupt = false;
mpuIntStatus = mpu.getIntStatus();
fifoCount = mpu.getFIFOCount();
if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
mpu.resetFIFO();
// Serial.println(F("FIFO overflow!"));
} else if (mpuIntStatus & 0x02) {
while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
mpu.getFIFOBytes(fifoBuffer, packetSize);
fifoCount -= packetSize;
mpu.dmpGetGyro(gyro, fifoBuffer);
mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
ypr[1] = (ypr[1] * 180 / M_PI); //y
ypr[2] = (ypr[2] * 180 / M_PI); //x
ypr[0] = (ypr[0] * 180 / M_PI); //z
ypr[1] = ypr[1]-Yoffset;
ypr[2] = ypr[2]-Xoffset;
//gyro[0],gyro[1],gyro[2]//x,y,z 각속도값
}
}

/* X-direction P + PID */
void PID_control_x() {
error_x = desired_angle_x - ypr[2]; //angle def
P_angle_pid_x = P_angle_gain_x * error_x; //angle def + outer P control
error_pid_x = P_angle_pid_x - gyro[0]; // Pcontrol_angle - angle rate = PID Goal
P_x = error_pid_x * P_gain_x; // Inner P control
D_x = (error_pid_x - error_pid_x1) / T * D_gain_x; // Inner D control
I_x += (error_pid_x) * T * I_gain_x; // Inner I control
I_x = constrain(I_x, -100, 100); // I control must be limited to prevent being jerk.
PID_x = P_x + D_x + I_x;
front_output = PID_x + throttle;
back_output = -PID_x + throttle;
front_output = constrain(front_output, 100, 255);
back_output = constrain(back_output, 100, 255);
/*v1x=front_output;
v2x=back_output;
v3x=back_output;
v4x=front_output;*/
error_pid_x1 = error_pid_x;
}

/* Y-direction P + PID */
void PID_control_y() {
error_y = desired_angle_y - ypr[1]; //angle def
P_angle_pid_y = P_angle_gain_y * error_y; //angle def + outer P control
error_pid_y = P_angle_pid_y + gyro[1]; // Pcontrol_angle - angle rate = PID Goal
P_y = error_pid_y * P_gain_y; // Inner P control
D_y = (error_pid_y - error_pid_y1) / T * D_gain_y; // Inner D control
I_y += (error_pid_y) * T * I_gain_y; // Inner I control
I_y = constrain(I_y, -100, 100); // I control must be limited to prevent being jerk.
PID_y = P_y + D_y + I_y;
right_output = PID_y + throttle;
left_output = -PID_y + throttle;
right_output = constrain(right_output, 100, 255);
left_output = constrain(left_output, 100, 255);
/* v1y=right_output;
v2y=right_output;
v3y=left_output;
v4y=left_output;*/
error_pid_y1 = error_pid_y;
}

void resultV(){
v1 = (PID_x + PID_y)/2 + throttle;
v2 = (-PID_x + PID_y)/2 + throttle;
v3 = (-PID_x - PID_y)/2 + throttle;
v4 = (PID_x - PID_y)/2 + throttle;
v1=constrain(v1, 200, 255);
v2=constrain(v2, 200, 255);
v3=constrain(v3, 200, 255);
v4=constrain(v4, 200, 255);
}

void serialEvent()
{
// while (bluetooth.available()) {
// throttle_real = bluetooth.read();
// Serial.write(throttle_real);
// bluetooth.write(throttle_real);
// /* Below is to prevent throttle value fucked up */
// if (throttle_real > 255 || throttle_real < 0 ) {
// throttle_real = throttle;
// }
// /* Below is to keep original throttle value when changing direction of drone */
// if (throttle_real > 20 && throttle_real < 255) {
// throttle = throttle_real;
// }
// }
while (bluetooth.available()) { 
bt=bluetooth.read();
Serial.write(bt);
bluetooth.write(bt);
}
while (Serial.available()) { 
bt=Serial.read();
bluetooth.write(bt);
}
// while (Serial.available()) {
// throttle_real = Serial.parseInt();
// /* Below is to prevent throttle value fucked up */
// if (throttle_real > 255 || throttle_real < 0 ) {
// throttle_real = throttle;
// }
// /* Below is to keep original throttle value when changing direction of drone */
// if (throttle_real > 20 && throttle_real < 255) {
// throttle = throttle_real;
// }
// }
/* Serial.print("throttle : ");
Serial.print(throttle);
Serial.print('\t');
Serial.print("throttle_real : ");
Serial.println(throttle_real);*/
}

void loop() {
dmploop(); //refresh new angle datas from MPU6050
serialEvent();
if (bt == 'x' ) {
stopped();
digitalWrite(LED,HIGH);
}
if (bt == 'c' ) {
stopped();
digitalWrite(LED,LOW);
}
if (throttle_real > 10 && throttle_real < 20 ) { // Throttle is too low to be operated. Turn motors off.
stopped();
}
if (bt == 'u' ) { // up
desired_angle_x = -10; //initialize other direction angle
desired_angle_y = 0;
PID_control_x();
PID_control_y();
resultV();
// Serial.println("up");
}
if (throttle_real == 3 ) { // down
desired_angle_x = 10; //initialize other direction angle
desired_angle_y = 0;
PID_control_x();
PID_control_y();
resultV();
//Serial.println("down");
}
if (bt == 'r' ) { // right 
desired_angle_x = 0; //initialize other direction angle
desired_angle_y = -10;
PID_control_x();
PID_control_y();
resultV();
// Serial.println("right");
}
if (bt == 'l' ) { // left 
desired_angle_x = 0 ; //initialize other direction angle
desired_angle_y = 10;
PID_control_x();
PID_control_y();
resultV();
// Serial.println("left");
}
if (bt == 'h' ) { // hovering
desired_angle_x = 0 ;
desired_angle_y = 0;
PID_control_x();
PID_control_y();
resultV();
// Serial.println("hovering");
}
if (throttle_real >= 20) { // If throttle is higher than 900, turn on the motors and begin PID control.
PID_control_x();
PID_control_y();
resultV();
}
// Serial.print("x : ");
Serial.print(ypr[2]);
Serial.print(", ");
Serial.print(ypr[1]);
Serial.print(" ");
// Serial.print(ypr[0]);
// Serial.print(", v1 : ");
Serial.print(v1);
Serial.print(",");
//Serial.print(v2);
//Serial.print(",");
//Serial.print(v3);
//Serial.print(",");
Serial.println(v4);
/*Serial.print(", P_x : ");
Serial.print(P_x);
Serial.print(", D_x : ");
Serial.print(D_x);
Serial.print(", I_x : ");
Serial.print(I_x);
Serial.print(", PID_x : ");
Serial.println(PID_x);*/
/*Serial.print(ypr[1]); //1이 y고, 2가 x다. 우리가 반대로 함.
Serial.print('\t');
Serial.print(right_output);
Serial.print('\t');
Serial.println(left_output);*/
/* Serial.print('\t');
Serial.print(-gyro[1]);
Serial.println('\t');
Serial.print(I_gain_x);
Serial.print('\t');
Serial.print(I_x);
Serial.print('\t');*/
digitalWrite(4,LOW);
analogWrite(5,v1);//~
analogWrite(6,v2);//~
digitalWrite(7,LOW);
digitalWrite(8,LOW);
analogWrite(9,v3);//~
analogWrite(10,v4);//~
digitalWrite(11,LOW);
}
/*void resultV(){
v1=(v1x+v1y)/2;
v2=(v2x+v2y)/2;
v3=(v3x+v3y)/2;
v4=(v4x+v4y)/2;
}*/
void stopped() {
/*front.writeMicroseconds(800);
left.writeMicroseconds(800);
back.writeMicroseconds(800);
right.writeMicroseconds(800);*/
v1=0;
v2=0;
v3=0;
v4=0;
}
void servoset() {
//front.attach(5);//1
//right.attach(6);//2
// back.attach(9);//3
//left.attach(10);//4
//front.writeMicroseconds(800); //Initial settings for each motors should be 800, which is MIN.
//right.writeMicroseconds(800);
// back.writeMicroseconds(800);
//left.writeMicroseconds(800);
pinMode(4, OUTPUT); 
pinMode(5, OUTPUT); //~1
pinMode(6, OUTPUT); //~ 2 
pinMode(7, OUTPUT); 
pinMode(8, OUTPUT); 
pinMode(9, OUTPUT); //~3
pinMode(10, OUTPUT); //~ 4 
pinMode(11, OUTPUT); 
}