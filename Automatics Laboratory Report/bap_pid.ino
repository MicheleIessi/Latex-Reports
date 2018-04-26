#include <Servo.h>
#include <TouchScreen.h>
#include <Math.h>

float ServoX_ang;
float ServoY_ang;
int ServoX_flat = 80; // 80
int ServoY_flat = 87; // 87
int ServoX_val = ServoX_flat;
int ServoY_val = ServoY_flat;

int XSize_pix = 970;
int YSize_pix = 900;
float XSize_m = 0.23;
float YSize_m = 0.19;

float xS = 0;
float yS = 0;
float EMA_a = 0.3;

float board_center_to_servo_center_X = 0.036;
float board_center_to_servo_center_Y = 0.036;
float servo_arm_length_X = 0.015;
float servo_arm_length_Y = 0.015;
float toll = 0;

float board_angX;
float board_angY;
float board_angX_max = atan(servo_arm_length_X / board_center_to_servo_center_X);
float board_angY_max = atan(servo_arm_length_Y / board_center_to_servo_center_Y);

// PID
int error_X, error_Y;
int previous_error_X, previous_error_Y;
float integral_X, integral_Y;
float derivative_X, derivative_Y;
float output_X, output_Y;
float Kp_X = 3.5;
float Ki_X = 0.0001;
float Kd_X = 8.5;
float Ko_X = 4250;

float Kp_Y = 2.25;
float Ki_Y = 0.0001;
float Kd_Y = 8.5;
float Ko_Y = 4250;


float t; // tempo corrente
int Xpix;
int Ypix; 
float Xm;
float Ym;
float Xnorm;
float Ynorm;

float t1; //tempo precedente
int X1pix;
int Y1pix;
float X1;
float Y1;

float t2;
int X2pix;
int Y2pix;
float X2;
float Y2;
float norm = 40;

float velX; // velocità corrente X
float velY; // velocità corrente Y
float velX1;// velocità precedente X
float velY1;// velocità precedente Y
float accX; // accelerazione corrente X
float accY; // accelerazione corrente Y

int Xtarget = 0;
int Ytarget = 0;

float follow_time;
float j = 0;
float k = 0;

Servo ServoX;
Servo ServoY;

#define YP A0
#define XM A1
#define YM 3
#define XP 4
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 90);
TSPoint p;


void setup() {

  ServoX.attach(5);
  ServoY.attach(6);

  ServoX.write(90);

  p = ts.getPoint();
  Serial.begin(115200);
  delay(200);
}

void loop() {


  p = ts.getPoint();

  if(p.x != 0) {

    Xpix = p.x - 500;
    Ypix = p.y - 480;

    xS = (EMA_a * Xpix) + ((1 - EMA_a) * xS);
    yS = (EMA_a * Ypix) + ((1 - EMA_a) * yS);

//    Serial.print(xS);
//    Serial.print(',');
//    Serial.println(yS);
    norm = sqrt(pow(Xtarget - Xpix, 2) + pow(Ytarget - Ypix, 2));
    if(norm > toll) {

      // controllo PID
      t = millis();

      //computeCircleSetpoint();
      //computeLemniscateSetpoint();
      
      Serial.println(norm);
      error_X = Xtarget - Xpix;
      integral_X = integral_X + (error_X * (t - t1));
      derivative_X = (error_X - previous_error_X) / ((t - t1)/100);
      output_X = (Kp_X*error_X) + (Ki_X*integral_X) + (Kd_X*derivative_X);
      previous_error_X = error_X;
  
      error_Y = Ytarget - Ypix;
      integral_Y = integral_Y + (error_Y * (t - t1));
      derivative_Y = (error_Y - previous_error_Y) / ((t - t1)/100);
      output_Y = (Kp_Y*error_Y) + (Ki_Y*integral_Y) + (Kd_Y*derivative_Y);
      previous_error_Y = error_Y;
  
      t1 = t;
  
      board_angX = constrain((-output_X / Ko_X), -board_angX_max, board_angX_max);
      board_angY = constrain((-output_Y / Ko_Y), -board_angY_max, board_angY_max);
  
      ServoX_ang = asin((board_center_to_servo_center_X * tan(board_angX)) / 
            sqrt(pow(servo_arm_length_X, 2) + pow(servo_arm_length_X * tan(board_angX), 2))) - board_angX;
      ServoX_val = int(map(ServoX_ang, -PI/2, PI/2, (ServoX_flat - 90), (90 + ServoX_flat)));
  
      ServoY_ang = asin((board_center_to_servo_center_Y * tan(board_angY)) / 
            sqrt(pow(servo_arm_length_Y, 2) + pow(servo_arm_length_Y * tan(board_angY), 2))) - board_angY;
      ServoY_val = int(map(ServoY_ang, -PI/2, PI/2, (ServoY_flat - 90), (90 + ServoY_flat)));
  
      int angoloX = (int) (ServoX_flat - ServoX_ang*100);
      int angoloY = (int) (ServoY_flat - ServoY_ang*70);
      
      ServoX.write(angoloY);
      ServoY.write(angoloX);
      

      delay(20);
    }
  }
}

void computeCircleSetpoint() {

  j -= PI/50;
  Xtarget = sin(j) * 150;
  Ytarget = cos(j) * 150; 
  
}

void computeLemniscateSetpoint() {
  k -= PI/50;
  Xtarget = 5*(50*cos(k))/(1+sin(k)*sin(k));
  Ytarget = 5*(50*sin(k)*cos(k))/(1+sin(k)*sin(k));
}

