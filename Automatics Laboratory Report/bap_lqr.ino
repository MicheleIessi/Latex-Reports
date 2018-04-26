#include <TouchScreen.h>
#include <Servo.h>

int servoX_flat = 80;
int servoY_flat = 87;

float Xpix = 0;
float Ypix = 0;
float xS = 0;
float yS = 0;
float x2S = 0;
float x4S = 0;
float prev_x1 = 0;
float prev_x3 = 0;
float EMA_a = 0.1;
float EMA_b = 0.3;

float x1 = 0;
float x2 = 0;
float x3 = 0;
float x4 = 0;

float u1 = 0;
float u2 = 0;

float k3 = -0.5*8.0178;
float k4 = -0.5*8.8288;
float k1 = 0.5*4.6391;   
float k2 = 0.5*8.1151;

float t;
float t1;

int Xtarget = 0;
int Ytarget = 0;

Servo ServoX;
Servo ServoY;

#define YP A0
#define XM A1
#define YM 3
#define XP 4
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 90);
TSPoint p;

void setup() {
  // put your setup code here, to run once:

  attachServi();
  resetServi();
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:

  p = ts.getPoint();

  if(p.x != 0) {

    Xpix = p.x - 500;
    Ypix = p.y - 520;
    
    xS = (EMA_a * Xpix) + ((1 - EMA_a) * xS);
    yS = (EMA_a * Ypix) + ((1 - EMA_a) * yS);

    t = millis();

    x1 = Xpix/100;
    x2 = (x1 - prev_x1) / (t - t1);
    x3 = Ypix/100;
    x4 = (x3 - prev_x3) / (t - t1);

    //x2S = (EMA_b * x2) + ((1 - EMA_b) * x2S);
    //x4S = (EMA_b * x4) + ((1 - EMA_b) * x4S);

    //x2 = x2S;
    //x4 = x4S;
    
    prev_x1 = x1;
    prev_x3 = x3;
    t1 = t;
    
    u1 = (int) -k1*x1 - k2*x2;
    u2 = (int) -k3*x3 - k4*x4;
    
    Serial.print(Xpix);
    Serial.print(',');
    Serial.println(Ypix);

    ServoY.write(servoY_flat + u1);
    ServoX.write(servoX_flat - u2);
  }
}


void attachServi() {
  ServoX.attach(5);
  ServoY.attach(6);
}

void resetServi() {
  ServoX.write(servoX_flat);
  ServoY.write(servoY_flat);
}

