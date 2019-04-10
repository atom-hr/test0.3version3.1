//*****test0.3 version3.1******
//****using 4 sensors**********
//*****************************

#include<QTRSensors.h>

//********defines********
#define indicator_on digitalWrite(13,HIGH)
#define indicator_off digitalWrite(13,LOW)

#define Kp 0.1
#define Kd 2
#define Ki 0

#define RMF 24
#define RMB 25
#define LMF 22
#define LMB 23
#define RME 11
#define LME 12
#define base_motor 150
#define max_speed 255

#define trigPin1 43
#define echoPin1 42
#define trigPin2 39
#define echoPin2 38
#define trigPin3 41
#define echoPin3 40

#define NUM_SENSORS   4
#define TIMEOUT       2500 
#define EMITTER_PIN   2 

QTRSensorsRC qtrrc((unsigned char[]){32,33,34,35},NUM_SENSORS,TIMEOUT,EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void setup(){
  //****setting up pinMode ******
  Serial.begin(9600);

  pinMode(13,OUTPUT);

  pinMode(LMF,OUTPUT);
  pinMode(LMB,OUTPUT);
  pinMode(RMF,OUTPUT);
  pinMode(RMB,OUTPUT);
  pinMode(LME,OUTPUT);
  pinMode(RME,OUTPUT);

  //*****calibration*****
  indicator_off;
  for(int i=0;i<100;i++)
  {
    qtrrc.calibrate();
    digitalWrite(13,HIGH);
    delay(20);
    digitalWrite(13,LOW);
  }
  delay(2000);

  if(true)
  {
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
  }
  indicator_on;
  //******End of calibration********

  digitalWrite(LMF, HIGH); 
  digitalWrite(LMB, LOW);
  digitalWrite(RMF, HIGH);
  digitalWrite(RMB, LOW);

}

//******Global variables*******
int last_error=0;
bool line_follow=false;
int sensors[NUM_SENSORS];
int direc=0;

void loop(){
  int line_position=qtrrc.readLine(sensors);
  int error=line_position-1500;

  if( sensors[0]>200 && sensors[1]>200 && sensors[2]>200 )
  {
    error=error-1000;
  }
  else if( sensors[2]>200 && sensors[3]>200 && sensors[1]>200 )
  {
    error=error+1000;
  }

  int motor_speed = Kp * error + Kd * (error - last_error);
  last_error = error;

  int left_motor=base_motor+motor_speed;
  int right_motor=base_motor-motor_speed;

  if(left_motor>max_speed)left_motor=max_speed;
  else if(left_motor<0)left_motor=0;
  if(right_motor>max_speed)right_motor=max_speed;
  else if(right_motor<0)right_motor=0;

  analogWrite(LME, left_motor);
  analogWrite(RME, right_motor);
}
