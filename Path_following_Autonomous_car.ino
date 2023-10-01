#include <ECE3.h>
#include <stdio.h>


const int Left_Nslp=31; // nslp HIGH ==> awake & ready for PWM
const int Right_Nslp=11; // nslp HIGH ==> awake & ready for PWM
const int Left_Direction=29;
const int Right_Direction=30;
const int Left_Pwm=40;
const int Right_Pwm=39;
const int LED_RED = 75;

//Timer for LED Flashing
int timer = 0;
bool LED_Status = false;
int Old_Encoder_Count = getEncoderCount_left();
int New_Encoder_Count = getEncoderCount_left();

int Base_Speed = 40;
int Max_Speed = 100;
int Donut_Speed = 30;
int Donut_Stop = 355; // The encoder count before we use changebasespeed
//to stop our car and run again.
int Donut_Delay = 50;


//P and D Control Weights
float P_Control_Weight = 0.1;
float D_Control_Weight = 0.3; //The D is calculated in terms of seconds!
float Error_Speed_Transfer = 0.3;

//The control: Negative is to turn left, positive is to turn right.

//The wheel speed for both wheels are stored here. 
struct Wheel_Speed{
  int Left;
  int Right;
  };


float Old_Sensor_Fusion;
float New_Sensor_Fusion;

void  ChangeBaseSpeed(int initialBaseSpd, int finalBaseSpd) {
/*  
 *   This functin changes the car base speed gradually (in about 300 ms) from
 *   initialspeed to final speed. This non-instantaneous speed change reduces the 
 *   load on the plastic geartrain, and reduces the failure rate of the motors. 
 */
  int numSteps = 5;
  int pwmLeftVal = initialBaseSpd; // initialize left wheel speed 
  int pwmRightVal = initialBaseSpd;  // initialize right wheel speed 
  int deltaLeft = (finalBaseSpd-initialBaseSpd)/numSteps; // left in(de)crement
  int deltaRight = (finalBaseSpd-initialBaseSpd)/numSteps;  // right in(de)crement

  for(int k=0;k<numSteps;k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(Left_Pwm,pwmLeftVal);    
    analogWrite(Right_Pwm,pwmRightVal); 
    delay(60);   
  } // end for int k
} // end void  ChangeBaseSpeed

float Array_Min(float Sensor_Values[8]){
  float Running_Min = Sensor_Values[0];
  for(int i = 0; i < 8 ; i++){
    if(Sensor_Values[i] < Running_Min)
      Running_Min = Sensor_Values[i];
    }
   return Running_Min;
   }

void Normalize_Min(float Raw_Sensor[8], float Min_Value){
  for(int i = 0; i < 8; i++){
    Raw_Sensor[i] = Raw_Sensor[i] - Min_Value;
    }
  }

float Array_Max (float Sensor_Values[8]){
  float Running_Max = Sensor_Values[0];
  for(int i = 0; i < 8; i++){
    if(Sensor_Values[i] > Running_Max)
      Running_Max = Sensor_Values[i];
    }
  return Running_Max;
  }

void Normalize_Max1k(float Sensor_Values[8], float Sensor_Max){
  for(int i = 0; i < 8; i++){
    Sensor_Values[i] = Sensor_Values[i]* (1000.0 / Sensor_Max);
    //Remember that, Sensor_values[]is an int array!
    //We lose info here!!!
    //Changed to float type.
    }
  }

  
int End_Count = 0;
bool Reach_End(){
  uint16_t Sensor_Values_Raw[8];
  ECE3_read_IR(Sensor_Values_Raw);
  //We pass the values to an int array.. uint16 look ugly.
  float Sensor_Values[8];
  for(int i = 0; i < 8; i++){
    Sensor_Values[i] = Sensor_Values_Raw[i];
    } 
  //Add a line of code checking for end block...
  if((Sensor_Values[0] > 2400) && (Sensor_Values[1] > 2400) && (Sensor_Values[2] > 2400) &&
  (Sensor_Values[3] > 2400) && (Sensor_Values[4] > 2400) && (Sensor_Values[5] > 2400) && 
  (Sensor_Values[6] > 2400) && (Sensor_Values[7] > 2400)){
    uint16_t Sensor_Values_Raw[8];
  ECE3_read_IR(Sensor_Values_Raw);
  //We pass the values to an int array.. uint16 look ugly.
  float Sensor_Values[8];
  for(int i = 0; i < 8; i++){
    Sensor_Values[i] = Sensor_Values_Raw[i];
    } 
   if((Sensor_Values[0] > 2400) && (Sensor_Values[1] > 2400) && (Sensor_Values[2] > 2400) &&
  (Sensor_Values[3] > 2400) && (Sensor_Values[4] > 2400) && (Sensor_Values[5] > 2400) && 
  (Sensor_Values[6] > 2400) && (Sensor_Values[7] > 2400))
    return true;
    
    }
   return false; 
  }

float Get_Sensor_Fusion(){
  uint16_t Sensor_Values_Raw[8];
  ECE3_read_IR(Sensor_Values_Raw);
  //We pass the values to an int array.. uint16 look ugly.
  float Sensor_Values[8];
  for(int i = 0; i < 8; i++){
    Sensor_Values[i] = Sensor_Values_Raw[i];
    } 

  
  
  //Now we do the normalization we did in the sensor fusion...
  //The [0] is the right most one!
  //Oh shoot we should not care thu, since how the sensor fusion is done...
  //We know when sensor fusion is >0, we are to the right!

  //Each sensor values get minused by the min value of the sensor.
  float Sensor_Min = Array_Min(Sensor_Values);
  Normalize_Min(Sensor_Values, Sensor_Min); 

  //Normalize the array so that the max value is 1000. 
  float Sensor_Max = Array_Max(Sensor_Values);
  Normalize_Max1k(Sensor_Values, Sensor_Max);

  //Hey, a PROblEM:
  //Does int has enough accruacy??? OR I HAVE TO USE float??
  //ASSUMING INT IS ENOUGH

  float Sensor_Fusion = ((-8)*Sensor_Values[0] + (-4)*Sensor_Values[1] + (-2)*Sensor_Values[2]
  + (-1)*Sensor_Values[3] + 1*Sensor_Values[4] + 2*Sensor_Values[5] + 4*Sensor_Values[6] + 8*Sensor_Values[7])/4;

  return Sensor_Fusion;
  
  }


void Fusion_Change_Speed(Wheel_Speed *Wheel_Speed, int Base_Speed, float Total_Error){ 
  //Thoughts: We store the desired speed into the array and change them later in main loop.
  //And we don't change the wheels' directions! Only speed...
  //When it's negative, turn to the right.
  //When it's positive, turn to the left. 
  //The region of total error is about +/- 100.
  
  (*Wheel_Speed).Left = Base_Speed - Total_Error * Error_Speed_Transfer;
  (*Wheel_Speed).Right = Base_Speed + Total_Error * Error_Speed_Transfer;
  
  }

void Donut(int Left_Direction, int Right_Direction, int Donut_Speed){
  ChangeBaseSpeed(Base_Speed, 0);
  digitalWrite(Left_Direction, LOW);
  digitalWrite(Right_Direction, HIGH);
  int Initial_Encoder_Left = getEncoderCount_left();
  ChangeBaseSpeed(0, Donut_Speed);
  int Current_Encoder_Left = getEncoderCount_left();
  while(Current_Encoder_Left - Initial_Encoder_Left <= Donut_Stop){
    Current_Encoder_Left = getEncoderCount_left();
    }
  digitalWrite(Left_Direction, LOW);
  digitalWrite(Right_Direction, LOW);
  ChangeBaseSpeed(0, Base_Speed);
  delay(Donut_Delay);
  
  }


void setup() {
  // put your setup code here, to run once:
  pinMode(Left_Nslp,OUTPUT);
  pinMode(Left_Direction,OUTPUT);
  pinMode(Left_Pwm,OUTPUT);
  pinMode(Right_Nslp,OUTPUT);
  pinMode(Right_Direction,OUTPUT);
  pinMode(Right_Pwm,OUTPUT);
  digitalWrite(Left_Nslp,HIGH);
  digitalWrite(Right_Nslp,HIGH);

  digitalWrite(Left_Direction, LOW); //Set direction to forward
  digitalWrite(Right_Direction, LOW);
  pinMode(LED_RED, OUTPUT);

  digitalWrite(LED_RED, LOW);


  delay(3000);

  ECE3_Init();
  ChangeBaseSpeed(0,Base_Speed);// Start the car up!!!


  Serial.begin(9600);

  Old_Sensor_Fusion = Get_Sensor_Fusion();
}

void loop() {
  // put your main code here, to run repeatedly:

  //The car need to turn LEFT when Fusion is >>0 (P control negative)
  //The car need to turn RIGHT when Fusion is <<0. (P control positive)
  //The idea of PID Control..
  //The P is easy so not again
  //The derivative control is, when the car is moving towards the 0 Fusion,
  //It will go against the change. (This is against the P control.)
  //This can prevent the car from weaving on the straight.
  //Derivative is positive: (D control negative) (Move away to right, or towards target)
  //Derivative is negative: (D control positive) (Move away to left)
  
  
  // Serial.print("End Count Now: "); Serial.println(End_Count);
  
  if(Reach_End() == true && (End_Count == 0)){
   // Serial.println("Doing Donut");
    Donut(Left_Direction, Right_Direction, Donut_Speed);
    End_Count++;
    }
  else if(Reach_End() && End_Count >= 1){
    ChangeBaseSpeed(Base_Speed, 0);
  //  Serial.println("Stopping car.");
    while(1)
      delay(1000);
    }
  
  New_Sensor_Fusion = Get_Sensor_Fusion();
  //Serial.print("Sensor Fusion is "); Serial.println(New_Sensor_Fusion);

  float Fusion_Delta = New_Sensor_Fusion - Old_Sensor_Fusion;

  Old_Sensor_Fusion = New_Sensor_Fusion;
  //Serial.print("The delta of fusion is "); Serial.println(Fusion_Derivative);

  //Thoughts: All the control will turn down to change the wheels' speed.
  //We generate a single signal (change the speed of the left / right wheels), instead of two..
  
  //We MUST first process the P control and the derivative first...
  //Ahh... need too normalize these. 
  //Is that where Kp, Kd come into place? Should be... 
  
  float Total_Error = Old_Sensor_Fusion * P_Control_Weight + Fusion_Delta * D_Control_Weight;

  //When it's negative, turn to the right.
  //When it's positive, turn to the left. 
  //The region of total error is about +/- 100.

  
  
  //Serial.print("The total error is "); Serial.println(Total_Error);
  
  
  Wheel_Speed Wheel_Speed;
  Wheel_Speed.Left = Base_Speed;
  Wheel_Speed.Right = Base_Speed;

  Fusion_Change_Speed(&Wheel_Speed, Base_Speed, Total_Error);
 

  analogWrite(Left_Pwm, Wheel_Speed.Left);
  analogWrite(Right_Pwm, Wheel_Speed.Right);

  
  /********LED FLASHING CODE!!!*******/
  New_Encoder_Count = getEncoderCount_left();
  
  if((millis() - timer > 1000) && (LED_Status == true) && (New_Encoder_Count!=Old_Encoder_Count)){
    digitalWrite(LED_RED, LOW);
    timer = millis();
    LED_Status = false;
    }
  else if((millis() - timer > 2000) && (LED_Status == false) && (New_Encoder_Count!=Old_Encoder_Count)){
    digitalWrite(LED_RED, HIGH);
    timer = millis();
    LED_Status = true;
    }
  
  Old_Encoder_Count = New_Encoder_Count;
  
  
  
}
