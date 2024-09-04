/*
______________________________________________________

Jesse's Hexapod Code
______________________________________________________

*/

// TASK: MAKE EACH TRIPOD MOVEMENT SEPERATE

// Libraries
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_LIS3DH.h>
#include <PS2X_lib.h>

// Servo Declerations
Adafruit_PWMServoDriver PWM1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver PWM2 = Adafruit_PWMServoDriver(0x41);

// Accelerometer Declerations
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// Declerations
#define SERVO_FREQ 50         // Analog servos run at ~50 Hz updates

#define TRIG 11               // Port Pin for Trigger
#define ECHO 12               // Port Pin for Echo

#define PS2_CLK 5
#define PS2_CMD 3
#define PS2_ATT 2
#define PS2_DAT 4

// Functions
void setup();
void loop();
void Base(); // Mode 0
void Forward(); // Mode 1
void Backwad(); // Mode 2
void Move_Left(); // Mode 3
void Move_Right(); // Mode 4
void Turn_Left(); // Mode 5
void Turn_Right(); // Mode 6
float ReadDistance();
void Check_Angles();

// Main Variables
int Input = 0;                // Value recieved from Controller
int Mode = 0;                 // These will be for each movement function
float Delay = 0.5;              // Delay in ms for the main loop, the bigger, the slower will the robot move
int Tripod2 = 0;              // To keep the rotation between the legs seperate
int Move1 = 0;                // These are used as counters to count degrees
int Move2 = 0;
int Move3 = 0;
int Move4 = 0;

int Move5 = 0;
int Move6 = 0;
int Move7 = 0;
int Move8 = 0;

int Move9 = 0;
int Move10 = 0;
int Move11 = 0;
int Move12 = 0;

int Move13 = 0;
int Move14 = 0;
int Move15 = 0;
int Move16 = 0;

int Move17 = 0;
int Move18 = 0;
int Move19 = 0;
int Move20 = 0;

int Move21 = 0;
int Move22 = 0;
int Move23 = 0;
int Move24 = 0;

int A_Lift = 0;

// Sensors
float i_Distance = 0;
float o_Distance = 0;
float PulseTime = 0;

// Accelerometer
int X = lis.x;
int Y = lis.y;
int Z = lis.z;
float AccX;
float AccY;
float AccZ;
float AngleRoll;
float AnglePitch;

// Gait
int Gait = 0;

// ObserveData
int Observe = 2;              // This is for the Serial Monitor and Plotter
int Count = 0;

// Controller
PS2X ps2x;
int error = 0; 
byte type = 0;
byte vibrate = 0;

// Legs and Motors
int adjust = 0; //50;
int L1_M1 = 315; // higher == forward, lower == backward
int L1_M2 = 380 - adjust; // higher == lower, lower == higher
int L1_M3 = 390 - adjust; // higher == outward, lower == inward

int L2_M1 = 335; // higher == forward, lower == backward
int L2_M2 = 290 + adjust; // higher == higher, lower == lower
int L2_M3 = 290 + adjust; // higher == inward, lower == outward

int L3_M1 = 335; // higher == forward, lower == backward
int L3_M2 = 315 + adjust; // higher == higher, lower == lower
int L3_M3 = 280 + adjust; // higher == inward, lower == outward

int L4_M1 = 300; // higher == forward, lower == backward
int L4_M2 = 290 + adjust; // higher == higher, lower == lower
int L4_M3 = 280 + adjust; // higher == inward, lower == outward

int L5_M1 = 300; // higher == backward, lower == forward
int L5_M2 = 390 - adjust; // higher == lower, lower == higher
int L5_M3 = 360 - adjust; // higher == outward, lower == inward

int L6_M1 = 310; // higher == backward, lower == forward
int L6_M2 = 350 - adjust; // higher == lower, lower == higher
int L6_M3 = 350 - adjust; // higher == outward, lower == inward


void setup()
{
  Serial.begin(9600);

  // Controller
  ControllerSetup();

  // Servos
  PWM1.begin();
  PWM1.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  PWM2.begin();
  PWM2.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  // Ultrasonic Sensors
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // Accelerometer
  lis.begin(); // 0x18
  lis.setRange(LIS3DH_RANGE_2_G); // 2G
  lis.setDataRate(LIS3DH_DATARATE_400_HZ);

  // Startup LED
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.

  Base();
  delay(1000); // Waits 1 Second.
}

void ControllerSetup()
{
  error = ps2x.config_gamepad(PS2_CLK,PS2_CMD,PS2_ATT,PS2_DAT,true,true);

  if(error == 0)
  {
    Serial.println("Found Controller, configured successful");
    Serial.println("Go to www.billporter.info for updates and to report bugs.");
  }
  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  
  type = ps2x.readType();
  switch(type)
  {
    case 0:
      Serial.println("Unknown Controller type");
      break;
    case 1:
      Serial.println("DualShock Controller Found");
      break;
    case 2:
      Serial.println("GuitarHero Controller Found");
      break;
  }
}

  int Change = 0;
void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED ON to indicate the code is running

  ps2x.read_gamepad(false, vibrate);

  //PitchCorLeg3();
  ObserveData();

  // Will change to a movement function depending on the received input
  if (ps2x.NewButtonState())
  {
    if (ps2x.Button(PSB_PAD_UP)) // 
    {
      if (Gait == 1)
      {
        Serial.println("a");
        Mode = 7;
      }
      else if (Gait == 2)
      {
        Serial.println("b");
        Mode = 13;
      }
      else
      {
        Serial.println("c");
        Mode = 1;
      }
    }
    else if (ps2x.Button(PSB_PAD_DOWN))
    {
      if (Gait == 1)
      {
        Mode = 8;
      }
      else if (Gait == 2)
      {
        Mode = 14;
      }
      else
      {
        Mode = 2;
      }
    }
    else if (ps2x.Button(PSB_PAD_LEFT))
    {
      if (Gait == 1)
      {
        Mode = 9;
      }
      else if (Gait == 2)
      {
        Mode = 15;
      }
      else
      {
        Mode = 3;
      }
    }
    else if (ps2x.Button(PSB_PAD_RIGHT))
    {
      if (Gait == 1)
      {
        Mode = 10;
      }
      else if (Gait == 2)
      {
        Mode = 16;
      }
      else
      {
        Mode = 4;
      }
    }
    else if (ps2x.Button(PSB_L1))
    {
      if (Gait == 1)
      {
        Mode = 11;
      }
      else if (Gait == 2)
      {
        Mode = 17;
      }
      else
      {
        Mode = 5;
      }
    }
    else if (ps2x.Button(PSB_R1))
    {
      if (Gait == 1)
      {
        Mode = 12;
      }
      else if (Gait == 2)
      {
        Mode = 18;
      }
      else
      {
        Mode = 6;
      }
    }
    else if (ps2x.ButtonPressed(PSB_PINK))
    {
      Gait = 1;
      if (Gait == 1)
      {
        Serial.println("Ripple Gait");     
      }
    }
    else if (ps2x.ButtonPressed(PSB_GREEN))
    {
      Gait = 0;
      if (Gait == 0)
      {
        Serial.println("Tripod Gait");     
      }
    }
    else if (ps2x.ButtonPressed(PSB_RED))
    {
      Gait = 2;
      if (Gait == 2)
      {
        Serial.println("Wave Gait");     
      }
    }
    else if (ps2x.ButtonPressed(PSB_SELECT))
    {
      Change++;
      if (Change > 2)
      {
        Change = 0;
      }
      if (Change == 1)
      {
      Delay = 0.5;
      Serial.print("Delay Speed "); Serial.print(Delay); Serial.println("m/s^2");
      }
      else if (Change == 2)
      {
        Delay = 10;
        Serial.print("Delay Speed "); Serial.print(Delay); Serial.println("m/s^2");
      }
      else if (Change == 0)
      {
        Delay = 5;
        Serial.print("Delay Speed "); Serial.print(Delay); Serial.println("m/s^2");
      }
    }
    else if (ps2x.Button(PSB_BLUE))
    {
      Serial.println("R2 pressed");
      Mode = 19;
    }
  }
  else if (!ps2x.Button(PSB_PAD_UP) && !ps2x.Button(PSB_PAD_DOWN) && !ps2x.Button(PSB_PAD_LEFT) && !ps2x.Button(PSB_PAD_RIGHT) && !ps2x.Button(PSB_L1) && !ps2x.Button(PSB_R1))
  {
    delay(5);
    Mode = 0;               // Starts at base position
    Move1 = 0;                // These are used as counters to count degrees
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
    Move5 = 0;
    Move6 = 0;
    Move7 = 0;
    Move8 = 0;
    Move9 = 0;
    Move10 = 0;
    Move11 = 0;
    Move12 = 0;
    Move13 = 0;
    Move14 = 0;
    Move15 = 0;
    Move16 = 0;
    Move17 = 0;
    Move18 = 0;
    Move19 = 0;
    Move20 = 0;
    Move21 = 0;
    Move22 = 0;
    Move23 = 0;
    Move24 = 0;
  }

  // Base Position
  if (Mode == 0)
  {
    Base();
  }

  // Move Forward
  if (Mode == 1)
  {
    Forward();
  }

  // Move Backward
  if (Mode == 2)
  {
    Backward();
  }

  // Move Left
  if (Mode == 3)
  {
    Move_Left();
  }

  // Move Right
  if (Mode == 4)
  {
    Move_Right();
  }

  // Turn Left
  if (Mode == 5)
  {
    Turn_Left();
  }

  // Turn Right
  if (Mode == 6)
  {
    Turn_Right();
  }

// ---------- Ripple Gait --------------- //

  // Move Forward
  if (Mode == 7)
  {
    Forward_Ripple();
  }

  // Move Backward
  if (Mode == 8)
  {
    Backward_Ripple();
  }

  // Move Left
  if (Mode == 9)
  {
    Move_Left_Ripple();
  }

  // Move Right
  if (Mode == 10)
  {
    Move_Right_Ripple();
  }

  // Turn Left
  if (Mode == 11)
  {
    Turn_Left_Ripple();
  }

  // Turn Right
  if (Mode == 12)
  {
    Turn_Right_Ripple();
  }

  // ---------- Wave Gait --------------- //

  // Move Forward
  if (Mode == 13)
  {
    Forward_Wave();
  }

  // Move Backward
  if (Mode == 14)
  {
    Backward_Wave();
  }

  // Move Left
  if (Mode == 15)
  {
    Move_Left_Wave();
  }

  // Move Right
  if (Mode == 16)
  {
    Move_Right_Wave();
  }

  // Turn Left
  if (Mode == 17)
  {
    Turn_Left_Wave();
  }

  // Turn Right
  if (Mode == 18)
  {
    Turn_Right_Wave();
  }

  // Rise
  if (Mode == 19)
  {
    RiseUp();
  }

  delay(Delay); // This controls the speed.
}

void ObserveData()
{
  if (Count == 5)
  {
    if (Observe == 1)
    {
      o_Distance = ReadDistance();      // Takes in the distance of objects.
      Serial.print("Distance = ");
      Serial.println(o_Distance);
    }
    else if (Observe == 2)
    {
      ProcessAccelData();
      Serial.print("\t\tRoll Angle = "); // Y
      Serial.print(AngleRoll);
      Serial.print(" \tPitch Angle = "); // X
      Serial.println(AnglePitch);
    }
    else if (Observe == 3)
    {
      ProcessAccelData();
      Serial.print("\t\tX: "); Serial.print(AccX);
      Serial.print(" \tY: "); Serial.print(AccY);
      Serial.print(" \tZ: "); Serial.print(AccZ);
      Serial.println(" m/s^2 ");
    }
    if (Observe == 4)
    {
      
    }
    Count = 0;
  }
  Count++;
  //Serial.println(Count);
}

void ProcessAccelData()
{
  lis.read();                       // get X Y and Z data at once
  sensors_event_t event;
  lis.getEvent(&event);

  AccX = event.acceleration.x - 0.6; // This takes out the erorrs
  AccY = event.acceleration.y - 0.1;
  AccZ = event.acceleration.z - 0.5;

  AngleRoll = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ))*1/(PI/180);
  AnglePitch = -atan(AccX/sqrt(AccY*AccY + AccZ*AccZ))*1/(PI/180);
}

//
void PitchCorLeg3()
{
  if (AnglePitch > 12 || AnglePitch < -12)
  {
    PWM1.setPWM(9, 0, L3_M2 + AnglePitch); 
    PWM1.setPWM(3, 0, L3_M3 + AnglePitch); 
  }
}
/*
void RollCorLeg3()
{
  if (AngleRoll > 2 || AngleRoll < -2)
  {
    PWM1.setPWM(9, 0, L3_M2 + AngleRoll); 
    PWM1.setPWM(3, 0, L3_M3 + AngleRoll); 
  }
}
*/

float ReadDistance()
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(5);

  digitalWrite(TRIG,LOW);

  PulseTime = pulseIn(ECHO, HIGH);
  i_Distance = PulseTime * 0.017015;

  return i_Distance;
}

void RiseUp()
{
  if (adjust > 50)
  {

  }
  else
  {
    // Leg 1
  PWM1.setPWM(1, 0, L1_M2 - adjust);
  PWM1.setPWM(2, 0, L1_M3 - adjust);

  // Leg 2
  PWM1.setPWM(5, 0, L2_M2 + adjust);
  PWM1.setPWM(6, 0, L2_M3 + adjust);

  // Leg 3
  PWM1.setPWM(9, 0, L3_M2 + adjust);
  PWM1.setPWM(3, 0, L3_M3 + adjust);

  // Leg 4
  PWM2.setPWM(1, 0, L4_M2 + adjust);
  PWM2.setPWM(2, 0, L4_M3 + adjust);

  // Leg 5
  PWM2.setPWM(5, 0, L5_M2 - adjust);
  PWM2.setPWM(6, 0, L5_M3 - adjust);

  // Leg 6
  PWM2.setPWM(9, 0, L6_M2 - adjust);
  PWM2.setPWM(10, 0, L6_M3 - adjust);
    adjust++;
  }
}

void Base()
{
  // Leg 1
  PWM1.setPWM(0, 0, L1_M1); 
  PWM1.setPWM(1, 0, L1_M2);
  PWM1.setPWM(2, 0, L1_M3);

  // Leg 2
  PWM1.setPWM(4, 0, L2_M1);
  PWM1.setPWM(5, 0, L2_M2);
  PWM1.setPWM(6, 0, L2_M3);

  // Leg 3
  PWM1.setPWM(8, 0, L3_M1);
  PWM1.setPWM(9, 0, L3_M2);
  PWM1.setPWM(3, 0, L3_M3);

  // Leg 4
  PWM2.setPWM(0, 0, L4_M1);
  PWM2.setPWM(1, 0, L4_M2);
  PWM2.setPWM(2, 0, L4_M3);

  // Leg 5
  PWM2.setPWM(4, 0, L5_M1);
  PWM2.setPWM(5, 0, L5_M2);
  PWM2.setPWM(6, 0, L5_M3);

  // Leg 6
  PWM2.setPWM(8, 0, L6_M1);
  PWM2.setPWM(9, 0, L6_M2);
  PWM2.setPWM(10, 0, L6_M3);
}

void Forward()
{
  // Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 - Move1); 
    PWM1.setPWM(2, 0, L1_M3 - Move1); 
    PWM1.setPWM(9, 0, L3_M2 + Move1); 
    PWM1.setPWM(3, 0, L3_M3 + Move1); 
    PWM2.setPWM(5, 0, L5_M2 - Move1); 
    PWM2.setPWM(6, 0, L5_M3 - Move1); 
    Move1++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move2 <= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 - Move2); 
    PWM1.setPWM(8, 0, L3_M1 + Move2); 
    PWM2.setPWM(4, 0, L5_M1 - Move2); 
    Move2++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 + Move3);
    PWM1.setPWM(2, 0, L1_M3 + Move3);
    PWM1.setPWM(9, 0, L3_M2 - Move3); 
    PWM1.setPWM(3, 0, L3_M3 - Move3);
    PWM2.setPWM(5, 0, L5_M2 + Move3); 
    PWM2.setPWM(6, 0, L5_M3 + Move3); 
    Move3++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move3 >= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 + Move4); 
    PWM1.setPWM(8, 0, L3_M1 - Move4); 
    PWM2.setPWM(4, 0, L5_M1 + Move4); 
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 + Move5);
      PWM1.setPWM(6, 0, L2_M3 + Move5);
      PWM2.setPWM(1, 0, L4_M2 + Move5); 
      PWM2.setPWM(2, 0, L4_M3 + Move5);
      PWM2.setPWM(9, 0, L6_M2 - Move5); 
      PWM2.setPWM(10, 0, L6_M3 - Move5); 
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 + Move6);
      PWM2.setPWM(0, 0, L4_M1 + Move6);
      PWM2.setPWM(8, 0, L6_M1 - Move6);
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 - Move7);
      PWM1.setPWM(6, 0, L2_M3 - Move7);
      PWM2.setPWM(1, 0, L4_M2 - Move7); 
      PWM2.setPWM(2, 0, L4_M3 - Move7);
      PWM2.setPWM(9, 0, L6_M2 + Move7); 
      PWM2.setPWM(10, 0, L6_M3 + Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 - Move8);
      PWM2.setPWM(0, 0, L4_M1 - Move8);
      PWM2.setPWM(8, 0, L6_M1 + Move8);
      Move8++;    
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Tripod2 = 0;
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
}

void Backward()
{
  // Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 - Move1); 
    PWM1.setPWM(2, 0, L1_M3 - Move1); 
    PWM1.setPWM(9, 0, L3_M2 + Move1); 
    PWM1.setPWM(3, 0, L3_M3 + Move1); 
    PWM2.setPWM(5, 0, L5_M2 - Move1); 
    PWM2.setPWM(6, 0, L5_M3 - Move1); 
    Move1++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move2 <= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 + Move2); 
    PWM1.setPWM(8, 0, L3_M1 - Move2); 
    PWM2.setPWM(4, 0, L5_M1 + Move2); 
    Move2++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 + Move3);
    PWM1.setPWM(2, 0, L1_M3 + Move3);
    PWM1.setPWM(9, 0, L3_M2 - Move3); 
    PWM1.setPWM(3, 0, L3_M3 - Move3);
    PWM2.setPWM(5, 0, L5_M2 + Move3); 
    PWM2.setPWM(6, 0, L5_M3 + Move3); 
    Move3++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move3 >= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 - Move4); 
    PWM1.setPWM(8, 0, L3_M1 + Move4); 
    PWM2.setPWM(4, 0, L5_M1 - Move4); 
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 + Move5);
      PWM1.setPWM(6, 0, L2_M3 + Move5);
      PWM2.setPWM(1, 0, L4_M2 + Move5); 
      PWM2.setPWM(2, 0, L4_M3 + Move5);
      PWM2.setPWM(9, 0, L6_M2 - Move5); 
      PWM2.setPWM(10, 0, L6_M3 - Move5); 
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 - Move6);
      PWM2.setPWM(0, 0, L4_M1 - Move6);
      PWM2.setPWM(8, 0, L6_M1 + Move6);
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 - Move7);
      PWM1.setPWM(6, 0, L2_M3 - Move7);
      PWM2.setPWM(1, 0, L4_M2 - Move7); 
      PWM2.setPWM(2, 0, L4_M3 - Move7);
      PWM2.setPWM(9, 0, L6_M2 + Move7); 
      PWM2.setPWM(10, 0, L6_M3 + Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 + Move8);
      PWM2.setPWM(0, 0, L4_M1 + Move8);
      PWM2.setPWM(8, 0, L6_M1 - Move8);
      Move8++;    
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Tripod2 = 0;
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
}

void Move_Right()
{
// Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 - Move1); 
    PWM1.setPWM(2, 0, L1_M3 - Move1); 
    PWM2.setPWM(6, 0, L5_M3 - Move1); 
    PWM2.setPWM(5, 0, L5_M2 - Move1);
    PWM1.setPWM(9, 0, L3_M2 + Move1); 
    Move1++;
  }  
      
  // Legs turn backwards by -40 deg
  if (Move2 <= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 + Move2);
    PWM2.setPWM(4, 0, L5_M1 - Move2);
    PWM1.setPWM(3, 0, L3_M3 - Move2 + 10);
    Move2++;
  }
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 + Move3);
    PWM1.setPWM(2, 0, L1_M3 + Move3);
    PWM2.setPWM(6, 0, L5_M3 + Move3);
    PWM2.setPWM(5, 0, L5_M2 + Move3);
    PWM1.setPWM(9, 0, L3_M2 - Move3); 
    Move3++;
  } 
  
  // Legs turn forwards by 40 deg
  if (Move3 >= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 - Move4);
    PWM2.setPWM(4, 0, L5_M1 + Move4);
    PWM1.setPWM(3, 0, L3_M3 + Move4 - 10);
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 + Move5);
      PWM1.setPWM(6, 0, L2_M3 + Move5);
      PWM2.setPWM(1, 0, L4_M2 + Move5); 
      PWM2.setPWM(2, 0, L4_M3 + Move5);
      PWM2.setPWM(9, 0, L6_M2 - Move5); 
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 + Move6);
      PWM2.setPWM(0, 0, L4_M1 - Move6);
      PWM2.setPWM(10, 0, L6_M3 - Move6 + 10); 
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 - Move7);
      PWM1.setPWM(6, 0, L2_M3 - Move7);
      PWM2.setPWM(1, 0, L4_M2 - Move7); 
      PWM2.setPWM(2, 0, L4_M3 - Move7);
      PWM2.setPWM(9, 0, L6_M2 + Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 - Move8);
      PWM2.setPWM(0, 0, L4_M1 + Move8);
      PWM2.setPWM(10, 0, L6_M3 + Move8 - 10); 
      Move8++;    
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Tripod2 = 0;
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
}

void Move_Left()
{
// Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 - Move1); 
    PWM1.setPWM(2, 0, L1_M3 - Move1); 
    PWM1.setPWM(9, 0, L3_M2 + Move1); 
    PWM2.setPWM(6, 0, L5_M3 - Move1); 
    PWM2.setPWM(5, 0, L5_M2 - Move1);
    Move1++;
  }  
      
  // Legs turn backwards by -40 deg
  if (Move2 <= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 - Move2);
    PWM2.setPWM(4, 0, L5_M1 + Move2);
    PWM1.setPWM(3, 0, L3_M3 + Move2 - 10);
    Move2++;
  }
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 + Move3);
    PWM1.setPWM(2, 0, L1_M3 + Move3);
    PWM1.setPWM(9, 0, L3_M2 - Move3); 
    PWM2.setPWM(6, 0, L5_M3 + Move3);
    PWM2.setPWM(5, 0, L5_M2 + Move3);
    Move3++;
  } 
  
  // Legs turn forwards by 40 deg
  if (Move3 >= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 + Move4);
    PWM2.setPWM(4, 0, L5_M1 - Move4);
    PWM1.setPWM(3, 0, L3_M3 - Move4 + 10);
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 + Move5);
      PWM1.setPWM(6, 0, L2_M3 + Move5);
      PWM2.setPWM(1, 0, L4_M2 + Move5); 
      PWM2.setPWM(2, 0, L4_M3 + Move5);
      PWM2.setPWM(9, 0, L6_M2 - Move5); 
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 - Move6);
      PWM2.setPWM(0, 0, L4_M1 + Move6);
      PWM2.setPWM(10, 0, L6_M3 + Move6 - 10); 
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 - Move7);
      PWM1.setPWM(6, 0, L2_M3 - Move7);
      PWM2.setPWM(1, 0, L4_M2 - Move7); 
      PWM2.setPWM(2, 0, L4_M3 - Move7);
      PWM2.setPWM(9, 0, L6_M2 + Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 + Move8);
      PWM2.setPWM(0, 0, L4_M1 - Move8);
      PWM2.setPWM(10, 0, L6_M3 - Move8 + 10); 
      Move8++;    
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Tripod2 = 0;
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
}

void Turn_Left()
{
  // Leg Lift 20 deg
  if (Move1 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 - Move1); 
    PWM1.setPWM(2, 0, L1_M3 - Move1); 
    PWM1.setPWM(9, 0, L3_M2 + Move1); 
    PWM1.setPWM(3, 0, L3_M3 + Move1); 
    PWM2.setPWM(5, 0, L5_M2 - Move1); 
    PWM2.setPWM(6, 0, L5_M3 - Move1); 
    Move1++;
  }  
      
  // Turn Forwards 40 deg
  if (Move2 <= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 + Move2); 
    PWM1.setPWM(8, 0, L3_M1 + Move2); 
    PWM2.setPWM(4, 0, L5_M1 + Move2); 
    Move2++;
  }
  
  // Touch Ground -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 + Move3);
    PWM1.setPWM(2, 0, L1_M3 + Move3);
    PWM1.setPWM(9, 0, L3_M2 - Move3); 
    PWM1.setPWM(3, 0, L3_M3 - Move3);
    PWM2.setPWM(5, 0, L5_M2 + Move3); 
    PWM2.setPWM(6, 0, L5_M3 + Move3); 
    Move3++;
  } 
  
  // Turn Backwards 40 deg  
  if (Move3 >= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 - Move4); 
    PWM1.setPWM(8, 0, L3_M1 - Move4); 
    PWM2.setPWM(4, 0, L5_M1 - Move4); 
    Move4++;
    Tripod2 = 1;      
  }

  // Reset + Tripod2 ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Leg Lift 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 + Move5);
      PWM1.setPWM(6, 0, L2_M3 + Move5);
      PWM2.setPWM(1, 0, L4_M2 + Move5); 
      PWM2.setPWM(2, 0, L4_M3 + Move5);
      PWM2.setPWM(9, 0, L6_M2 - Move5); 
      PWM2.setPWM(10, 0, L6_M3 - Move5); 
      Move5++;
    }

    // Leg Turns Forwards 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 + Move6);
      PWM2.setPWM(0, 0, L4_M1 + Move6);
      PWM2.setPWM(8, 0, L6_M1 + Move6);
      Move6++;
    }
  
    // Touch Ground -10 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 - Move7);
      PWM1.setPWM(6, 0, L2_M3 - Move7);
      PWM2.setPWM(1, 0, L4_M2 - Move7); 
      PWM2.setPWM(2, 0, L4_M3 - Move7);
      PWM2.setPWM(9, 0, L6_M2 + Move7); 
      PWM2.setPWM(10, 0, L6_M3 + Move7); 
      Move7++;
    } 
  
    // Leg Turns Backwards 40 deg  
    if (Move7 >= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 - Move8);
      PWM2.setPWM(0, 0, L4_M1 - Move8);
      PWM2.setPWM(8, 0, L6_M1 - Move8);
      Move8++;    
    }

    // Reset
    if(Move8 >= 40)
    {
      Tripod2 = 0;
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
}

void Turn_Right()
{
  // Leg Lift 20 deg
  if (Move1 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 - Move1); 
    PWM1.setPWM(2, 0, L1_M3 - Move1); 
    PWM1.setPWM(9, 0, L3_M2 + Move1); 
    PWM1.setPWM(3, 0, L3_M3 + Move1); 
    PWM2.setPWM(5, 0, L5_M2 - Move1); 
    PWM2.setPWM(6, 0, L5_M3 - Move1); 
    Move1++;
  }  
      
  // Turn Forwards 40 deg
  if (Move2 <= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 - Move2); 
    PWM1.setPWM(8, 0, L3_M1 - Move2); 
    PWM2.setPWM(4, 0, L5_M1 - Move2); 
    Move2++;
  }
  
  // Touch Ground -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 + Move3);
    PWM1.setPWM(2, 0, L1_M3 + Move3);
    PWM1.setPWM(9, 0, L3_M2 - Move3); 
    PWM1.setPWM(3, 0, L3_M3 - Move3);
    PWM2.setPWM(5, 0, L5_M2 + Move3); 
    PWM2.setPWM(6, 0, L5_M3 + Move3); 
    Move3++;
  } 
  
  // Turn Backwards 40 deg  
  if (Move3 >= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 + Move4); 
    PWM1.setPWM(8, 0, L3_M1 + Move4); 
    PWM2.setPWM(4, 0, L5_M1 + Move4); 
    Move4++;
    Tripod2 = 1;      
  }

  // Reset + Tripod2 ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Leg Lift 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 + Move5);
      PWM1.setPWM(6, 0, L2_M3 + Move5);
      PWM2.setPWM(1, 0, L4_M2 + Move5); 
      PWM2.setPWM(2, 0, L4_M3 + Move5);
      PWM2.setPWM(9, 0, L6_M2 - Move5); 
      PWM2.setPWM(10, 0, L6_M3 - Move5); 
      Move5++;
    }

    // Leg Turns Forwards 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 - Move6);
      PWM2.setPWM(0, 0, L4_M1 - Move6);
      PWM2.setPWM(8, 0, L6_M1 - Move6);
      Move6++;
    }
  
    // Touch Ground -10 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 - Move7);
      PWM1.setPWM(6, 0, L2_M3 - Move7);
      PWM2.setPWM(1, 0, L4_M2 - Move7); 
      PWM2.setPWM(2, 0, L4_M3 - Move7);
      PWM2.setPWM(9, 0, L6_M2 + Move7); 
      PWM2.setPWM(10, 0, L6_M3 + Move7); 
      Move7++;
    } 
  
    // Leg Turns Backwards 40 deg  
    if (Move7 >= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 + Move8);
      PWM2.setPWM(0, 0, L4_M1 + Move8);
      PWM2.setPWM(8, 0, L6_M1 + Move8);
      Move8++;    
    }

    // Reset
    if(Move8 >= 40)
    {
      Tripod2 = 0;
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
}

//-------------------------------- RIPPLE -------------------------------//

void Forward_Ripple()
{
  if (Tripod2 == 0)
  {
  // Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM2.setPWM(1, 0, L4_M2 + Move1); 
    PWM2.setPWM(2, 0, L4_M3 + Move1); 
    PWM2.setPWM(9, 0, L6_M2 - Move1); 
    PWM2.setPWM(10, 0, L6_M3 - Move1); 
    Move1++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move2 <= 40)
  {
    PWM2.setPWM(0, 0, L4_M1 + Move2); 
    PWM2.setPWM(8, 0, L6_M1 - Move2); 
    Move2++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM2.setPWM(1, 0, L4_M2 - Move1); 
    PWM2.setPWM(2, 0, L4_M3 - Move1); 
    PWM2.setPWM(9, 0, L6_M2 + Move1); 
    PWM2.setPWM(10, 0, L6_M3 + Move1); 
    Move3++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move3 >= 40)
  {
    PWM2.setPWM(0, 0, L4_M1 - Move2); 
    PWM2.setPWM(8, 0, L6_M1 + Move2); 
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 + Move5);
      PWM1.setPWM(3, 0, L3_M3 + Move5);
      PWM1.setPWM(1, 0, L1_M2 - Move5); 
      PWM1.setPWM(2, 0, L1_M3 - Move5);
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(8, 0, L3_M1 + Move6);
      PWM1.setPWM(0, 0, L1_M1 - Move6);
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(1, 0, L1_M2 + Move7); 
      PWM1.setPWM(2, 0, L1_M3 + Move7);
      PWM1.setPWM(9, 0, L3_M2 - Move7); 
      PWM1.setPWM(3, 0, L3_M3 - Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(0, 0, L1_M1 + Move8);
      PWM1.setPWM(8, 0, L3_M1 - Move8);
      Move8++;    
      Tripod2 = 2;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 2)
  {
    // Legs Lift by 20 deg
    if (Move9 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 + Move9);
      PWM1.setPWM(6, 0, L2_M3 + Move9);
      PWM2.setPWM(5, 0, L5_M2 - Move9); 
      PWM2.setPWM(6, 0, L5_M3 - Move9);
      Move9++;
    }

    // Legs turns forwards by 40 deg
    if (Move10 <= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 + Move10);
      PWM2.setPWM(4, 0, L5_M1 - Move10);
      Move10++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move10 > 40 && Move11 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 - Move11); 
      PWM1.setPWM(6, 0, L2_M3 - Move11);
      PWM2.setPWM(5, 0, L5_M2 + Move11); 
      PWM2.setPWM(6, 0, L5_M3 + Move11); 
      Move11++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move11 >= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 - Move12);
      PWM2.setPWM(4, 0, L5_M1 + Move12);
      Move12++;    
      Tripod2 = 0;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move12 >= 40)
    {
      Move9 = 0;
      Move10 = 0;
      Move11 = 0;
      Move12 = 0;
    }    
  }
}

void Backward_Ripple()
{
  if (Tripod2 == 0)
  {
  // Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM2.setPWM(1, 0, L4_M2 + Move1); 
    PWM2.setPWM(2, 0, L4_M3 + Move1); 
    PWM2.setPWM(9, 0, L6_M2 - Move1); 
    PWM2.setPWM(10, 0, L6_M3 - Move1); 
    Move1++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move2 <= 40)
  {
    PWM2.setPWM(0, 0, L4_M1 - Move2); 
    PWM2.setPWM(8, 0, L6_M1 + Move2); 
    Move2++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM2.setPWM(1, 0, L4_M2 - Move1); 
    PWM2.setPWM(2, 0, L4_M3 - Move1); 
    PWM2.setPWM(9, 0, L6_M2 + Move1); 
    PWM2.setPWM(10, 0, L6_M3 + Move1); 
    Move3++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move3 >= 40)
  {
    PWM2.setPWM(0, 0, L4_M1 + Move2); 
    PWM2.setPWM(8, 0, L6_M1 - Move2); 
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 + Move5);
      PWM1.setPWM(3, 0, L3_M3 + Move5);
      PWM1.setPWM(1, 0, L1_M2 - Move5); 
      PWM1.setPWM(2, 0, L1_M3 - Move5);
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(8, 0, L3_M1 - Move6);
      PWM1.setPWM(0, 0, L1_M1 + Move6);
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(1, 0, L1_M2 + Move7); 
      PWM1.setPWM(2, 0, L1_M3 + Move7);
      PWM1.setPWM(9, 0, L3_M2 - Move7); 
      PWM1.setPWM(3, 0, L3_M3 - Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(0, 0, L1_M1 - Move8);
      PWM1.setPWM(8, 0, L3_M1 + Move8);
      Move8++;    
      Tripod2 = 2;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 2)
  {
    // Legs Lift by 20 deg
    if (Move9 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 + Move9);
      PWM1.setPWM(6, 0, L2_M3 + Move9);
      PWM2.setPWM(5, 0, L5_M2 - Move9); 
      PWM2.setPWM(6, 0, L5_M3 - Move9);
      Move9++;
    }

    // Legs turns forwards by 40 deg
    if (Move10 <= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 - Move10);
      PWM2.setPWM(4, 0, L5_M1 + Move10);
      Move10++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move10 > 40 && Move11 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 - Move11); 
      PWM1.setPWM(6, 0, L2_M3 - Move11);
      PWM2.setPWM(5, 0, L5_M2 + Move11); 
      PWM2.setPWM(6, 0, L5_M3 + Move11); 
      Move11++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move11 >= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 + Move12);
      PWM2.setPWM(4, 0, L5_M1 - Move12);
      Move12++;    
      Tripod2 = 0;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move12 >= 40)
    {
      Move9 = 0;
      Move10 = 0;
      Move11 = 0;
      Move12 = 0;
    }    
  }
}

void Move_Left_Ripple()
{
  if (Tripod2 == 0)
  {
  // Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM2.setPWM(1, 0, L4_M2 + Move1); 
    PWM2.setPWM(2, 0, L4_M3 + Move1); 
    PWM2.setPWM(9, 0, L6_M2 - Move1); 
    Move1++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move2 <= 40)
  {
    PWM2.setPWM(0, 0, L4_M1 + Move2); 
    PWM2.setPWM(10, 0, L6_M3 + Move2 - 10); 
    Move2++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM2.setPWM(1, 0, L4_M2 - Move1); 
    PWM2.setPWM(2, 0, L4_M3 - Move1); 
    PWM2.setPWM(9, 0, L6_M2 + Move1); 
    Move3++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move3 >= 40)
  {
    PWM2.setPWM(0, 0, L4_M1 - Move2); 
    PWM2.setPWM(10, 0, L6_M3 - Move2 + 10); 
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 + Move5);
      PWM1.setPWM(1, 0, L1_M2 - Move5); 
      PWM1.setPWM(2, 0, L1_M3 - Move5);
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(3, 0, L3_M3 + Move6 - 10);
      PWM1.setPWM(0, 0, L1_M1 - Move6);
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(1, 0, L1_M2 + Move7); 
      PWM1.setPWM(2, 0, L1_M3 + Move7);
      PWM1.setPWM(9, 0, L3_M2 - Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(0, 0, L1_M1 + Move8);
      PWM1.setPWM(3, 0, L3_M3 - Move8 + 10); 
      Move8++;    
      Tripod2 = 2;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 2)
  {
    // Legs Lift by 20 deg
    if (Move9 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 + Move9);
      PWM1.setPWM(6, 0, L2_M3 + Move9);
      PWM2.setPWM(5, 0, L5_M2 - Move9); 
      PWM2.setPWM(6, 0, L5_M3 - Move9);
      Move9++;
    }

    // Legs turns forwards by 40 deg
    if (Move10 <= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 - Move10);
      PWM2.setPWM(4, 0, L5_M1 + Move10);
      Move10++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move10 > 40 && Move11 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 - Move11); 
      PWM1.setPWM(6, 0, L2_M3 - Move11);
      PWM2.setPWM(5, 0, L5_M2 + Move11); 
      PWM2.setPWM(6, 0, L5_M3 + Move11); 
      Move11++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move11 >= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 + Move12);
      PWM2.setPWM(4, 0, L5_M1 - Move12);
      Move12++;    
      Tripod2 = 0;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move12 >= 40)
    {
      Move9 = 0;
      Move10 = 0;
      Move11 = 0;
      Move12 = 0;
    }    
  }
}

void Move_Right_Ripple()
{
  if (Tripod2 == 0)
  {
  // Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM2.setPWM(1, 0, L4_M2 + Move1); 
    PWM2.setPWM(2, 0, L4_M3 + Move1); 
    PWM2.setPWM(9, 0, L6_M2 - Move1); 
    Move1++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move2 <= 40)
  {
    PWM2.setPWM(0, 0, L4_M1 - Move2); 
    PWM2.setPWM(10, 0, L6_M3 - Move2 + 10); 
    Move2++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM2.setPWM(1, 0, L4_M2 - Move1); 
    PWM2.setPWM(2, 0, L4_M3 - Move1); 
    PWM2.setPWM(9, 0, L6_M2 + Move1); 
    Move3++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move3 >= 40)
  {
    PWM2.setPWM(0, 0, L4_M1 + Move2); 
    PWM2.setPWM(10, 0, L6_M3 + Move2 - 10); 
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 + Move5);
      PWM1.setPWM(1, 0, L1_M2 - Move5); 
      PWM1.setPWM(2, 0, L1_M3 - Move5);
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(3, 0, L3_M3 - Move6 + 10);
      PWM1.setPWM(0, 0, L1_M1 + Move6);
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(1, 0, L1_M2 + Move7); 
      PWM1.setPWM(2, 0, L1_M3 + Move7);
      PWM1.setPWM(9, 0, L3_M2 - Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(0, 0, L1_M1 - Move8);
      PWM1.setPWM(3, 0, L3_M3 + Move8 - 10); 
      Move8++;    
      Tripod2 = 2;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 2)
  {
    // Legs Lift by 20 deg
    if (Move9 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 + Move9);
      PWM1.setPWM(6, 0, L2_M3 + Move9);
      PWM2.setPWM(5, 0, L5_M2 - Move9); 
      PWM2.setPWM(6, 0, L5_M3 - Move9);
      Move9++;
    }

    // Legs turns forwards by 40 deg
    if (Move10 <= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 + Move10);
      PWM2.setPWM(4, 0, L5_M1 - Move10);
      Move10++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move10 > 40 && Move11 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 - Move11); 
      PWM1.setPWM(6, 0, L2_M3 - Move11);
      PWM2.setPWM(5, 0, L5_M2 + Move11); 
      PWM2.setPWM(6, 0, L5_M3 + Move11); 
      Move11++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move11 >= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 - Move12);
      PWM2.setPWM(4, 0, L5_M1 + Move12);
      Move12++;    
      Tripod2 = 0;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move12 >= 40)
    {
      Move9 = 0;
      Move10 = 0;
      Move11 = 0;
      Move12 = 0;
    }    
  }
}

void Turn_Left_Ripple()
{
  if (Tripod2 == 0)
  {
  // Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM2.setPWM(1, 0, L4_M2 + Move1); 
    PWM2.setPWM(2, 0, L4_M3 + Move1); 
    PWM2.setPWM(9, 0, L6_M2 - Move1); 
    PWM2.setPWM(10, 0, L6_M3 - Move1); 
    Move1++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move2 <= 40)
  {
    PWM2.setPWM(0, 0, L4_M1 + Move2); 
    PWM2.setPWM(8, 0, L6_M1 + Move2); 
    Move2++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM2.setPWM(1, 0, L4_M2 - Move1); 
    PWM2.setPWM(2, 0, L4_M3 - Move1); 
    PWM2.setPWM(9, 0, L6_M2 + Move1); 
    PWM2.setPWM(10, 0, L6_M3 + Move1); 
    Move3++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move3 >= 40)
  {
    PWM2.setPWM(0, 0, L4_M1 - Move2); 
    PWM2.setPWM(8, 0, L6_M1 - Move2); 
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 + Move5);
      PWM1.setPWM(3, 0, L3_M3 + Move5);
      PWM1.setPWM(1, 0, L1_M2 - Move5); 
      PWM1.setPWM(2, 0, L1_M3 - Move5);
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(8, 0, L3_M1 + Move6);
      PWM1.setPWM(0, 0, L1_M1 + Move6);
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(1, 0, L1_M2 + Move7); 
      PWM1.setPWM(2, 0, L1_M3 + Move7);
      PWM1.setPWM(9, 0, L3_M2 - Move7); 
      PWM1.setPWM(3, 0, L3_M3 - Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(0, 0, L1_M1 - Move8);
      PWM1.setPWM(8, 0, L3_M1 - Move8);
      Move8++;    
      Tripod2 = 2;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 2)
  {
    // Legs Lift by 20 deg
    if (Move9 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 + Move9);
      PWM1.setPWM(6, 0, L2_M3 + Move9);
      PWM2.setPWM(5, 0, L5_M2 - Move9); 
      PWM2.setPWM(6, 0, L5_M3 - Move9);
      Move9++;
    }

    // Legs turns forwards by 40 deg
    if (Move10 <= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 + Move10);
      PWM2.setPWM(4, 0, L5_M1 + Move10);
      Move10++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move10 > 40 && Move11 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 - Move11); 
      PWM1.setPWM(6, 0, L2_M3 - Move11);
      PWM2.setPWM(5, 0, L5_M2 + Move11); 
      PWM2.setPWM(6, 0, L5_M3 + Move11); 
      Move11++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move11 >= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 - Move12);
      PWM2.setPWM(4, 0, L5_M1 - Move12);
      Move12++;    
      Tripod2 = 0;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move12 >= 40)
    {
      Move9 = 0;
      Move10 = 0;
      Move11 = 0;
      Move12 = 0;
    }    
  }
}

void Turn_Right_Ripple()
{
  if (Tripod2 == 0)
  {
  // Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM2.setPWM(1, 0, L4_M2 + Move1); 
    PWM2.setPWM(2, 0, L4_M3 + Move1); 
    PWM2.setPWM(9, 0, L6_M2 - Move1); 
    PWM2.setPWM(10, 0, L6_M3 - Move1); 
    Move1++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move2 <= 40)
  {
    PWM2.setPWM(0, 0, L4_M1 - Move2); 
    PWM2.setPWM(8, 0, L6_M1 - Move2); 
    Move2++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM2.setPWM(1, 0, L4_M2 - Move1); 
    PWM2.setPWM(2, 0, L4_M3 - Move1); 
    PWM2.setPWM(9, 0, L6_M2 + Move1); 
    PWM2.setPWM(10, 0, L6_M3 + Move1); 
    Move3++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move3 >= 40)
  {
    PWM2.setPWM(0, 0, L4_M1 + Move2); 
    PWM2.setPWM(8, 0, L6_M1 + Move2); 
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 + Move5);
      PWM1.setPWM(3, 0, L3_M3 + Move5);
      PWM1.setPWM(1, 0, L1_M2 - Move5); 
      PWM1.setPWM(2, 0, L1_M3 - Move5);
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(8, 0, L3_M1 - Move6);
      PWM1.setPWM(0, 0, L1_M1 - Move6);
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(1, 0, L1_M2 + Move7); 
      PWM1.setPWM(2, 0, L1_M3 + Move7);
      PWM1.setPWM(9, 0, L3_M2 - Move7); 
      PWM1.setPWM(3, 0, L3_M3 - Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(0, 0, L1_M1 + Move8);
      PWM1.setPWM(8, 0, L3_M1 + Move8);
      Move8++;    
      Tripod2 = 2;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 2)
  {
    // Legs Lift by 20 deg
    if (Move9 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 + Move9);
      PWM1.setPWM(6, 0, L2_M3 + Move9);
      PWM2.setPWM(5, 0, L5_M2 - Move9); 
      PWM2.setPWM(6, 0, L5_M3 - Move9);
      Move9++;
    }

    // Legs turns forwards by 40 deg
    if (Move10 <= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 - Move10);
      PWM2.setPWM(4, 0, L5_M1 - Move10);
      Move10++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move10 > 40 && Move11 <= 40)
    {
      PWM1.setPWM(5, 0, L2_M2 - Move11); 
      PWM1.setPWM(6, 0, L2_M3 - Move11);
      PWM2.setPWM(5, 0, L5_M2 + Move11); 
      PWM2.setPWM(6, 0, L5_M3 + Move11); 
      Move11++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move11 >= 40)
    {
      PWM1.setPWM(4, 0, L2_M1 + Move12);
      PWM2.setPWM(4, 0, L5_M1 + Move12);
      Move12++;    
      Tripod2 = 0;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move12 >= 40)
    {
      Move9 = 0;
      Move10 = 0;
      Move11 = 0;
      Move12 = 0;
    }    
  }
}

void Forward_Wave()
{
  if (Tripod2 == 0)
  {
  // Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM1.setPWM(5, 0, L2_M2 + Move1); 
    PWM1.setPWM(6, 0, L2_M3 + Move1); 
    Move1++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move2 <= 40)
  {
    PWM1.setPWM(4, 0, L2_M1 + Move2); 
    Move2++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM1.setPWM(5, 0, L2_M2 - Move3); 
    PWM1.setPWM(6, 0, L2_M3 - Move3); 
    Move3++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move3 >= 40)
  {
    PWM1.setPWM(4, 0, L2_M1 - Move4); 
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 + Move5);
      PWM1.setPWM(3, 0, L3_M3 + Move5);
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(8, 0, L3_M1 + Move6);
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 - Move7); 
      PWM1.setPWM(3, 0, L3_M3 - Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(8, 0, L3_M1 - Move8);
      Move8++;    
      Tripod2 = 2;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 2)
  {
    // Legs Lift by 20 deg
    if (Move9 <= 40)
    {
      PWM2.setPWM(1, 0, L4_M2 + Move9); 
      PWM2.setPWM(2, 0, L4_M3 + Move9);
      Move9++;
    }

    // Legs turns forwards by 40 deg
    if (Move10 <= 40)
    {
      PWM2.setPWM(0, 0, L4_M1 + Move10);
      Move10++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move10 > 40 && Move11 <= 40)
    {
      PWM2.setPWM(1, 0, L4_M2 - Move11); 
      PWM2.setPWM(2, 0, L4_M3 - Move11); 
      Move11++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move11 >= 40)
    {
      PWM2.setPWM(0, 0, L4_M1 - Move12);
      Move12++;    
      Tripod2 = 3;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move12 >= 40)
    {
      Move9 = 0;
      Move10 = 0;
      Move11 = 0;
      Move12 = 0;
    }    
  }

//----------------------------------------------------------------------------//

  if (Tripod2 == 5)
  {
  // Legs Lift by 20 deg
  if (Move13 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 - Move13); 
    PWM1.setPWM(2, 0, L1_M3 - Move13); 
    Move13++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move14 <= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 - Move14); 
    Move14++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move14 > 40 && Move15 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 + Move15); 
    PWM1.setPWM(2, 0, L1_M3 + Move15); 
    Move15++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move15 >= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 + Move16); 
    Move16++;
    Tripod2 = 0;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move16 >= 40)
  {
    Move13 = 0;
    Move14 = 0;
    Move15 = 0;
    Move16 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 4)
  {
    // Legs Lift by 20 deg
    if (Move17 <= 40)
    {
      PWM2.setPWM(9, 0, L6_M2 - Move17);
      PWM2.setPWM(10, 0, L6_M3 - Move17);
      Move17++;
    }

    // Legs turns forwards by 40 deg
    if (Move18 <= 40)
    {
      PWM2.setPWM(8, 0, L6_M1 - Move18);
      Move18++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move18 > 40 && Move19 <= 40)
    {
      PWM2.setPWM(9, 0, L6_M2 + Move19); 
      PWM2.setPWM(10, 0, L6_M3 + Move19); 
      Move19++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move19 >= 40)
    {
      PWM2.setPWM(8, 0, L6_M1 + Move20);
      Move20++;    
      Tripod2 = 5;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move20 >= 40)
    {
      Move17 = 0;
      Move18 = 0;
      Move19 = 0;
      Move20 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 3)
  {
    // Legs Lift by 20 deg
    if (Move21 <= 40)
    {
      PWM2.setPWM(5, 0, L5_M2 - Move21); 
      PWM2.setPWM(6, 0, L5_M3 - Move21);
      Move21++;
    }

    // Legs turns forwards by 40 deg
    if (Move22 <= 40)
    {
      PWM2.setPWM(4, 0, L5_M1 - Move22);
      Move22++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move22 > 40 && Move23 <= 40)
    {
      PWM2.setPWM(5, 0, L5_M2 + Move23); 
      PWM2.setPWM(6, 0, L5_M3 + Move23); 
      Move23++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move23 >= 40)
    {
      PWM2.setPWM(4, 0, L5_M1 + Move24);
      Move24++;    
      Tripod2 = 4;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move24 >= 40)
    {
      Move21 = 0;
      Move22 = 0;
      Move23 = 0;
      Move24 = 0;
    }    
  }
}

void Backward_Wave()
{
  if (Tripod2 == 0)
  {
  // Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM1.setPWM(5, 0, L2_M2 + Move1); 
    PWM1.setPWM(6, 0, L2_M3 + Move1); 
    Move1++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move2 <= 40)
  {
    PWM1.setPWM(4, 0, L2_M1 - Move2); 
    Move2++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM1.setPWM(5, 0, L2_M2 - Move3); 
    PWM1.setPWM(6, 0, L2_M3 - Move3); 
    Move3++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move3 >= 40)
  {
    PWM1.setPWM(4, 0, L2_M1 + Move4); 
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 + Move5);
      PWM1.setPWM(3, 0, L3_M3 + Move5);
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(8, 0, L3_M1 - Move6);
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 - Move7); 
      PWM1.setPWM(3, 0, L3_M3 - Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(8, 0, L3_M1 + Move8);
      Move8++;    
      Tripod2 = 2;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 2)
  {
    // Legs Lift by 20 deg
    if (Move9 <= 40)
    {
      PWM2.setPWM(1, 0, L4_M2 + Move9); 
      PWM2.setPWM(2, 0, L4_M3 + Move9);
      Move9++;
    }

    // Legs turns forwards by 40 deg
    if (Move10 <= 40)
    {
      PWM2.setPWM(0, 0, L4_M1 - Move10);
      Move10++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move10 > 40 && Move11 <= 40)
    {
      PWM2.setPWM(1, 0, L4_M2 - Move11); 
      PWM2.setPWM(2, 0, L4_M3 - Move11); 
      Move11++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move11 >= 40)
    {
      PWM2.setPWM(0, 0, L4_M1 + Move12);
      Move12++;    
      Tripod2 = 3;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move12 >= 40)
    {
      Move9 = 0;
      Move10 = 0;
      Move11 = 0;
      Move12 = 0;
    }    
  }

//----------------------------------------------------------------------------//

  if (Tripod2 == 5)
  {
  // Legs Lift by 20 deg
  if (Move13 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 - Move13); 
    PWM1.setPWM(2, 0, L1_M3 - Move13); 
    Move13++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move14 <= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 + Move14); 
    Move14++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move14 > 40 && Move15 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 + Move15); 
    PWM1.setPWM(2, 0, L1_M3 + Move15); 
    Move15++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move15 >= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 - Move16); 
    Move16++;
    Tripod2 = 0;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move16 >= 40)
  {
    Move13 = 0;
    Move14 = 0;
    Move15 = 0;
    Move16 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 4)
  {
    // Legs Lift by 20 deg
    if (Move17 <= 40)
    {
      PWM2.setPWM(9, 0, L6_M2 - Move17);
      PWM2.setPWM(10, 0, L6_M3 - Move17);
      Move17++;
    }

    // Legs turns forwards by 40 deg
    if (Move18 <= 40)
    {
      PWM2.setPWM(8, 0, L6_M1 + Move18);
      Move18++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move18 > 40 && Move19 <= 40)
    {
      PWM2.setPWM(9, 0, L6_M2 + Move19); 
      PWM2.setPWM(10, 0, L6_M3 + Move19); 
      Move19++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move19 >= 40)
    {
      PWM2.setPWM(8, 0, L6_M1 - Move20);
      Move20++;    
      Tripod2 = 5;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move20 >= 40)
    {
      Move17 = 0;
      Move18 = 0;
      Move19 = 0;
      Move20 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 3)
  {
    // Legs Lift by 20 deg
    if (Move21 <= 40)
    {
      PWM2.setPWM(5, 0, L5_M2 - Move21); 
      PWM2.setPWM(6, 0, L5_M3 - Move21);
      Move21++;
    }

    // Legs turns forwards by 40 deg
    if (Move22 <= 40)
    {
      PWM2.setPWM(4, 0, L5_M1 + Move22);
      Move22++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move22 > 40 && Move23 <= 40)
    {
      PWM2.setPWM(5, 0, L5_M2 + Move23); 
      PWM2.setPWM(6, 0, L5_M3 + Move23); 
      Move23++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move23 >= 40)
    {
      PWM2.setPWM(4, 0, L5_M1 - Move24);
      Move24++;    
      Tripod2 = 4;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move24 >= 40)
    {
      Move21 = 0;
      Move22 = 0;
      Move23 = 0;
      Move24 = 0;
    }    
  }
}

void Move_Left_Wave()
{
  if (Tripod2 == 0)
  {
  // Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM1.setPWM(5, 0, L2_M2 + Move1); 
    PWM1.setPWM(6, 0, L2_M3 + Move1); 
    Move1++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move2 <= 40)
  {
    PWM1.setPWM(4, 0, L2_M1 - Move2); 
    Move2++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM1.setPWM(5, 0, L2_M2 - Move3); 
    PWM1.setPWM(6, 0, L2_M3 - Move3); 
    Move3++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move3 >= 40)
  {
    PWM1.setPWM(4, 0, L2_M1 + Move4); 
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 + Move5);
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(3, 0, L3_M3 + Move6 - 10);
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 - Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(3, 0, L3_M3 - Move8 + 10); 
      Move8++;    
      Tripod2 = 2;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 2)
  {
    // Legs Lift by 20 deg
    if (Move9 <= 40)
    {
      PWM2.setPWM(1, 0, L4_M2 + Move9); 
      PWM2.setPWM(2, 0, L4_M3 + Move9);
      Move9++;
    }

    // Legs turns forwards by 40 deg
    if (Move10 <= 40)
    {
      PWM2.setPWM(0, 0, L4_M1 + Move10);
      Move10++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move10 > 40 && Move11 <= 40)
    {
      PWM2.setPWM(1, 0, L4_M2 - Move11); 
      PWM2.setPWM(2, 0, L4_M3 - Move11); 
      Move11++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move11 >= 40)
    {
      PWM2.setPWM(0, 0, L4_M1 - Move12);
      Move12++;    
      Tripod2 = 3;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move12 >= 40)
    {
      Move9 = 0;
      Move10 = 0;
      Move11 = 0;
      Move12 = 0;
    }    
  }

//----------------------------------------------------------------------------//

  if (Tripod2 == 5)
  {
  // Legs Lift by 20 deg
  if (Move13 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 - Move13); 
    PWM1.setPWM(2, 0, L1_M3 - Move13); 
    Move13++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move14 <= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 - Move14); 
    Move14++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move14 > 40 && Move15 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 + Move15); 
    PWM1.setPWM(2, 0, L1_M3 + Move15); 
    Move15++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move15 >= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 + Move16); 
    Move16++;
    Tripod2 = 0;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move16 >= 40)
  {
    Move13 = 0;
    Move14 = 0;
    Move15 = 0;
    Move16 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 4)
  {
    // Legs Lift by 20 deg
    if (Move17 <= 40)
    {
      PWM2.setPWM(9, 0, L6_M2 - Move17);
      Move17++;
    }

    // Legs turns forwards by 40 deg
    if (Move18 <= 40)
    {
      PWM2.setPWM(10, 0, L6_M3 + Move18 - 10);
      Move18++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move18 > 40 && Move19 <= 40)
    {
      PWM2.setPWM(9, 0, L6_M2 + Move19); 
      Move19++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move19 >= 40)
    {
      PWM2.setPWM(10, 0, L6_M3 - Move19 + 10); 
      Move20++;    
      Tripod2 = 5;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move20 >= 40)
    {
      Move17 = 0;
      Move18 = 0;
      Move19 = 0;
      Move20 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 3)
  {
    // Legs Lift by 20 deg
    if (Move21 <= 40)
    {
      PWM2.setPWM(5, 0, L5_M2 - Move21); 
      PWM2.setPWM(6, 0, L5_M3 - Move21);
      Move21++;
    }

    // Legs turns forwards by 40 deg
    if (Move22 <= 40)
    {
      PWM2.setPWM(4, 0, L5_M1 + Move22);
      Move22++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move22 > 40 && Move23 <= 40)
    {
      PWM2.setPWM(5, 0, L5_M2 + Move23); 
      PWM2.setPWM(6, 0, L5_M3 + Move23); 
      Move23++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move23 >= 40)
    {
      PWM2.setPWM(4, 0, L5_M1 - Move24);
      Move24++;    
      Tripod2 = 4;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move24 >= 40)
    {
      Move21 = 0;
      Move22 = 0;
      Move23 = 0;
      Move24 = 0;
    }    
  }
}

void Move_Right_Wave()
{
  if (Tripod2 == 0)
  {
  // Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM1.setPWM(5, 0, L2_M2 + Move1); 
    PWM1.setPWM(6, 0, L2_M3 + Move1); 
    Move1++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move2 <= 40)
  {
    PWM1.setPWM(4, 0, L2_M1 + Move2); 
    Move2++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM1.setPWM(5, 0, L2_M2 - Move3); 
    PWM1.setPWM(6, 0, L2_M3 - Move3); 
    Move3++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move3 >= 40)
  {
    PWM1.setPWM(4, 0, L2_M1 - Move4); 
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 + Move5);
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(3, 0, L3_M3 - Move6 + 10);
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 - Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(3, 0, L3_M3 + Move8 - 10); 
      Move8++;    
      Tripod2 = 2;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 2)
  {
    // Legs Lift by 20 deg
    if (Move9 <= 40)
    {
      PWM2.setPWM(1, 0, L4_M2 + Move9); 
      PWM2.setPWM(2, 0, L4_M3 + Move9);
      Move9++;
    }

    // Legs turns forwards by 40 deg
    if (Move10 <= 40)
    {
      PWM2.setPWM(0, 0, L4_M1 - Move10);
      Move10++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move10 > 40 && Move11 <= 40)
    {
      PWM2.setPWM(1, 0, L4_M2 - Move11); 
      PWM2.setPWM(2, 0, L4_M3 - Move11); 
      Move11++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move11 >= 40)
    {
      PWM2.setPWM(0, 0, L4_M1 + Move12);
      Move12++;    
      Tripod2 = 3;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move12 >= 40)
    {
      Move9 = 0;
      Move10 = 0;
      Move11 = 0;
      Move12 = 0;
    }    
  }

//----------------------------------------------------------------------------//

  if (Tripod2 == 5)
  {
  // Legs Lift by 20 deg
  if (Move13 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 - Move13); 
    PWM1.setPWM(2, 0, L1_M3 - Move13); 
    Move13++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move14 <= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 + Move14); 
    Move14++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move14 > 40 && Move15 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 + Move15); 
    PWM1.setPWM(2, 0, L1_M3 + Move15); 
    Move15++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move15 >= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 - Move16); 
    Move16++;
    Tripod2 = 0;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move16 >= 40)
  {
    Move13 = 0;
    Move14 = 0;
    Move15 = 0;
    Move16 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 4)
  {
    // Legs Lift by 20 deg
    if (Move17 <= 40)
    {
      PWM2.setPWM(9, 0, L6_M2 - Move17);
      Move17++;
    }

    // Legs turns forwards by 40 deg
    if (Move18 <= 40)
    {
      PWM2.setPWM(10, 0, L6_M3 - Move18 + 10);
      Move18++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move18 > 40 && Move19 <= 40)
    {
      PWM2.setPWM(9, 0, L6_M2 + Move19); 
      Move19++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move19 >= 40)
    {
      PWM2.setPWM(10, 0, L6_M3 + Move19 - 10); 
      Move20++;    
      Tripod2 = 5;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move20 >= 40)
    {
      Move17 = 0;
      Move18 = 0;
      Move19 = 0;
      Move20 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 3)
  {
    // Legs Lift by 20 deg
    if (Move21 <= 40)
    {
      PWM2.setPWM(5, 0, L5_M2 - Move21); 
      PWM2.setPWM(6, 0, L5_M3 - Move21);
      Move21++;
    }

    // Legs turns forwards by 40 deg
    if (Move22 <= 40)
    {
      PWM2.setPWM(4, 0, L5_M1 - Move22);
      Move22++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move22 > 40 && Move23 <= 40)
    {
      PWM2.setPWM(5, 0, L5_M2 + Move23); 
      PWM2.setPWM(6, 0, L5_M3 + Move23); 
      Move23++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move23 >= 40)
    {
      PWM2.setPWM(4, 0, L5_M1 + Move24);
      Move24++;    
      Tripod2 = 4;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move24 >= 40)
    {
      Move21 = 0;
      Move22 = 0;
      Move23 = 0;
      Move24 = 0;
    }    
  }
}

void Turn_Left_Wave()
{
  if (Tripod2 == 0)
  {
  // Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM1.setPWM(5, 0, L2_M2 + Move1); 
    PWM1.setPWM(6, 0, L2_M3 + Move1); 
    Move1++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move2 <= 40)
  {
    PWM1.setPWM(4, 0, L2_M1 + Move2); 
    Move2++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM1.setPWM(5, 0, L2_M2 - Move3); 
    PWM1.setPWM(6, 0, L2_M3 - Move3); 
    Move3++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move3 >= 40)
  {
    PWM1.setPWM(4, 0, L2_M1 - Move4); 
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 + Move5);
      PWM1.setPWM(3, 0, L3_M3 + Move5);
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(8, 0, L3_M1 + Move6);
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 - Move7); 
      PWM1.setPWM(3, 0, L3_M3 - Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(8, 0, L3_M1 - Move8);
      Move8++;    
      Tripod2 = 2;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 2)
  {
    // Legs Lift by 20 deg
    if (Move9 <= 40)
    {
      PWM2.setPWM(1, 0, L4_M2 + Move9); 
      PWM2.setPWM(2, 0, L4_M3 + Move9);
      Move9++;
    }

    // Legs turns forwards by 40 deg
    if (Move10 <= 40)
    {
      PWM2.setPWM(0, 0, L4_M1 + Move10);
      Move10++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move10 > 40 && Move11 <= 40)
    {
      PWM2.setPWM(1, 0, L4_M2 - Move11); 
      PWM2.setPWM(2, 0, L4_M3 - Move11); 
      Move11++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move11 >= 40)
    {
      PWM2.setPWM(0, 0, L4_M1 - Move12);
      Move12++;    
      Tripod2 = 3;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move12 >= 40)
    {
      Move9 = 0;
      Move10 = 0;
      Move11 = 0;
      Move12 = 0;
    }    
  }

//----------------------------------------------------------------------------//

  if (Tripod2 == 5)
  {
  // Legs Lift by 20 deg
  if (Move13 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 - Move13); 
    PWM1.setPWM(2, 0, L1_M3 - Move13); 
    Move13++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move14 <= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 + Move14); 
    Move14++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move14 > 40 && Move15 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 + Move15); 
    PWM1.setPWM(2, 0, L1_M3 + Move15); 
    Move15++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move15 >= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 - Move16); 
    Move16++;
    Tripod2 = 0;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move16 >= 40)
  {
    Move13 = 0;
    Move14 = 0;
    Move15 = 0;
    Move16 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 4)
  {
    // Legs Lift by 20 deg
    if (Move17 <= 40)
    {
      PWM2.setPWM(9, 0, L6_M2 - Move17);
      PWM2.setPWM(10, 0, L6_M3 - Move17);
      Move17++;
    }

    // Legs turns forwards by 40 deg
    if (Move18 <= 40)
    {
      PWM2.setPWM(8, 0, L6_M1 + Move18);
      Move18++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move18 > 40 && Move19 <= 40)
    {
      PWM2.setPWM(9, 0, L6_M2 + Move19); 
      PWM2.setPWM(10, 0, L6_M3 + Move19); 
      Move19++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move19 >= 40)
    {
      PWM2.setPWM(8, 0, L6_M1 - Move20);
      Move20++;    
      Tripod2 = 5;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move20 >= 40)
    {
      Move17 = 0;
      Move18 = 0;
      Move19 = 0;
      Move20 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 3)
  {
    // Legs Lift by 20 deg
    if (Move21 <= 40)
    {
      PWM2.setPWM(5, 0, L5_M2 - Move21); 
      PWM2.setPWM(6, 0, L5_M3 - Move21);
      Move21++;
    }

    // Legs turns forwards by 40 deg
    if (Move22 <= 40)
    {
      PWM2.setPWM(4, 0, L5_M1 + Move22);
      Move22++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move22 > 40 && Move23 <= 40)
    {
      PWM2.setPWM(5, 0, L5_M2 + Move23); 
      PWM2.setPWM(6, 0, L5_M3 + Move23); 
      Move23++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move23 >= 40)
    {
      PWM2.setPWM(4, 0, L5_M1 - Move24);
      Move24++;    
      Tripod2 = 4;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move24 >= 40)
    {
      Move21 = 0;
      Move22 = 0;
      Move23 = 0;
      Move24 = 0;
    }    
  }
}

void Turn_Right_Wave()
{
  if (Tripod2 == 0)
  {
  // Legs Lift by 20 deg
  if (Move1 <= 40)
  {
    PWM1.setPWM(5, 0, L2_M2 + Move1); 
    PWM1.setPWM(6, 0, L2_M3 + Move1); 
    Move1++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move2 <= 40)
  {
    PWM1.setPWM(4, 0, L2_M1 - Move2); 
    Move2++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move2 > 40 && Move3 <= 40)
  {
    PWM1.setPWM(5, 0, L2_M2 - Move3); 
    PWM1.setPWM(6, 0, L2_M3 - Move3); 
    Move3++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move3 >= 40)
  {
    PWM1.setPWM(4, 0, L2_M1 + Move4); 
    Move4++;
    Tripod2 = 1;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move4 >= 40)
  {
    Move1 = 0;
    Move2 = 0;
    Move3 = 0;
    Move4 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 1)
  {
    // Legs Lift by 20 deg
    if (Move5 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 + Move5);
      PWM1.setPWM(3, 0, L3_M3 + Move5);
      Move5++;
    }

    // Legs turns forwards by 40 deg
    if (Move6 <= 40)
    {
      PWM1.setPWM(8, 0, L3_M1 - Move6);
      Move6++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move6 > 40 && Move7 <= 40)
    {
      PWM1.setPWM(9, 0, L3_M2 - Move7); 
      PWM1.setPWM(3, 0, L3_M3 - Move7); 
      Move7++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move7 >= 40)
    {
      PWM1.setPWM(8, 0, L3_M1 + Move8);
      Move8++;    
      Tripod2 = 2;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move8 >= 40)
    {
      Move5 = 0;
      Move6 = 0;
      Move7 = 0;
      Move8 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 2)
  {
    // Legs Lift by 20 deg
    if (Move9 <= 40)
    {
      PWM2.setPWM(1, 0, L4_M2 + Move9); 
      PWM2.setPWM(2, 0, L4_M3 + Move9);
      Move9++;
    }

    // Legs turns forwards by 40 deg
    if (Move10 <= 40)
    {
      PWM2.setPWM(0, 0, L4_M1 - Move10);
      Move10++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move10 > 40 && Move11 <= 40)
    {
      PWM2.setPWM(1, 0, L4_M2 - Move11); 
      PWM2.setPWM(2, 0, L4_M3 - Move11); 
      Move11++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move11 >= 40)
    {
      PWM2.setPWM(0, 0, L4_M1 + Move12);
      Move12++;    
      Tripod2 = 3;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move12 >= 40)
    {
      Move9 = 0;
      Move10 = 0;
      Move11 = 0;
      Move12 = 0;
    }    
  }

//----------------------------------------------------------------------------//

  if (Tripod2 == 5)
  {
  // Legs Lift by 20 deg
  if (Move13 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 - Move13); 
    PWM1.setPWM(2, 0, L1_M3 - Move13); 
    Move13++;
  }  
      
  // Legs turns forwards by 40 deg
  if (Move14 <= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 - Move14); 
    Move14++;
  }
  // These work in tandum
  
  // Legs pushed into floor by -20 deg
  if (Move14 > 40 && Move15 <= 40)
  {
    PWM1.setPWM(1, 0, L1_M2 + Move15); 
    PWM1.setPWM(2, 0, L1_M3 + Move15); 
    Move15++;
  } 
  
  // Legs pushes/turns backwards by -40 deg  
  if (Move15 >= 40)
  {
    PWM1.setPWM(0, 0, L1_M1 + Move16); 
    Move16++;
    Tripod2 = 0;      
  }

  // Legs counter is reset and other pair is switched ON
  if(Move16 >= 40)
  {
    Move13 = 0;
    Move14 = 0;
    Move15 = 0;
    Move16 = 0;
  }
  }
  
//----------------------------------------------------------------------------//

  if (Tripod2 == 4)
  {
    // Legs Lift by 20 deg
    if (Move17 <= 40)
    {
      PWM2.setPWM(9, 0, L6_M2 - Move17);
      PWM2.setPWM(10, 0, L6_M3 - Move17);
      Move17++;
    }

    // Legs turns forwards by 40 deg
    if (Move18 <= 40)
    {
      PWM2.setPWM(8, 0, L6_M1 - Move18);
      Move18++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move18 > 40 && Move19 <= 40)
    {
      PWM2.setPWM(9, 0, L6_M2 + Move19); 
      PWM2.setPWM(10, 0, L6_M3 + Move19); 
      Move19++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move19 >= 40)
    {
      PWM2.setPWM(8, 0, L6_M1 + Move20);
      Move20++;    
      Tripod2 = 5;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move20 >= 40)
    {
      Move17 = 0;
      Move18 = 0;
      Move19 = 0;
      Move20 = 0;
    }    
  }
  //----------------------------------------------------------------------------//

  if (Tripod2 == 3)
  {
    // Legs Lift by 20 deg
    if (Move21 <= 40)
    {
      PWM2.setPWM(5, 0, L5_M2 - Move21); 
      PWM2.setPWM(6, 0, L5_M3 - Move21);
      Move21++;
    }

    // Legs turns forwards by 40 deg
    if (Move22 <= 40)
    {
      PWM2.setPWM(4, 0, L5_M1 - Move22);
      Move22++;
    }
  
    // Legs pushed into floor by -20 deg
    if (Move22 > 40 && Move23 <= 40)
    {
      PWM2.setPWM(5, 0, L5_M2 + Move23); 
      PWM2.setPWM(6, 0, L5_M3 + Move23); 
      Move23++;
    } 
  
    // Legs turns backwards by -40 deg
    if (Move23 >= 40)
    {
      PWM2.setPWM(4, 0, L5_M1 + Move24);
      Move24++;    
      Tripod2 = 4;
    }

    // Legs counter is reset and other pair is switched OFF    
    if(Move24 >= 40)
    {
      Move21 = 0;
      Move22 = 0;
      Move23 = 0;
      Move24 = 0;
    }    
  }
}