#include <Adafruit_LIS3DH.h>

// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

void setup(void)
{
  Serial.begin(9600);

  lis.begin();
  lis.setRange(LIS3DH_RANGE_2_G);   // 2G
  lis.setDataRate(LIS3DH_DATARATE_400_HZ);
}

void loop()
{
  lis.read();      // get X Y and Z data at once

  int a = lis.x;
  int b = lis.y;
  int c = lis.z;

  /* Then print out the raw data
  Serial.print("X:  "); Serial.print(a);
  Serial.print("  \tY:  "); Serial.print(b);
  Serial.print("  \tZ:  "); Serial.print(c);
  */
  
  
  //--Acceleration Values--

  sensors_event_t event;
  lis.getEvent(&event);

  float AccX = event.acceleration.x - 0.7;
  float AccY = event.acceleration.y - 0.1;
  float AccZ = event.acceleration.z - 0.5;

  // Display the results (acceleration is measured in m/s^2)//
  Serial.print("\t\tX: "); Serial.print(AccX);
  Serial.print(" \tY: "); Serial.print(AccY);
  Serial.print(" \tZ: "); Serial.print(AccZ);
  Serial.println(" m/s^2 ");
  //

  // MOVE LEFT & RIGHT (RIPPLE) (WAVE)

  float AngleRoll;
  float AnglePitch;

  AngleRoll = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ))*1/(PI/180);// - 1;
  AnglePitch = -atan(AccX/sqrt(AccY*AccY + AccZ*AccZ))*1/(PI/180);// + 3.75;
/*
  Serial.print("\t\tRoll Angle = "); // Y
  Serial.print(AngleRoll);
  Serial.print(" \tPitch Angle = "); // X
  Serial.println(AnglePitch);
*/

  /*
  Serial.print(a); // X
  Serial.print(',');
  Serial.print(b); // Y
  Serial.print(',');
  Serial.print(c); // Z
  Serial.print(',');
  */
  //Serial.println();

  delay(200);
}
