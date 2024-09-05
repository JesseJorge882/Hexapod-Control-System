/*
______________________________________

Ultrasonic Sensor Test
______________________________________

*/

#define TRIG 11
#define ECHO 12


void setup()
{
  Serial.begin(9600);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop()
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(5);

  digitalWrite(TRIG, LOW);


  float t = pulseIn(ECHO, HIGH);
  float distance = t*0.017; // Distance in cm
  float slope = atan(distance/12)*180/PI;

  Serial.print("Distance: ");
  Serial.println(distance);
  Serial.print("Slope angle: ");
  Serial.print(slopeAngle);
  Serial.println(" degrees");

  delay(2500);
}