/*
* Created by 'Lankash'
* @6/2/2022
* File content: Reading + Mapping for Potentiometer Sensor
* Note:- Sensor (2), The red one.
*/

#define POTENTIOMETER_PIN A0

void setup() 
{
  Serial.begin(9600);
}

void loop()
{
  int data = analogRead(POTENTIOMETER_PIN);
  int percentage = map(data, 1023, 0, -81, 1900);
  Serial.print("Potentiometer at ");
  Serial.print(percentage);
  Serial.println(" mm");
  delay(100);
}
