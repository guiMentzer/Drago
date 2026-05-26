#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_PWM  78 // This is the 'minimum' pulse length count (out of 4096)
#define MAX_PWM  585 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Máx. e mín. ângulo da junta 1
#define MIN_1 -1.57*(180/3.1415)
#define MAX_1 1.57*(180/3.1415)

// Máx. e mín. ângulo da junta 2
#define MIN_2 -1.31*(180/3.1415)
#define MAX_2 2.50*(180/3.1415)

// Máx. e mín. ângulo da junta 3
#define MIN_3 -4.00*(180/3.1415)
#define MAX_3 0.15*(180/3.1415)

// Máx. e mín. ângulo da junta 4
#define MIN_4 -1.57*(180/3.1415)
#define MAX_4 1.57*(180/3.1415)

// Máx. e mín. ângulo da junta 5
#define MIN_5 -1.57*(180/3.1415)
#define MAX_5 1.396*(180/3.1415)

// Máx. e mín. ângulo da junta 6
#define MIN_6 -1.57*(180/3.1415)
#define MAX_6 1.57*(180/3.1415)

Adafruit_PWMServoDriver Driver = Adafruit_PWMServoDriver();

int angleToPulse(float angle, int joint) 
{
  switch(joint)
  {
    case 0:
    if (angle>MAX_1) {return MAX_PWM;}
    if (angle<MIN_1) {return MIN_PWM;}
    return map(angle, MIN_1, MAX_1, MIN_PWM, MAX_PWM)+48;

    case 1:
    if (angle>MAX_2) {return MIN_PWM;}
    if (angle<MIN_2) {return MAX_PWM;}
    return map(angle, MIN_2, MAX_2, MAX_PWM, MIN_PWM)-12;

    case 2:
    if (angle>MAX_3) {return MAX_PWM;}
    if (angle<MIN_3) {return MIN_PWM;}
    return map(angle, MIN_3, MAX_3, MIN_PWM, MAX_PWM)+3;

    case 3:
    if (angle>MAX_4) {return 139;}
    if (angle<MIN_4) {return MAX_PWM;}
    return map(angle, MIN_4, MAX_4, MAX_PWM, 139)+13;

    case 4:
    if (angle>MAX_5) {return MIN_PWM;}
    if (angle<MIN_5) {return 529;}
    return map(angle, MIN_5, MAX_5, MAX_PWM, MIN_PWM)-7;

    case 5:
    if (angle>MAX_6) {return 544;}
    if (angle<MIN_6) {return 120;}
    return map(angle, MIN_6, MAX_6, 120, 544);
  }
}

float angles[6];

void setup()
  {
    Serial.begin(115200);
    Driver.begin();
    Driver.setPWMFreq(SERVO_FREQ);
    
    delay(10);
  }

void loop() 
{
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n'); 
  input.trim();

  if (input.length() == 0) return;
  if (input == "HOME") 
  {
    //Serial.println("OK HOME");
    angles[6] = {0.0}; 
    for(byte j=0;j<=5;j++)
    {
    Driver.setPWM(j, 0, angleToPulse(angles[j], j));
    delay(10);
    }
  }

if (input.startsWith("J ")) {
  String data = input.substring(3);
  data.trim();

  int parsed = 0;

    // Separa por espaços um token de cada vez
    while (parsed < 6 && data.length() > 0) {
      int spaceIdx = data.indexOf(' ');
      String token;

      if (spaceIdx == -1) {
        // último token
        token = data;
        data = "";
      } else {
        token = data.substring(0, spaceIdx);
        data = data.substring(spaceIdx + 1);
        data.trim();
      }

      if (token.length() > 0) {
        angles[parsed] = token.toFloat();
        parsed++;
      }
    }

   if (parsed != 6) {
      //Serial.print("ERR expected ");
      //Serial.print("6");
      //Serial.print(" angles, got ");
      //Serial.println(parsed);
      return;
    }

    for(byte j=0;j<=5;j++)
    {
    Driver.setPWM(j, 0, angleToPulse(angles[j], j));
    delay(10);
    //Serial.println(angles[j]);
    //Serial.println(angleToPulse(angles[j]));
    }

    //Serial.println("OK J");
    return;
}  

//Serial.print("ERR unknown: ");
  //Serial.println(input);

}