// In development

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Adafruit 16-channel PWM & Servo driver
// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  0 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  1000 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 300 // Analog servos run at ~50 Hz updates

#define pirPin1 35 // PIR downstairs
#define pirPin2 34 // PIR upstairs
bool pirState1 = false; // by default, no motion detected
bool pirState2 = false;
int pirValue1 = LOW; // variable to store the sensor status (value)
int pirValue2 = LOW;
unsigned long pirCheckInterval=100;    // the time we need to wait in ms
unsigned long pirPreviousMillis=0; // millis() returns an unsigned long.

int lightState = 0; // Defines if the lights are currently off (0), go on in ascending series (1) or go on in decending series (2)
unsigned long lightStayOnTime=10000;    // the time the lights stay on if PIR is triggered in ms
unsigned long lightPreviousMillis=0; // millis() returns an unsigned long.

// our servo # counter
uint8_t servonum = 0;
int steps = 20;

void setup() {
  Serial.begin(9600);
  Serial.println("2 channel Servo test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  pinMode(pirPin1, INPUT);    // initialize sensor as an input
  pinMode(pirPin2, INPUT);    // initialize sensor as an input

  delay(10);
}

void loop() {

  // Check PIR States
  if ((unsigned long)(millis() - pirPreviousMillis) >= pirCheckInterval) {
    pirPreviousMillis = millis();

    // PIR 1
    pirValue1 = digitalRead(pirPin1);   // read sensor value
    if (pirValue1 == HIGH) {           // check if the sensor is HIGH
      if (pirState1 == false) {
        Serial.println("Motion on Sensor 1 detected!"); 
        pirState1 = true;       // update variable state to true
      }
    } 
    else {
        if (pirState1 == true){
          Serial.println("Motion on Sensor 1 stopped!");
          pirState1 = false;       // update variable state to false
      }
    }

    // PIR 2
    pirValue2 = digitalRead(pirPin2);   // read sensor value
    if (pirValue2 == HIGH) {           // check if the sensor is HIGH
      if (pirState2 == false) {
        Serial.println("Motion on Sensor 2 detected!"); 
        pirState2 = true;       // update variable state to true
      }
    } 
    else {
        if (pirState2 == true){
          Serial.println("Motion on Sensor 2 stopped!");
          pirState2 = false;       // update variable state to false
        }
    }

      // If motion is detected switch on lights in ascending or descending series
    if (pirState1 == true) {
      // Check if currently active
      if (lightState == 0){
        lightState = 1; // Switch on light in ascending series
        Serial.println("Light on in ascending series!"); 
        // Start timer for switching lights off in ascending series
        lightPreviousMillis = millis();
        // Function Licht von unten nach oben an
      }      
      else if (lightState == 2){
        // Reset timer for off
        lightPreviousMillis = 0;
      }
    }

    if (pirState2 == true) {
      // Check if currently active
      if (lightState == 0){
        lightState = 2; // Switch on light in descending series
        Serial.println("Light on in descending series!"); 
        // Start timer for switching lights off in descending series
        lightPreviousMillis = millis();
        // Function Licht von oben nach unten an
      }      
      else if (lightState == 1){
        // Reset timer for off
        lightPreviousMillis = 0;
      }
    }
  }



  // Switch lights off
  if (lightState == 1){
    //Check if timer is zero
    if ((unsigned long)(millis() - lightPreviousMillis) >= lightStayOnTime) {
      // Switch light off in ascending series
      Serial.println("Light off in ascending series!"); 
      lightState = 0;
    }
    
  }
  else if  (lightState == 2){
    //Check if timer is zero
    if ((unsigned long)(millis() - lightPreviousMillis) >= lightStayOnTime) {
      // Switch light off in descending series
      Serial.println("Light off in descending series!"); 
      lightState = 0;
    }  
  }


  

/*
  //PIR 2
  
  */

  // LED 
  
  /*
  delay(1000);

  // Drive each servo one at a time using setPWM()
  servonum = 0;
  Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen <= steps; pulselen++) {
    pwm.setPWM(servonum, 0, expFunction(pulselen));
    delay(30);
  }

  servonum = 1;
  Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen <= steps; pulselen++) {
    pwm.setPWM(servonum, 0, expFunction(pulselen));
    delay(30);
  }
  // Hier alle weiteren Steppenstufen

  delay(1000); // Verzögerung wenn oben

  servonum = 0;
  Serial.println(servonum);
  for (uint16_t pulselen = steps; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, expFunction(pulselen));
    delay(30);
  }
  //pwm.setPWM(servonum, 0, 4096);

  servonum = 1;
  Serial.println(servonum);
  for (uint16_t pulselen = steps; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, expFunction(pulselen));
    delay(30);
  }
  //pwm.setPWM(servonum, 0, 4096);

  */

}

int expFunction(int x) {

  double y = pow(1.5157, x) - 1; 
  return (int) y;

}

/*
int expSteps(int interval) {

  steps = pow((interval - 1), 1/20); // In 20 steps
  return (int) y;

}
*/