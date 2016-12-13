#include <Bounce2.h>

#include <HID-Project.h>
#include <HID-Settings.h>

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>


//========== CONFIGURATION SETTINGS ==========
#define BOUNCE_WAIT 10
#define BOUNCE_COUNT 1
#define ACTIVE_BUTTONS 5

int buttonPins[] = {
  4, //button 1
  5, //button 2
  6, //button 3
  7, //button 4
  8, //button 5
  2, //button 6
  9, //button 7
  7, //button 8
  18, //button 9
  19, //button 1 back panel
  20, //button 2 back panel
  21, //button 3 back panel
  10, //joystick up
  16, //joystick down
  14, //joystick left
  15 // joystick right
};

const int xAxis = 18;         // joystick X axis
const int yAxis = 19;         // joystick Y axis

//========== END CONFIGURATION SETTINGS ==========

// Instantiate button state array
boolean buttonPressed[16];
// Instantiate a Bounce object array to store each debouncer in
Bounce debouncers[16];
//Instantiate a counter array for each button to debounce count timer
int debounceCount[16];

short quatx;
short quaty;
short quatrx;
short quatry;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

#define N  (2)

int RXLED = 17;  // The RX LED has a defined Arduino pin

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

bool hasBNO = true;

// parameters for reading the joystick:
int range = 254;               // output range of X or Y movement
int responseDelay = 5;        // response delay of the mouse, in ms
int threshold = range / 4;    // resting threshold
int center = range / 2;       // resting position value

void setup() {
  // put your setup code here, to run once:

inputString.reserve(200);
  Gamepad.begin();

  // Create debounce instances :
   for (int i = 0; i < ACTIVE_BUTTONS; i++) {
     debouncers[i] = Bounce();
     debounceCount[i] = 0;
     pinMode(buttonPins[i],INPUT_PULLUP);
     (debouncers[i]).attach(buttonPins[i]);
     (debouncers[i]).interval(BOUNCE_WAIT);
        delay(100);
     buttonPressed[i] = false;
   }

  //debug:
  //Serial.begin(9600);
  //Serial.setTimeout(10);
  //Serial.println("Orientation Sensor Test"); Serial.println("");


  pinMode(RXLED, OUTPUT);  // Set RX LED as an output
  digitalWrite(RXLED, LOW);


  if(hasBNO){
    /* Initialise the sensor */
    if (!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
    }

    delay(1000);

    bno.setExtCrystalUse(true);
  }

}

void loop() {
  
  // put your main code here, to run repeatedly:
  /* Get a new sensor event */
  //sensors_event_t event;
  //bno.getEvent(&event);

// print the string when a newline arrives or it's been a while:

  if(hasBNO){
      imu::Quaternion quat = bno.getQuat();
        /* Display the quat data ordered: qw, x, y, z*/

        quatx = (short) ( (int) ( quat.x()*32767 ) );
        quaty = (short) ( (int) ( quat.y()*32767 ) );
        quatrx = (short) ( (int) ( quat.z()*32767 ) );
        quatry = (short) ( (int) ( quat.w()*32767 ) );
        
        Gamepad.xAxis(quatx);
        Gamepad.yAxis(quaty);
        Gamepad.rxAxis(quatrx);
        Gamepad.ryAxis(quatry);

        Gamepad.write();
  }

  
  for (int j = 0; j < ACTIVE_BUTTONS; j++) { //iterate over each button (pin)
    
     (debouncers[j]).update(); //check current value
     int value = (debouncers[j]).read();
     
     if ( value == LOW ) { //if button pressed
       
       if(debounceCount[j] == BOUNCE_COUNT && buttonPressed[j] == false) { //the button has been held down long enough and it hasn't been previously registered as pressed
          if(j==0){
            //NKROKeyboard.press(KEY_1);
            Gamepad.press(1);
          }
          if(j==1){
            //NKROKeyboard.press(KEY_2);
            Gamepad.press(2);
          }
          if(j==2){
            //NKROKeyboard.press(KEY_2);
            Gamepad.press(3);
          }
          if(j==3){
            //NKROKeyboard.press(KEY_2);
            Gamepad.press(4);
          }
          if(j==4){
            //NKROKeyboard.press(KEY_2);
            Gamepad.press(5);
          }
          Gamepad.write();
          digitalWrite(RXLED, HIGH);
          //Keyboard.send();
          //Keyboard.press('r');
          //Joystick.pressButton(buttonPresets[j]);
          buttonPressed[j] = true;
          //delay(50);
          //NKROKeyboard.releaseAll();
          //Gamepad.releaseAll();
          //Gamepad.write();
          //delay(50);
        } else {
            if(debounceCount[j] < BOUNCE_COUNT) { 
              debounceCount[j] = debounceCount[j] + 1; //else increment the count
            }
        }
        
      } else { //button is not pressed
        
        if(debounceCount[j] > 0) {
          debounceCount[j] = debounceCount[j] - 1; //keep decreasing count unless 0
        } else {
           //Keyboard.release(char(buttonPresets[j])); //if 0 then release button
           //NKROKeyboard.releaseAll();
           //Joystick.releaseButton(buttonPresets[j]);
           buttonPressed[j] = false;

          if(j==0){
            //NKROKeyboard.press(KEY_1);
            Gamepad.release(1);
          } 
          if(j==1){
            //NKROKeyboard.press(KEY_2);
            Gamepad.release(2);
          }
          if(j==2){
            //NKROKeyboard.press(KEY_2);
            Gamepad.release(3);
          }
          if(j==3){
            //NKROKeyboard.press(KEY_2);
            Gamepad.release(4);
          }
          if(j==4){
            //NKROKeyboard.press(KEY_2);
            Gamepad.release(5);
          }
            Gamepad.write();
            digitalWrite(RXLED, LOW);
        }
        
      }
  }

   // read and scale the two axes:
  int8_t xReading = readAxis(xAxis);
  int8_t yReading = readAxis(yAxis);

  Gamepad.zAxis(map(xReading,-255,255,-255,255));
  Gamepad.rzAxis(map(yReading,-255,255,-255,255));
  Gamepad.write();
 
}

void serialEventRun(void) {
  if (Serial.available()) serialEvent();
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    //Serial.print(inChar);
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    //if (inChar == '\r') {
    if(inputString.endsWith("\r") ) {
      stringComplete = true;
      //thrust = inputString.charAt( inputString.length()-2 );
      //digitalWrite(RXLED, HIGH);
    }
  }
}

/*
  reads an axis (0 or 1 for x or y) and scales the
 analog input range to a range from 0 to <range>
 */

int readAxis(int thisAxis) {
  // read the analog input:
  int reading = analogRead(thisAxis);

  // map the reading from the analog input range to the output range:
  reading = map(reading, 0, 1023, 0, range);

  // if the output reading is outside from the
  // rest position threshold,  use it:
  int distance = reading - center;

  if (abs(distance) < threshold) {
    distance = 0;
  }

  // return the distance for this axis:
  return distance;
}

