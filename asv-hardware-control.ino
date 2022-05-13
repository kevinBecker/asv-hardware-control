#include <Servo.h>
#include "h-bridge.h"
#include "pins.h"
#include "constants.h"

// Serial communication variables
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;
boolean hasFirstMsg = false;

//linear actuator variables
double rudderSetpoint = 0;
double rudderPosition = 0;
double rudderError = 0;
int rudderPotVoltage;
const int offset = 1; // Required for LA library
Motor rudderLA = Motor(LA_IN1, LA_IN2);
int demo_count = 0;

// Motor variables
int throttle;    //Servo PWM setting for motor speed. Full fwd: 2000, neutral: 1500, full ast: 1000
Servo esc;              //object to control brushless motor with esc
double throttleSetpoint = 0;
String outMsg;

// General variables
static unsigned long lastLoop = 0;

void setup() {
  
    Serial.begin(115200);
    setupLA();
    setupMotor();
    Serial.println("<Arduino is ready>");
}

long previousMillis_serial = 0;
long previousMillis_actuator = 0;
long previousMillis_polling = 0;
long missed_msgs = 0;
long previousMillis_demo = 0;
long analog_offset = 0;


void loop() {
    recvWithStartEndMarkers();
    unsigned long currentMillis = millis();

    
    if (DEMO){
      if (currentMillis - previousMillis_demo > DEMO_WAIT) {
        previousMillis_demo = currentMillis;
        runDemo();
      }
    }

    //accepts data from the serial
    if (currentMillis - previousMillis_serial > SERIAL_PERIOD) {
        previousMillis_serial = currentMillis;
        Serial.println(outMsg);
        if (newData) {
            missed_msgs = 0;
            //parse data from serial
            parseInput(receivedChars);
            newData = false;
            hasFirstMsg = true;            
        }
        else{
          if (hasFirstMsg and not DEBUG){
            missed_msgs ++;
          }
        }
    }
    
    
    // Updates the LA pins, sets it to FWD REV or brake
    if (currentMillis - previousMillis_actuator > ACTUATOR_PERIOD) {
       previousMillis_actuator = currentMillis;
       // Do the stuff
       updateMotor();
       updateRudder();
    }

    // Updates the LA reading. Polling too fast returns erranious readings
    if (currentMillis - previousMillis_polling > POLLING_PERIOD) {
       previousMillis_polling = currentMillis;
       // Do the stuff
       calculateRudderError();
    }
}

void setupLA() {
    pinMode(LA_IN1, OUTPUT);
    pinMode(LA_IN2, OUTPUT);


    if (SELF_CALIBRATE){
      Serial.println("Calibrating rudder...");
      rudderLA.fwd();
      delay(TIME_TO_ENDS);
      analog_high = analogRead(LA_POT);
      Serial.print("LA limit 1 = ");
      Serial.println(analog_low);
      rudderLA.reverse();
      delay(TIME_TO_ENDS);
      analog_low = analogRead(LA_POT);
      Serial.print("LA limit 2 = ");
      Serial.println(analog_low);
      rudderLA.brake();  
      rudderError = 1;
    }
    analog_offset = (analog_high - analog_low) * OFFSET;
}

double calculateRudderError() {
    int la_reading = analogRead(LA_POT);
    rudderPosition = scalePotentiometerInput(la_reading);
    if (DEBUG){
      Serial.print("la_reading = ");
      Serial.println(la_reading); 
    }
    double newError = rudderSetpoint - rudderPosition;
    rudderError = newError;
}

//scales the input to be between 1 and -1 using analog_high and LOW as the min and max
double scalePotentiometerInput(int input) {
    return 2 / (analog_high - analog_low) * ((double)input - analog_high) + 1;
}

double pulse_counter;
void updateRudder() {
    double error = rudderError;
    if (DEBUG){
      Serial.print("Error = ");
      Serial.println(error);
    }
    if (abs(error) > RUDDER_TOLERANCE) {
        // if it is far from error=0, goes at full speed
        if (abs(error) > SLOWDOWN_RANGE){
            pulse_counter = 0;
            if(error > 0) {
              rudderLA.fwd();
              outMsg = "Driving rudder forward";
            } else if(error < 0) {
              rudderLA.reverse();
              outMsg = "Driving rudder reverse";
            }
        }
        // if the error is low but still outside the tolerence
        // runs at a lower speed by pulsing the output
        else{
            pulse_counter += SLOWDOWN_SPEED;
    
            // Move rudder
            if (pulse_counter >= 1){
              pulse_counter --;
              if(error > 0) {
                rudderLA.fwd();
                outMsg = "Pulsing rudder forward";
              }
              else if(error < 0) {
                rudderLA.reverse();
                outMsg = "Pulsing rudder reverse";
              }
            }
            // Pause ruder
            else {
              rudderLA.brake();
              outMsg = "Pulsing rudder brake";
            }
         }		
      } 
    else {
        rudderLA.brake();
        outMsg = "Braking rudder";
    }
}




void setupMotor() {
    throttle = NEUTRAL_PWM_VALUE;
    esc.attach(MOTOR_PWM);
    delay(1000);
}



void updateMotor() {
  
  // Forward Scaling
  if (throttleSetpoint > 0){
    throttle = throttleSetpoint * (FWD_PWM_VALUE - NEUTRAL_PWM_VALUE) + NEUTRAL_PWM_VALUE;
  }
  
  // Reverse Scaling
  else{
    throttle = throttleSetpoint * (NEUTRAL_PWM_VALUE - REV_PWM_VALUE) + NEUTRAL_PWM_VALUE;
  }
  
  // Timeout turns rudder off
  if (missed_msgs >= MISSED_MSG_FOR_TIMEOUT){
    throttle = THROTTLE_OFF_VALUE;
    outMsg = "Timeout activated! Throttle turned off.";
  }
  
  // Prevents motor jittering when close to 0 by setting it to the min throttle
  if (abs(throttleSetpoint) < MIN_THROTTLE){
    // Gets the sign of throttleSetpoint and assigns it to the MIN_THROTTLE
    throttle = throttleSetpoint/abs(throttleSetpoint) * MIN_THROTTLE; 
  }
  
  esc.write(throttle);
}




void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

 // if (Serial.available() > 0) {
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

// Read the serial input
void parseInput(String input) {
    String received = input;
    received.replace("<","");
    received.replace(">","");
    int delimiter = received.indexOf(",");
    String first = received.substring(0, delimiter);
    String second = received.substring(delimiter + 1);
    throttleSetpoint = first.toDouble();
    Serial.println(throttleSetpoint);
    rudderSetpoint = second.toDouble();
    Serial.println(rudderSetpoint);
}


// Implementation of signum
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}



void runDemo(){
   Serial.print("Demo count: "); 
        Serial.println(demo_count);   
        previousMillis_demo = currentMillis;
        if (demo_count == 0){
          rudderSetpoint = 1;
          Serial.print("DEMO = ");
          Serial.println(rudderSetpoint);
        }
        if (demo_count == 1){
          rudderSetpoint = .5;
          Serial.print("DEMO = ");
          Serial.println(rudderSetpoint);
        }
        if (demo_count == 2){
          rudderSetpoint = 0;
          Serial.print("DEMO = ");
          Serial.println(rudderSetpoint);
        }
        if (demo_count == 3){
          rudderSetpoint = -.5;
          Serial.print("DEMO = ");
          Serial.println(rudderSetpoint);
        }
        if (demo_count == 4){
          rudderSetpoint = -1;
          Serial.print("DEMO = ");
          Serial.println(rudderSetpoint);

        }
         demo_count ++;
         if (demo_count == 5){
          demo_count = 0;
          Serial.println("reset demo");
         }
   }
}
