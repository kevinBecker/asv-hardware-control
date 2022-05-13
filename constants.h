#ifndef CONSTANTS_H
#define CONSTANTS_H

const unsigned long SERIAL_PERIOD = 1000; // Loop period for the serial
const unsigned long ACTUATOR_PERIOD = 2; // Loop period for the actuator
const unsigned long POLLING_PERIOD =  5; // Loop period for the LA polling rate

const unsigned long MISSED_MSG_FOR_TIMEOUT = 5; //Number of missed serial msgs before a timeout

const double RUDDER_TOLERANCE = 0.03; // X%
const double SLOWDOWN_RANGE   = 0.25; // once within X% it slows down to enable more accurate positioning
const double SLOWDOWN_SPEED   = 0.12;  // this is the speed it slows down to

const bool DEBUG = false;               //debug mode prints more values
const bool DEMO = true;
const unsigned long DEMO_WAIT = 3000; //seconds to wait between each demo message


//LA self calibration configs
const bool SELF_CALIBRATE = false;       // self-calibration re-finds the analog min and max
// Only used if SELF_CALIBRATE = true
const unsigned long TIME_TO_ENDS = 3000; // 3 seconds to reach the ends of travel. Used for setup with the analog high and low
const unsigned int ANALOG_OFFSET = 10;  // adds/subtracts this to adjust the min/max LA bounds.
// LA potentiometer bounds (only used if SELF_CALIBRATE = false)
double ANALOG_HIGH = 440;
double ANALOG_LOW = 50;


const unsigned int MIN_THROTTLE = 0.05; // minimum throttle value (+/-), prevents motor jittering

// Values for throttle PWM
const double FWD_PWM_VALUE = 2000;
const double NEUTRAL_PWM_VALUE = 1089;
const double REV_PWM_VALUE = 1000;
const double THROTTLE_OFF_VALUE = 0;

#endif
