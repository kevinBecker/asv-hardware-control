#include "h-bridge.h"
#include <Arduino.h>

Motor::Motor(int pin1, int pin2)
{
	this->pin1 = pin1;
	this->pin2 = pin2;

	pinMode(pin1, OUTPUT);
	pinMode(pin2, OUTPUT);
}

void Motor::fwd()
{
	digitalWrite(pin1, HIGH);
	digitalWrite(pin2, LOW);
}

void Motor::reverse()
{
	digitalWrite(pin1, LOW);
	digitalWrite(pin2, HIGH);
}

void Motor::brake()
{
	digitalWrite(pin1, LOW);
	digitalWrite(pin2, LOW);
}
