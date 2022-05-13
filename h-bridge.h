#ifndef H_BRIDGE_H
#define H_BRIDGE_H

#include <Arduino.h>

class Motor
{
	public:
		// Constructor
		Motor(int pin1, int pin2);

		// Drive forward
		void fwd();

		// Drive reverse
		void reverse();

		// Brake drive
		void brake();
	
	private:
		int pin1, pin2;
};

#endif
