/*
  Mecanum.h - Library for Mecanum wheels with two Devantech MD25 motor controllers.
  Created by Josh Villbrandt (http://javconcepts.com/), July 18, 2012.
  Released into the public domain.
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//#include "Wire.h"
#include "Mecanum.h"
#include "../MD25/MD25.h"
//#include "../I2C/I2C.h"

/*
* Constructors
*
* default address is B0 << 1 i.e. 0x58 as a 7 bit address for the wire lib
*/

Mecanum::Mecanum()
{
	this->frontController = MD25(0xB0 >> 1);
	this->backController = MD25(0xB2 >> 1);
	
	for(int i = 0; i < 4; i++)
		this->lastEncoderCounts[i] = 0;
}


/*
* Public Methods
*/

void Mecanum::cmd_vel(byte x, byte y, byte xy)
{
	// normalize
	int x2 = x - 128;
	int y2 = y - 128;
	int xy2 = xy - 128;
	int sum = abs(x2) + abs(y2) + abs(xy2);
	
	if(sum > 127) {
		x2 = (x2 * 127 / sum);
		y2 = (y2 * 127 / sum);
		xy2 = (xy2 * 127 / sum);
	}
	
	this->frontController.setMotor1Speed(x2 - y2 + 128 - xy2); // M_fl
	this->frontController.setMotor2Speed(x2 + y2 + 128 + xy2); // M_fr
	this->backController.setMotor1Speed(x2 + y2 + 128 - xy2); // M_bl
	this->backController.setMotor2Speed(x2 - y2 + 128 + xy2); // M_br
}

void Mecanum::getEncoderCounts(float *xytCounts)
{
	// get new encoder counts from MD25 boards
	int newEncoderCounts[4];
	newEncoderCounts[0] = this->frontController.getEncoder1(); // M_fl
	newEncoderCounts[1] = this->frontController.getEncoder2(); // M_fr
	newEncoderCounts[2] = this->backController.getEncoder1(); // M_bl
	newEncoderCounts[3] = this->backController.getEncoder2(); // M_br
	
	// find deltas
	int deltaEncoderCounts[4];
	for(int i = 0; i < 4; i++) {
		// check for overflow
		if(abs(this->lastEncoderCounts[i]) > COUNT_OVERFLOW && abs(newEncoderCounts[i]) > COUNT_OVERFLOW && sign(this->lastEncoderCounts[i]) != sign(newEncoderCounts[i])) {
			if(sign(this->lastEncoderCounts[i]) > 0)
				deltaEncoderCounts[i] = newEncoderCounts[i] - this->lastEncoderCounts[i] + INT_MAX;
			else
				deltaEncoderCounts[i] = newEncoderCounts[i] - this->lastEncoderCounts[i] - INT_MAX;
		}
		else deltaEncoderCounts[i] = newEncoderCounts[i] - this->lastEncoderCounts[i];
		
		// check for gross change -> MD25 board power cycled
		if(abs(deltaEncoderCounts[i]) > COUNT_RESET) deltaEncoderCounts[i] = 0;
		// no idea what the real value is in this case
		
		// save encoder counts
		this->lastEncoderCounts[i] = newEncoderCounts[i]; // i should grow up and just use a point after the for loop
	}
	
	// convert the motor counts into x, y, theta counts
	xytCounts[0] = (deltaEncoderCounts[0] + deltaEncoderCounts[1] + deltaEncoderCounts[2] + deltaEncoderCounts[3]) / 4;
	xytCounts[1] = (0 - deltaEncoderCounts[0] + deltaEncoderCounts[1] + deltaEncoderCounts[2] - deltaEncoderCounts[3]) / 4;
	xytCounts[2] = (0 - deltaEncoderCounts[0] + deltaEncoderCounts[1] - deltaEncoderCounts[2] + deltaEncoderCounts[3]) / 4;
	
	// debug 
	//xytCounts[0] = newEncoderCounts[0];
	//xytCounts[1] = newEncoderCounts[1];
	//xytCounts[2] = newEncoderCounts[2];
}

/*
* Private Methods
*/

