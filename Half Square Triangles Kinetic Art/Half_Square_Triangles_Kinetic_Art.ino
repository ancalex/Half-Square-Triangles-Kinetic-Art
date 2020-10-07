#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <AccelStepper.h> //http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
#include <MCP3017AccelStepper.h> //https://github.com/johannilsson/MCP3017AccelStepper
#include "HSTModels.h"

#define STEPPER_COUNT 16 //number of steppers

const int buttonPin = 6;     // the number of the pushbutton pin
const int homingPin =  7;      // the number of the homing switches pin
bool random_mode = 0;
const int homingCompensation[] = {152, 209, 188, 204, 192, 210, 204, 180, 216, 197, 192, 221, 232, 252, 212, 214}; //after running HST_Kinetic_Art_Init

Adafruit_MCP23017 mcp0;
Adafruit_MCP23017 mcp1;
Adafruit_MCP23017 mcp2;
Adafruit_MCP23017 mcp3;

MCP3017AccelStepper steppers[STEPPER_COUNT] = {
		// interface, I/O expander pins
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 0, 2, 1, 3),
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 4, 6, 5, 7),
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 8, 10, 9, 11),
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 15, 13, 14, 12),
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 0, 2, 1, 3),
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 4, 6, 5, 7),
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 8, 10, 9, 11),
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 15, 13, 14, 12),
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 3, 1, 2, 0), //
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 4, 6, 5, 7),
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 11, 9, 10, 8), //
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 12, 14, 13, 15),
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 3, 1, 2, 0),  //
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 7, 5, 6, 4), //
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 11, 9, 10, 8),  //
		MCP3017AccelStepper(AccelStepper::HALF4WIRE, 15, 13, 14, 12), //
};

bool homingState = HIGH;         // variable for reading the homing switches status
long initial_homing=-1;  // Used for homing at startup
int steppers_running = 16; // number of stepers running for making a HST model
int modelNumber = 0;

void set_steppers (int index) {  // first four steppers are attached to the first expander, next four to the second expander and so on
	if (index < 4) steppers[index].setMcp(mcp3);
	if ((index >= 4) && (index < 8)) steppers[index].setMcp(mcp2);
	if ((index >= 8) && (index < 12)) steppers[index].setMcp(mcp1);
	if (index >= 12) steppers[index].setMcp(mcp0);
}

void setup()
{
	Serial.begin(115200);
	Serial.println("Start Homing");
	// initialize the button pin as an input:
	pinMode(buttonPin, INPUT_PULLUP);
	delay(10);
	if (buttonPin == LOW) {
		random_mode = 1;
	}
	// initialize the homing switches pin as an input:
	pinMode(homingPin, INPUT_PULLUP);
	mcp0.begin();
	TWBR = 12; // I2C bus speed 400khz
	mcp1.begin(1);
	TWBR = 12;
	mcp2.begin(2);
	TWBR = 12;
	mcp3.begin(3);
	TWBR = 12;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		Serial.print("Steppers count:");  Serial.println(i);
		set_steppers(i);
		steppers[i].enableOutputs();
		steppers[i].setMaxSpeed(150.0);
		steppers[i].setAcceleration(100.0);
		Serial.print("Stepper "); Serial.print(i); Serial.println(" is Homing . . . . . ");
		// read the state of the homing switches value:
		while (digitalRead(homingPin)) {
			steppers[i].moveTo(initial_homing);  // Set the position to move to
			initial_homing--;  // Decrease by 1 for next move if needed
			steppers[i].run();  // Start moving the stepper
			delay(5);
		}
		steppers[i].setCurrentPosition(0);
		initial_homing=1;
		// Make the Stepper move CW until the switch is deactivated
		while (!digitalRead(homingPin)) {
			steppers[i].moveTo(initial_homing);  // Set the position to move to
			initial_homing++;  // Increase by 1 for next move if needed
			steppers[i].run();  // Start moving the stepper
			delay(5);
		}
		steppers[i].setCurrentPosition(0);
		steppers[i].runToNewPosition(homingCompensation[i]);
		steppers[i].setCurrentPosition(0);
		initial_homing=-1;
		Serial.println("Homing Completed");
	}
	delay(4000);
}

void loop()
{
	//4076 steps per full revolution (in half step mode)
	if (random_mode) {
		int random_steppers_list[16];
		for (int i = 0; i < 16; i++) { //initialize the stepper list, in order from 0 to 15
			random_steppers_list[i] = i;
		}
		int seed = analogRead(2); //try to randomize a little bit
		randomSeed(seed);
		for (int i = 0; i < 16; i++) { //shuffle the stepper list
			int j = random(0,i);  // https://www.rosettacode.org/wiki/Knuth_shuffle
			int temp = random_steppers_list[i];
			random_steppers_list[i]= random_steppers_list[j];
			random_steppers_list[j] = temp;
		}
		int steppersNumber = random(16); // how many steppers are rotating
		for (int i = 0; i < steppersNumber; i++) {
			steppers[random_steppers_list[i]].moveTo(random(4)*1019);  //random movement - 0, 90, 180, 270 degrees
			steppers_running = 16;
		}
	}
	else {  //model mode
		for (int i = 0; i < STEPPER_COUNT; i++) {
			set_steppers(i);
			steppers[i].moveTo(HSTM[modelNumber][i]*1019);
			//Serial.print(HSTM[modelNumber][i]);Serial.print("-");
		}
		Serial.println("");
		modelNumber++;
		if (modelNumber == 73) {modelNumber = 0;}
		steppers_running = 16;
	}
	while (steppers_running > 0 ) { //move the steppers until all has arrived to desired position
		for (int i = 0; i < STEPPER_COUNT; i++) {
			set_steppers(i);
			if (steppers[i].distanceToGo() != 0) {
				steppers[i].run();
			}
			else {
				steppers_running--;
			}
		}
		if (steppers_running > 0) {steppers_running = 16;}
	}
	delay(4000);
}
