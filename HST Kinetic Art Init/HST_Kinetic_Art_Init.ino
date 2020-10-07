#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <AccelStepper.h>
#include <MCP3017AccelStepper.h>

#define STEPPER_COUNT 16

const int homingPin =  7;      // the number of the homing switches pin
int homingCompensation[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

Adafruit_MCP23017 mcp0;
Adafruit_MCP23017 mcp1;
Adafruit_MCP23017 mcp2;
Adafruit_MCP23017 mcp3;

MCP3017AccelStepper steppers[STEPPER_COUNT] = {
		// interface, pins
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
long initial_homing=-1;  // Used to Home Stepper at startup
int stepper_number;

void set_steppers (int index) {
	if (index < 4) steppers[index].setMcp(mcp3);
	if ((index >= 4) && (index < 8)) steppers[index].setMcp(mcp2);
	if ((index >= 8) && (index < 12)) steppers[index].setMcp(mcp1);
	if (index >= 12) steppers[index].setMcp(mcp0);
}

void setup()
{
	Serial.begin(115200);
	Serial.println("Start Initialization");
	// initialize the homing switches pin as an input:
	pinMode(homingPin, INPUT_PULLUP);
	mcp0.begin();
	mcp1.begin(1);
	mcp2.begin(2);
	mcp3.begin(3);

}

void loop()
{
	if (Serial.available()) {
		int serial_data = Serial.read();
		if (serial_data >= 'A' && serial_data < 'Q') {
			stepper_number = serial_data - 65;
			Serial.print("Active stepper:");  Serial.println(stepper_number);
			set_steppers(stepper_number);
			steppers[stepper_number].enableOutputs();
			steppers[stepper_number].setMaxSpeed(150.0);
			steppers[stepper_number].setAcceleration(100.0);
			if (homingCompensation[stepper_number] == 0) {
			Serial.print("Stepper "); Serial.print(stepper_number); Serial.println(" is Homing . . . . . ");
			// read the state of the homing switches value:
			while (digitalRead(homingPin)) {
				steppers[stepper_number].moveTo(initial_homing);  // Set the position to move to
				initial_homing--;  // Decrease by 1 for next move if needed
				steppers[stepper_number].run();  // Start moving the stepper
				delay(5);
			}
			steppers[stepper_number].setCurrentPosition(0);
			initial_homing=1;
			// Make the Stepper move CW until the switch is deactivated
			while (!digitalRead(homingPin)) {
				steppers[stepper_number].moveTo(initial_homing);  // Set the position to move to
				initial_homing++;  // Increase by 1 for next move if needed
				steppers[stepper_number].run();  // Start moving the stepper
				delay(5);
			}
			}
		}
		else if (serial_data == '+') {
			int position = steppers[stepper_number].currentPosition();
			position++;
			homingCompensation[stepper_number] = position;
			steppers[stepper_number].runToNewPosition(position);
			Serial.print("Current homing compensation:"); Serial.println(steppers[stepper_number].currentPosition());
		}
		else if (serial_data == '-') {
			int position = steppers[stepper_number].currentPosition();
			position--;
			homingCompensation[stepper_number] = position;
			steppers[stepper_number].runToNewPosition(position);
			Serial.print("Current compensation:"); Serial.println(steppers[stepper_number].currentPosition());
		}
		else if (serial_data == 'R') {
			for (int i = 0; i < 16; i++) {
				Serial.print(homingCompensation[i]);
				Serial.print(", ");
			}
		}
	}
}
