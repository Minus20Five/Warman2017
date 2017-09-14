
#include "Arduino.h"
#include "Encoder.h"
#include "PID_v1.h"
#include "Ultrasonic.h"
#include "L298N.h"

#define ENCODER_OPTIMIZE_INTERRUPTS


L298N leftMotor(10, 8, 9);
L298N rightMotor(13, 11, 12);

//2,3,18,19 -> interrupt pins DO NOT CHANGE!!!
Encoder encoderRightMotor   (18, 19);
Encoder encoderLeftMotor    (2,3);

void WaitForBallDrop(){
    //to be implemented with shock sensor
}


void Align2(int masterPower, int repetitions, unsigned int microseconds){

	Ultrasonic ultrasonicLeft (0,5,40000UL);
	Ultrasonic ultrasonicRight (6,7,40000UL);
	
	for (int i = 0; i < repetitions;i++){
		int power = masterPower * ((repetitions - i)/repetitions);
		
		if (ultrasonicLeft.distanceRead() > ultrasonicRight.distanceRead()){
			//nudge right
			leftMotor.backward(power);
			rightMotor.forward(power);
		} else {
			//nudge left
			leftMotor.forward(power);
			rightMotor.backward(power);
		}
		delayMicroseconds(microseconds);
	}	
	//stop the motors
	leftMotor.stop();
	rightMotor.stop();
}




//move the wheels really slowly until they align themselves
void Align(int motorPower, int tolerance, int numPulsesDelay){

	Ultrasonic ultrasonicLeft (0,5,40000UL);
	Ultrasonic ultrasonicRight (6,7,40000UL);

	long leftDistance;
	long rightDistance;
	long numPulses = 0;

	leftMotor.backward(motorPower);
	rightMotor.forward(motorPower);

	do{
		leftDistance = ultrasonicLeft.distanceRead();
		rightDistance = ultrasonicRight.distanceRead();
		numPulses++;
		
	} while(((leftDistance - rightDistance) > tolerance) ||  numPulses < numPulsesDelay);

	leftMotor.stop();
	rightMotor.stop();

}


void turnLeftDeg(int degrees, int power)
{
	//Reset encoders
	encoderLeftMotor.write(0);
	encoderRightMotor.write(0);

	//Determine tickGoal through experiment
	int tickGoal = (23 * degrees) / 10;

	//Start the motors in a left point turn.
	leftMotor.backward(power);
	rightMotor.forward(power);

	//Since the wheels may go at slightly different speeds due to manufacturing tolerances, etc., 
	//we need to test both encoders and control both motors separately. This may result in one motor
	//going for longer than another but it will ultimately result in a much more accurate turn.
	while(encoderRightMotor.read() < tickGoal || encoderLeftMotor.read() > -1 * tickGoal) {
		if(encoderRightMotor.read() > tickGoal) {
			rightMotor.stop();
		}
		if(encoderLeftMotor.read() < -1 * tickGoal) {
			leftMotor.stop();
		}
	}
	//Make sure both motors stop at the end of the turn.
	leftMotor.stop();
	rightMotor.stop();
}

void turnRightDeg(int degrees, int power)
{
	//Reset encoders
	encoderLeftMotor.write(0);
	encoderRightMotor.write(0);

	//Determine tickGoal
	int tickGoal = (23 * degrees) / 10;

	//Start the motors in a right point turn.
	leftMotor.forward(power);
	rightMotor.backward(power);

	//Since the wheels may go at slightly different speeds due to manufacturing tolerances, etc., 
	//we need to test both encoders and control both motors separately. This may result in one motor
	//going for longer than another but it will ultimately result in a much more accurate turn.
	while(encoderLeftMotor.read() < tickGoal || encoderRightMotor.read() > -1 * tickGoal) {
		if(encoderLeftMotor.read() > tickGoal) {
			leftMotor.stop();
		}
		if(encoderRightMotor.read()< -1 * tickGoal) {
			rightMotor.stop();
		}
	}
	//Make sure both motors stop at the end of the turn.
	leftMotor.stop();
	rightMotor.stop();
}

void DriveDistance(double distanceSetPoint, int distanceTolerance, double masterPower, double distanceKP, double moveStraightKP, double moveStraightKI, double moveStraightKD){

    //Initialize a PID controller to move straight
    double moveStraightError = 0, motorAdjust = 0, moveStraightSetpoint = 0;
    PID PID_MotorController(&moveStraightError, &motorAdjust, &moveStraightSetpoint, moveStraightKP, moveStraightKI, moveStraightKD, DIRECT);

	//Set to manual only if need to manually change the output
	PID_MotorController.SetMode(AUTOMATIC);				
	//Saturate the output
	PID_MotorController.SetOutputLimits(-(255 - masterPower), (255 - masterPower)); 
	//Refresh time. Shorter should be more accurate
	PID_MotorController.SetSampleTime(10);

	//send power to the motors to move foward
	leftMotor.forward(masterPower);
	rightMotor.forward(masterPower);

	//wait for garbage values
	delay(50);

    while((distanceSetPoint - encoderLeftMotor.read()) > distanceTolerance){
		moveStraightError = encoderLeftMotor.read() - encoderRightMotor.read();
		PID_MotorController.Compute();
		//Sets speed variable via PWM 
		leftMotor.forward(masterPower + motorAdjust);
		rightMotor.forward(masterPower - motorAdjust);
		//master power decreases based on square of the distance travelled
		masterPower -= distanceKP*encoderLeftMotor.read()*encoderLeftMotor.read();

		Serial.print(motorAdjust);
		Serial.print(" ");
		Serial.print(leftMotor.getSpeed());
		Serial.print(" ");
		Serial.print(rightMotor.getSpeed());
		Serial.print(" ");
		Serial.print(encoderLeftMotor.read());
		Serial.print(" ");
		Serial.print(encoderRightMotor.read());
		Serial.println(" ");

	}
	leftMotor.stop();
	rightMotor.stop();
}

void setup() { 
	Serial.begin(9600);
	leftMotor.setSpeed(0);
	rightMotor.setSpeed(0);
}

void loop() {
    WaitForBallDrop();
	DriveDistance(100000, 50, 230, 0, 3, 2, 1); // change this shit to proper values
}