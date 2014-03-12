
/*This will be a colour-detecting robot: It will follow a line based on its colour pattern.
 For example, if a center line is blue, there will be a green line on the right and
 a red line on the left so that the robot can stay on track. It will display the colour
 it is seeing on the screen, and when it reaches a black box at the end of the line it
 will dance and "sing".
 The ultrasonic sensor will be used to determine how far away the robot is from an object. If
 it is too close, the robot will slow down. If it is way too close, it will stop.
 The sound sensor will switch the direction of the robot.*/

//Declarations of defined words
#define SPEED 50
	//Light level stuff
#define FLOOR_LOWER 48
#define FLOOR_HIGHER 53
#define BLUE_LOWER 32
#define BLUE_HIGHER 47
#define RED_LOWER 0
#define RED_HIGHER 0
#define BLACK_LOWER 0
#define BLACK_HIGHER 0
	//Sound level stuff
#define LOUD 60
	//Distance level stuff
#define STOP_DISTANCE 25
#define SLOW_DISTANCE 40
//Declarations of types

//Declarations of functions
void Drive(int speedLeft,int speedRight);
int SenseColour();
void celebrate();
void displayImages();
void isLoud();
int sonarDistance();
//Declaration of global variables
int lightLevel, soundLevel, ultraLevel, direction;
task main()
{
	//Initialize global variables
	lightLevel = 0; soundLevel = 0; ultraLevel = 0; direction = 1;
	//Declare sensors
	SensorType[1] = sensorLightActive;
	SensorType[2] = sensorSoundDB;
	SensorType[3] = sensorSONAR;
	//Enable PID on motors
	nMotorPIDSpeedCtrl[1] = mtrSpeedReg;
	nMotorPIDSpeedCtrl[2] = mtrSpeedReg;
	//Declare variables
	int colour, speed;
	//Main loop
	while(true)
	{
			//Check if direction is switching
			isLoud();
			speed = SPEED * direction;//*sonarDistance();
			colour = SenseColour();
			nxtDisplayTextLine(2, "Sound level: %d", soundLevel);
			nxtDisplayTextLine(3, "Light Level: %d", lightLevel);
			nxtDisplayTextLine(4, "Speed: %d", speed);
			nxtDisplayTextLine(5, "ColourType: %d", colour);
			//Drive or rotate based on the colour of the tape
			switch(colour)
			{
				case 0:
					Drive(speed, speed);
					break;
				case 1:
					Drive(speed/2, -speed/2);
					break;
				case 2:
					Drive(0, 0);
					break;
				case 3:
					Drive(0, 0);
					break;
				case 4:
					Drive(0,0);
					break;
			}
			//Drive(SPEED, SPEED);
			//Debugging displays.
			/*nxtDisplayTextLine(1, "Sound level: %d", soundLevel);
			nxtDisplayTextLine(2, "Ultrasonic level:");
			nxtDisplayTextLine(3, "  %d", ultraLevel);
			nxtDisplayTextLine(4, "Light level: %d",lightLevel);*/
		wait10Msec(20); //delay 200 msecs before next loop repetition
		eraseDisplay();
	}
}

void Drive(int speedLeft, int speedRight)
{
	motor[1] = speedRight;
	motor[2] = speedLeft;
}

int SenseColour()
{
	lightLevel = SensorValue[1];
	int r = 4;

	//Check type of reflectivity and return a normalized value from 0-3.
	if( lightLevel >= FLOOR_LOWER && lightLevel <= FLOOR_HIGHER)
			r = 0;
	else if(lightLevel >= BLUE_LOWER && lightLevel <= BLUE_HIGHER)
			r = 1;
	/*else if(lightLevel >= RED_LOWER && lightLevel <= RED_HIGHER)
			r = 2;
	else if(lightLevel >= BLACK_LOWER && lightLevel <= BLACK_HIGHER)
			r = 3;
	else
			r = 4;*/
	return r;
}

void isLoud()
{
	soundLevel = SensorValue[2];

	if(soundLevel > LOUD && direction == 1)
	{
		motor[1] = 0; motor[2] = 0;
		direction = -1;
		wait10Msec(20);
	}
	else if(soundLevel > LOUD && direction == -1)
	{
		motor[1] = 0; motor[2] = 0;
		direction = 1;
		wait10Msec(20);
	}
}
