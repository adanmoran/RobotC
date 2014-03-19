
/*This will be a colour-detecting robot: It will follow a line based on its colour pattern.
 For example, if a center line is blue, there will be a green line on the right and
 a red line on the left so that the robot can stay on track. It will display the colour
 it is seeing on the screen, and when it reaches a black box at the end of the line it
 will dance and "sing".
 The ultrasonic sensor will be used to determine how far away the robot is from an object. If
 it is too close, the robot will slow down. If it is way too close, it will stop.
 The sound sensor will switch the direction of the robot.*/

//Declarations of defined words
#define SPEED 30
	//Light level stuff
#define FLOOR_LOWER 47
#define FLOOR_HIGHER 55
#define BLUE_LOWER 30
#define BLUE_HIGHER 40
#define YELLOW_LOWER 59
#define YELLOW_HIGHER 66
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
void Celebrate();
void DisplayImages();
void isLoud();
void SonarDistance();
//Declaration of global variables
int lightLevel, soundLevel, direction;
float ultraLevel;
bool finished;
task main()
{
	//Initialize global variables
	lightLevel = 0; soundLevel = 0; ultraLevel = 1; direction = 1; finished = false;
	//Declare sensors
	SensorType[0] = sensorLightActive; //port 1
	SensorType[1] = sensorSoundDB; //port 2
	SensorType[2] = sensorSONAR; //port 3
	//Enable PID on motors
	nMotorPIDSpeedCtrl[1] = mtrSpeedReg;
	nMotorPIDSpeedCtrl[2] = mtrSpeedReg;
	//Declare variables
	int colour, speed;
	//Main loop
	while(true)// !finished)
	{
			//Check distance from objects in front.
			SonarDistance();
			//Check if direction is switching
			isLoud();
			speed = SPEED * direction * ultraLevel;
			colour = SenseColour();
			nxtDisplayTextLine(2, "Sound level: %d", soundLevel);
			nxtDisplayTextLine(3, "Light Level: %d", lightLevel);
			nxtDisplayTextLine(4, "Speed: %d", speed);
			nxtDisplayTextLine(5, "ColourType: %d", colour);
			nxtDisplayTextLine(6, "Ultrasonic: %f", ultraLevel);
			//Drive or rotate based on the colour of the tape
			switch(colour)
			{
				case 0: //On Floor
					Drive(speed, speed);
					break;
				case 1: //On Blue
					Drive(speed/2, -speed/2);
					break;
				case 2: //On Yellow
					Drive(-speed/2, speed/2);
					break;
				case 3: //On Black
					Drive(0, 0);
					finished = true;
					break;
				case 4: //None of the above
					Drive(0,0);
					break;
			}
		wait10Msec(20); //delay 200 msecs before next loop repetition
	}
}

void Drive(int speedLeft, int speedRight)
{
	motor[1] = speedRight;
	motor[2] = speedLeft;
}

int SenseColour()
{
	lightLevel = SensorValue[0];
	int r = 4;

	//Check type of reflectivity and return a normalized value from 0-3.
	if( lightLevel >= FLOOR_LOWER && lightLevel <= FLOOR_HIGHER)
			r = 0;
	else if(lightLevel >= BLUE_LOWER && lightLevel <= BLUE_HIGHER)
			r = 1;
	else if(lightLevel >= YELLOW_LOWER && lightLevel <= YELLOW_HIGHER)
			r = 2;
	/*else if(lightLevel >= BLACK_LOWER && lightLevel <= BLACK_HIGHER)
			r = 3;*/
	return r;
}

/*
Method switches direction of robot it sound sensor is triggered past preset loudness variable*/
void isLoud()
{
	soundLevel = SensorValue[1];

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

void SonarDistance()
{
	int distance = SensorValue[2];
	nxtDisplayTextLine(7, "Distance: %d", distance);
	float scale = 1;
	if (distance >= SLOW_DISTANCE)
		scale = 1.0;
  else if (distance <= STOP_DISTANCE)
  	scale = 0.0;
 	else
 		scale = ((float)distance/(SLOW_DISTANCE-STOP_DISTANCE)) - ((float)STOP_DISTANCE/(SLOW_DISTANCE-STOP_DISTANCE));
 	ultraLevel = scale;
}
