/*This will be a colour-detecting robot: It will follow a line based on its colour pattern.
 For example, if a center line is blue, there will be a green line on the right and
 a red line on the left so that the robot can stay on track. It will display the colour
 it is seeing on the screen, and when it reaches a black box at the end of the line it
 will dance and "sing".
 The ultrasonic sensor will be used to determine how far away the robot is from an object. If
 it is too close, the robot will slow down. If it is way too close, it will stop.
 The sound sensor will switch the direction of the robot.*/

//Speed stuff
#define SPEED 15

//Light level stuff
#define FLOOR_LOWER 47
#define FLOOR_HIGHER 55
#define BLUE_LOWER 30
#define BLUE_HIGHER 40
#define YELLOW_LOWER 59
#define YELLOW_HIGHER 66
#define BLACK_LOWER 0
#define BLACK_HIGHER 29

	//Sound level stuff
#define LOUD 60

	//Distance level stuff
#define STOP_DISTANCE 25
#define SLOW_DISTANCE 40

//Declarations of types
typedef byte Image[9][9];

/*MUSICAL NOTE DECLARATIONS*/
//FIFTH OCTAVE NOTE FREQUENCIES
#define	C5  523	//	C	note frequency
#define	D5  587	//	D	note frequency
#define	F5  698	//	F	note frequency
#define	G5  784	//	G	note frequency
#define	A5  880	//	A	note frequency
#define	As5 932	//	A	sharp note frequency
//SIXTH OCTAVE NOTE FREQUENCIES
#define	C6  1046	//	note frequency
#define	D6  1175	//	note frequency
#define	P		0 //note frequency (a rest note)

/*NOTE DURATION FACTORS (THEY MULTIPLY THE WHOLE NOTE DURATION)*/
#define	N1   1.0     //whole note
#define	N2   0.5     //half note
#define	N2h  0.75    //dotted half note
#define	N4   0.25    //quarter note
#define	N4h  0.375   //dotted quarter note
#define	N8   0.125   //eighth note
#define	N8h  0.1875  //dotted eighth note
#define	N16  0.0625  //sixteenth note
#define	N16h	 0.09375 //dotted	sixteenth note
#define	N32	 0.03125	 //thirty-secondth note
#define	N32h	 0.046875 //dotted thirty-secondth note
#define	N64	 0.03

//Declarations of functions
void Drive(int speedLeft,int speedRight);
int SenseColour();
void Celebrate();
void DisplayImages(Image image);
void isLoud();
void SonarDistance();

//Called by celebration method
task motion();
void CelebrateRotate(int speed, int msecs);
void CelebrateDrive(int speed, int msecs);

//Declaration of global variables
int lightLevel, soundLevel, direction, noteindex;
float ultraLevel;
bool finished;

int notesF[] =
				{	P,G5,A5,As5,C6,F5,P,F5,
				  G5,A5,D5,P,D5,C5,P,C5,
				  G5,P,G5,A5,P,A5,P,A5,
				  As5,C6,F5,P,F5,G5,A5,C6,
				  G5,A5,As5,A5,G5,F5,G5,
				};

float notesD[] =
				{	N64,N4,N4,N4,N2,N2,N64,N2,
					N4,N4,N1,N64,N4,N4,N64,N4,
					N2,N64,N2h,N4,N64,N2h,N64,N4,
					N4,N2,N2,N64,N2,N4,N2,N1,
					N4,N4,N2,N4,N2h,N4,N1, 0
				};

	byte leftArrow[9][9]=
				{	{0,0,0,0,0,0,0,0,0},
					{0,0,0,1,0,0,0,0,0},
					{0,0,1,1,0,0,0,0,0},
					{0,1,1,1,1,1,1,1,1},
					{1,1,1,1,1,1,1,1,1},
					{0,1,1,1,1,1,1,1,1},
					{0,0,1,1,0,0,0,0,0},
					{0,0,0,1,0,0,0,0,0},
					{0,0,0,0,0,0,0,0,0}
				};

	byte rightArrow[9][9]=
				{	{0,0,0,0,0,0,0,0,0},
					{0,0,0,0,0,1,0,0,0},
					{0,0,0,0,0,1,1,0,0},
					{1,1,1,1,1,1,1,1,0},
					{1,1,1,1,1,1,1,1,1},
					{1,1,1,1,1,1,1,1,0},
					{0,0,0,0,0,1,1,0,0},
					{0,0,0,0,0,1,0,0,0},
					{0,0,0,0,0,0,0,0,0}
				};

		byte star[9][9]=
				{	{1,0,0,0,1,0,0,0,1},
					{0,1,0,0,1,0,0,1,0},
					{0,0,1,0,1,0,1,0,0},
					{0,0,0,1,1,1,0,0,0},
					{1,1,1,1,1,1,1,1,1},
					{0,0,0,1,1,1,0,0,0},
					{0,0,1,0,1,0,1,0,0},
					{0,1,0,0,1,0,0,1,0},
					{1,0,0,0,1,0,0,0,1}
				};

		byte checkmark[9][9]=
				{	{0,0,0,0,0,0,0,0,0},
					{0,0,0,0,0,0,0,0,1},
					{0,0,0,0,0,0,0,1,1},
					{0,0,0,0,0,0,1,1,0},
					{0,0,0,0,0,1,1,0,0},
					{1,0,0,0,1,1,0,0,0},
					{1,1,0,1,1,0,0,0,0},
					{0,1,1,1,0,0,0,0,0},
					{0,0,1,0,0,0,0,0,0}
				};

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
	while(finished == false)
	{
			eraseDisplay();
			//Check distance from objects in front.
			SonarDistance();
			//Check if direction is switching
			isLoud();
			speed = SPEED * direction * ultraLevel;
			colour = SenseColour();
			/*nxtDisplayTextLine(2, "Sound level: %d", soundLevel);
			nxtDisplayTextLine(3, "Light Level: %d", lightLevel);
			nxtDisplayTextLine(4, "Speed: %d", speed);
			nxtDisplayTextLine(5, "ColourType: %d", colour);
			nxtDisplayTextLine(6, "Ultrasonic: %f", ultraLevel);*/
			//Drive or rotate based on the colour of the tape
			switch(colour)
			{
				case 0: //On Floor
					DisplayImages(checkmark);
					nxtDisplayBigStringAt(20, 35, "FLOOR");
					Drive(speed, speed);
					break;
				case 1: //On Blue
					eraseDisplay();
					if(direction == 1)
						DisplayImages(leftArrow);
					else if(direction == -1)
						DisplayImages(rightArrow);
					nxtDisplayBigStringAt(30, 35, "BLUE");
					Drive(speed/2, -speed/2);
					break;
				case 2: //On Yellow
					if(direction == 1)
						DisplayImages(rightArrow);
					else if(direction == -1)
						DisplayImages(leftArrow);
					nxtDisplayBigStringAt(10, 35, "YELLOW");
					Drive(-speed/2, speed/2);
					break;
				case 3: //On Black
					DisplayImages(star);
					nxtDisplayBigStringAt(30, 35, "YAY!");
					Drive(0, 0);
					finished = true;
					break;
				case 4: //None of the above
					nxtDisplayBigStringAt(20,35, "ERROR");
					nxtDisplayTextLine(6, "Colour not");
					nxtDisplayTextLine(7, "recognized.");
					Drive(0,0);
					break;
			}
		wait10Msec(20); //delay 200 msecs before next loop repetition
	}
	Celebrate();
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
	else if(lightLevel >= BLACK_LOWER && lightLevel <= BLACK_HIGHER)
			r = 3;
	return r;
}

/*Method switches direction of robot it sound sensor is triggered past preset loudness variable*/
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
	//nxtDisplayTextLine(7, "Distance: %d", distance);
	float scale = 1;
	if (distance >= SLOW_DISTANCE)
		scale = 1.0;
  else if (distance <= STOP_DISTANCE)
  	scale = 0.0;
 	else
 		scale = ((float)distance/(SLOW_DISTANCE-STOP_DISTANCE)) - ((float)STOP_DISTANCE/(SLOW_DISTANCE-STOP_DISTANCE));
 	ultraLevel = scale;
}

void DisplayImages(Image image)
{
	int i, j;

	for(i = 0; i < 9; i++)
	{
		for(j = 0; j < 9; j++)
		{
			if(image[i][j] == 0)
				nxtClearPixel(50+j, 10-i);
			else if(image[i][j] == 1)
				nxtSetPixel(50+j, 10-i);

		}
	}
}

void Celebrate()
{
	int freq;
	float dur;
	int beats; //beats per minutes (one beat = one quater-note)
	int wholenote, quarternote;//duration of whole and quarter note in millisecond tics
	int tics; //duration in 10-millisecond tics needed by PlayTone( ) function

	beats = 250;//tempo in beats per minute
	quarternote = 1000*(60/(float)beats); //how many milliseconds
	wholenote = 4*quarternote; //how many milliseconds

	bPlaySounds = true;
	nVolume = 2;
	StartTask(motion);

	//turn off voltage regulation for motor[0] (headlight is not a motor!)
	nMotorPIDSpeedCtrl[0]=   mtrNoReg;

	noteindex=0;
	do
	{
		freq = notesF[noteindex];
		dur = notesD[noteindex];
		tics = (int)((wholenote*dur)/10);//how many 10 msec "tics"
		PlayTone(freq, tics);
		if(freq==0)
			motor[0] = 0;//headlight OFF for rest notes
		else
			motor[0] = 100;//headlight ON for all other notes
		wait10Msec(tics);
		noteindex = noteindex+1;
	} while (notesD[noteindex] != 0); //duration-0 note marks end of song

	StopTask(motion);
}

task motion()
{
	int oldindex=-1; //initialize so first one won't match

	//turn on speed regulation for accurate tracking
	//between left and right wheel rotations
	while(true)
	{
		if(noteindex!=oldindex)//has note changed in main()?
		{
			oldindex = noteindex;
			switch(noteindex)
			{
				case 0:
					CelebrateDrive(100,500);
					break;
				case 8:
					CelebrateDrive(-100,300);
					break;
				case 16:
					CelebrateRotate(-50,400);
					break;
				case 24:
					CelebrateRotate(50,200);
					break;
				case 32:
					CelebrateRotate(50,500);
					break;
				case 36:
					CelebrateRotate(-50,500);
					break;
				case 40:
					CelebrateRotate(-100,1000);
					break;
			}
		}
		else
			wait10Msec(5);
	}
}

void CelebrateRotate(int speed, int msecs)
{
	//set speeds
	motor[1] = +speed;
	motor[2] = -speed;
	//wait for needed rotation time
	wait1Msec(msecs);
	//set motors back to 0 speed
	motor[1] = 0;
	motor[2] = 0;
}

void CelebrateDrive(int speed, int msecs)
{
	//set speeds
	motor[1] = +speed;
	motor[2] = +speed;
	//wait for needed rotation time
	wait1Msec(msecs);
	//set motors back to 0 speed
	motor[1] = 0;
	motor[2] = 0;
}
