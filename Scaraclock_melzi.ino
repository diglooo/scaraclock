/*
	Name:       scaraclock.ino
	Created:  	20/03/2020 09:45:25
	Author:     Davide Di Gloria
              davidedigloria87@gmail.com
*/

#include <DS3231.h>
#include <MultiStepper.h>
#include <AccelStepper.h>
#include <Wire.h>
#include "font.h"


//MELZI board features, copied from Marlin firmware board header.
#define X_STOP_PIN         18
#define X_STEP_PIN         15
#define X_DIR_PIN          21
#define X_ENABLE_PIN       14
#define Y_STOP_PIN         19
#define Y_STEP_PIN         22
#define Y_DIR_PIN          23
#define Y_ENABLE_PIN       14
#define Z_STOP_PIN         20
#define Z_STEP_PIN          3
#define Z_DIR_PIN           2
#define Z_ENABLE_PIN       26
#define E0_ENABLE_PIN      14
#define E0_STEP_PIN         1
#define E0_DIR_PIN          0
#define TEMP_0_PIN          7   // Analog Input (pin 33 extruder)
#define TEMP_BED_PIN        6   // Analog Input (pin 34 bed)
#define HEATER_0_PIN       13   // (extruder)
#define LED_PIN          2
#define HEATER_BED_PIN   12   // (bed)


//Roboclock functions linked to the MELZI pins
#define ALPHA_HOME_PIN Y_STOP_PIN
#define ALPHA_STEP_PIN Y_STEP_PIN
#define ALPHA_DIR_PIN Y_DIR_PIN
#define ALPHA_MOT_ENA_PIN Y_ENABLE_PIN
#define BETA_HOME_PIN Z_STOP_PIN
#define BETA_STEP_PIN Z_STEP_PIN
#define BETA_DIR_PIN Z_DIR_PIN
#define BETA_MOT_ENA_PIN Z_ENABLE_PIN
#define SCREEN_STEP_PIN X_STEP_PIN
#define SCREEN_DIR_PIN X_DIR_PIN
#define SCREEN_MOT_ENA_PIN X_ENABLE_PIN
#define PEN_DOWN_PIN HEATER_0_PIN

//Motor speed in steps/seconds
#define MAX_SPEED 2000.0
#define MIN_SPEED 100.0
#define MAX_ACCEL 10000

//motion profile parameters

//Linear motions between two cartesian points is interpolated by
//splitting the segment into a fixed amount of small non-linear motions
#define LIN_IPO_STEPS 100

//how many IPO steps are dedicated to the accel/decel ramps
#define ACC_RAMP_LENGTH_STEPS 10

//calculate the speed increment for each step
#define VEL_INC_PER_STEP MAX_SPEED/ACC_RAMP_LENGTH_STEPS

//mechanical config
//arm lenght in millimeters
#define L1 100.0 
#define L2 150.0

//workspace limits
#define X_MAX 70
#define X_MIN -70
#define Y_MAX 185
#define Y_MIN 100

//drawing board size
#define FRAME_MAX_X 110
#define FRAME_MAX_Y 80

//arm motor config
#define STEPS_PER_REV 12800L
#define SCREEN_MOT_STEPS_PER_REV 8192L
#define DEG_PER_STEP (float)(360.0/STEPS_PER_REV)
#define ANG_TO_STEPS(a) ((long)((a) / DEG_PER_STEP))

//arm cartesian actual position
float cart_pos_act[2];
//arm actual position in joint angles
float joint_pos_act[2];

//array containing the interpolation steps
//to make a linear movement
float ipo_steps[2][LIN_IPO_STEPS];

AccelStepper betaMotor(AccelStepper::FULL2WIRE, BETA_STEP_PIN, BETA_DIR_PIN);
AccelStepper alphaMotor(AccelStepper::FULL2WIRE, ALPHA_STEP_PIN, ALPHA_DIR_PIN);
AccelStepper screenMotor(AccelStepper::FULL2WIRE, SCREEN_STEP_PIN, SCREEN_DIR_PIN);

//MultiStepper allows to synchronize many stepper motors 
//in such a way the motor motion is started and finished in the same moment
MultiStepper multiStepperOject;

DS3231 RTC;

void printActPos()
{
	Serial.print("(A,B,x,y)=(");
	Serial.print(joint_pos_act[0], 4);
	Serial.print(" ,");
	Serial.print(joint_pos_act[1], 4);
	Serial.print(" ,");
	Serial.print(cart_pos_act[0], 4);
	Serial.print(" ,");
	Serial.print(cart_pos_act[1], 4);
	Serial.print(")");
	Serial.print(" (alpha_mot, beta_mot)=(");
	Serial.print(alphaMotor.currentPosition());
	Serial.print(" ,");
	Serial.print(betaMotor.currentPosition());
	Serial.println(") ");
}

//inverse kinematics of the arm X,Y -> A,B
//computes the arm joint angles A,B to put the end effector in the desired X,Y position
bool inverseKinematics(float x, float y, float *alpha, float *beta)
{
	float acos_arg = (x*x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
 
	if (acos_arg < -1 || acos_arg > 1) return false;
	if (x == 0 && y == 0) return false;

	//L2-L1 arm angle
	*alpha = acos(acos_arg);

	//L1 arm angle
	*beta = atan2(y, x) - atan2((L2*sin(*alpha)), (L1 + L2 * cos(*alpha)));

	*alpha = *alpha * 57.2958;
	*beta = *beta  * 57.2958;

	return true;
}

//forward kinematics of the arm A,B -> X,Y
//computes the X,Y end effector position from the A,B joint angles.
bool forwardKinematics(float *x, float *y, float alpha, float beta)
{
   alpha = alpha / 57.2958; 
   beta = beta  / 57.2958; 
   *x= L1 * cos(beta) + L2 * cos(alpha + beta);
   *y= L1 * sin(beta) + L2 * sin(alpha + beta);
   return true;
}

//Connects the actual cartesian position with the destination position with a segment
//divide the segment into LIN_IPO_STEPS small segments
int ipo_lin_pos(float startx, float starty, float endx, float endy)
{
	float dx = endx - startx;
	float dy = endy - starty;
	int numPoints = LIN_IPO_STEPS;
	float stepx = dx / numPoints;
	float stepy = dy / numPoints;

	for (int i = 0; i < numPoints - 1; i++)
	{
		ipo_steps[0][i] = startx + i * stepx; //X
		ipo_steps[1][i] = starty + i * stepy; //Y
	}
	ipo_steps[0][0] = startx;
	ipo_steps[1][0] = starty;
	ipo_steps[0][numPoints-1] = endx;
	ipo_steps[1][numPoints-1] = endy;

	if (numPoints > 0)	return numPoints;
	else return -1;
}

//Command a motion in the joint space towards a point in the joint space
bool moveJ_joint(float alpha, float beta)
{
	long positions[2];
	positions[0] = ANG_TO_STEPS(alpha + beta);
	positions[1] = ANG_TO_STEPS(beta); 
	multiStepperOject.moveTo(positions);
	multiStepperOject.runSpeedToPosition();
}

//Command a motion in the joint space towards a point in the cartesian space
bool moveJ_cartesian(float x, float y)
{
	float a, b;
	//check workspace
	if(x<X_MIN || x>X_MAX || y<Y_MIN || y>Y_MAX) Serial.println("Out of area!");
	else
	{
		//inverse kinematic calculation
		if (inverseKinematics(x, y, &a, &b))
		{
			moveJ_joint(a, b);
			joint_pos_act[0] = a;
			joint_pos_act[1] = b;
			cart_pos_act[0] = x;
			cart_pos_act[1] = y;
		}
		else Serial.println("inverseKinematics error!");    
	}
}

//Make a linear movement towards a position in the cartesian space
bool moveL(float dest_pos_x, float dest_pos_y)
{
	float curr_speed = 0;

	//do the segment thing
	int numsteps = ipo_lin_pos(cart_pos_act[0], cart_pos_act[1], dest_pos_x, dest_pos_y);
	if (!numsteps) return false;

	for (int i = 0; i < numsteps - 1; i++)
	{
		//acceleration ramp
		if (i < min(ACC_RAMP_LENGTH_STEPS, numsteps / 2))
			curr_speed = min((i + 1) * VEL_INC_PER_STEP + MIN_SPEED, MAX_SPEED);

		//deceleration ramp
		if (i > max(numsteps - ACC_RAMP_LENGTH_STEPS, numsteps / 2))
			curr_speed = min((numsteps - i + 1) * VEL_INC_PER_STEP + MIN_SPEED, MAX_SPEED);
		  
		alphaMotor.setMaxSpeed(curr_speed);
		betaMotor.setMaxSpeed(curr_speed);
		moveJ_cartesian(ipo_steps[0][i], ipo_steps[1][i]);
	}
}

void draw_char(char ch, float x_scaling, float y_scaling, float x_offs, float y_offs)
{
  pinMode(PEN_DOWN_PIN,OUTPUT);
  
  switch (ch)
  {
  case '0':
	  for (int i = 0; i < 21; i++)
	  {
		  moveL(char_0[i].X*x_scaling + x_offs, char_0[i].Y*y_scaling + y_offs);
		  digitalWrite(PEN_DOWN_PIN, HIGH);
	  }
	  break;
  case '1':
	  for (int i = 0; i < 5; i++)
	  {
		  moveL(char_1[i].X*x_scaling + x_offs, char_1[i].Y*y_scaling + y_offs);
		  digitalWrite(PEN_DOWN_PIN, HIGH);
	  }
	  break;
  case '2':
	  for (int i = 0; i < 10; i++)
	  {
		  moveL(char_2[i].X*x_scaling + x_offs, char_2[i].Y*y_scaling + y_offs);
		  digitalWrite(PEN_DOWN_PIN, HIGH);
	  }
	  break;
  case '3':
	  for (int i = 0; i < 19; i++)
	  {
		  moveL(char_3[i].X*x_scaling + x_offs, char_3[i].Y*y_scaling + y_offs);
		  digitalWrite(PEN_DOWN_PIN, HIGH);
	  }
	  break;
  case '4':
	  for (int i = 0; i < 6; i++)
	  {
		  moveL(char_4[i].X*x_scaling + x_offs, char_4[i].Y*y_scaling + y_offs);
		  digitalWrite(PEN_DOWN_PIN, HIGH);
	  }
	  break;
  case '5':
	  for (int i = 0; i < 13; i++)
	  {
		  moveL(char_5[i].X*x_scaling + x_offs, char_5[i].Y*y_scaling + y_offs);
		  digitalWrite(PEN_DOWN_PIN, HIGH);
	  }
	  break;
  case '6':
	  for (int i = 0; i < 20; i++)
	  {
		  moveL(char_6[i].X*x_scaling + x_offs, char_6[i].Y*y_scaling + y_offs);
		  digitalWrite(PEN_DOWN_PIN, HIGH);
	  }
	  break;
  case '7':
	  for (int i = 0; i < 4; i++)
	  {
		  moveL(char_7[i].X*x_scaling + x_offs, char_7[i].Y*y_scaling + y_offs);
		  digitalWrite(PEN_DOWN_PIN, HIGH);
	  }
	  break;
  case '8':
	  for (int i = 0; i < 24; i++)
	  {
		  moveL(char_8[i].X*x_scaling + x_offs, char_8[i].Y*y_scaling + y_offs);
		  digitalWrite(PEN_DOWN_PIN, HIGH);
	  }
	  break;
  case '9':
	  for (int i = 0; i < 21; i++)
	  {
		  moveL(char_9[i].X*x_scaling + x_offs, char_9[i].Y*y_scaling + y_offs);
		  digitalWrite(PEN_DOWN_PIN, HIGH);
	  }
	  break;
  case ':':
	  moveL(char_col[0].X*x_scaling + x_offs, char_col[0].Y*y_scaling + y_offs);
	  digitalWrite(PEN_DOWN_PIN, HIGH);
	  moveL(char_col[1].X*x_scaling + x_offs, char_col[1].Y*y_scaling + y_offs);
	  digitalWrite(PEN_DOWN_PIN, LOW);

	  moveL(char_col[2].X*x_scaling + x_offs, char_col[2].Y*y_scaling + y_offs);
	  digitalWrite(PEN_DOWN_PIN, HIGH);
	  moveL(char_col[3].X*x_scaling + x_offs, char_col[3].Y*y_scaling + y_offs);
	  digitalWrite(PEN_DOWN_PIN, LOW);
	  break;
  case '/':
	  moveL(char_sl[0].X*x_scaling + x_offs, char_sl[0].Y*y_scaling + y_offs);
	  digitalWrite(PEN_DOWN_PIN, HIGH);
	  moveL(char_sl[1].X*x_scaling + x_offs, char_sl[1].Y*y_scaling + y_offs);
	  digitalWrite(PEN_DOWN_PIN, LOW);

  case ' ': break;
  }
	digitalWrite(PEN_DOWN_PIN,LOW);
}

void draw_string(char* str, float x_orig, float y_orig, float x_scaling, float y_scaling, float spacing)
{
	for (int c = 0; c < 5; c++)
	{
		draw_char(str[c], x_scaling, y_scaling, spacing*c + x_orig , y_orig);
		delay(100);
	}
}

//interrupt handling for the two limit switches
volatile bool alpha_homed, alpha_homing_on = false;
volatile bool beta_homed, beta_homing_on = false;
ISR (PCINT2_vect)
{
  if(!digitalRead(BETA_HOME_PIN) && beta_homing_on)
  {
    pciClear();
    beta_homed = true;
    beta_homing_on=false;
    betaMotor.stop();
  }

  if(!digitalRead(ALPHA_HOME_PIN)&& alpha_homing_on)
  {
	pciClear();
    alpha_homed = true;
    alphaMotor.stop();
  }
} 

//Utility to set the time on the DS3121
//Copied from one sample sketch
byte Year, Month, Date, DoW, Hour, Minute, Second;
void GetDateStuff(byte& Year, byte& Month, byte& Day, byte& DoW, byte& Hour, byte& Minute, byte& Second) {
	// Call this if you notice something coming in on 
	// the serial port. The stuff coming in should be in 
	// the order YYMMDDwHHMMSS, with an 'x' at the end.
	boolean GotString = false;
	char InChar;
	byte Temp1, Temp2;
	char InString[20];

	byte j = 0;
	while (!GotString) {
		if (Serial.available()) {
			InChar = Serial.read();
			InString[j] = InChar;
			j += 1;
			if (InChar == 'x') {
				GotString = true;
			}
		}
	}
	Serial.println(InString);
	// Read Year first
	Temp1 = (byte)InString[0] - 48;
	Temp2 = (byte)InString[1] - 48;
	Year = Temp1 * 10 + Temp2;
	// now month
	Temp1 = (byte)InString[2] - 48;
	Temp2 = (byte)InString[3] - 48;
	Month = Temp1 * 10 + Temp2;
	// now date
	Temp1 = (byte)InString[4] - 48;
	Temp2 = (byte)InString[5] - 48;
	Day = Temp1 * 10 + Temp2;
	// now Day of Week
	DoW = (byte)InString[6] - 48;
	// now Hour
	Temp1 = (byte)InString[7] - 48;
	Temp2 = (byte)InString[8] - 48;
	Hour = Temp1 * 10 + Temp2;
	// now Minute
	Temp1 = (byte)InString[9] - 48;
	Temp2 = (byte)InString[10] - 48;
	Minute = Temp1 * 10 + Temp2;
	// now Second
	Temp1 = (byte)InString[11] - 48;
	Temp2 = (byte)InString[12] - 48;
	Second = Temp1 * 10 + Temp2;
}

//Set the port change interrupt
void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

//clear the port change interrupt
void pciClear()
{
  PCIFR=0;
  PCICR=0;
}

void setup()
{
	//disable the motors
	pinMode(ALPHA_MOT_ENA_PIN, OUTPUT);
	pinMode(BETA_MOT_ENA_PIN, OUTPUT);
	digitalWrite(ALPHA_MOT_ENA_PIN, HIGH);
	digitalWrite(BETA_MOT_ENA_PIN, HIGH);

	//alpha motors nees to be reversed
	alphaMotor.setPinsInverted((bool)true,(bool)false,(bool)false);

	Wire.begin();
	Serial.begin(115200);
	delay(1000); 

	Serial.print(F("set time inputting YYMMDDWHHMMSS, with an 'x' at the end "));
	int i = 5;
	while (!Serial.available() && i > 0)
	{
		Serial.print(i--); Serial.print(" ");
		delay(100);
	}
	if (Serial.available())
	{
		GetDateStuff(Year, Month, Date, DoW, Hour, Minute, Second);
		RTC.setClockMode(false);
		RTC.setYear(Year);
		RTC.setMonth(Month);
		RTC.setDate(Date);
		RTC.setDoW(DoW);
		RTC.setHour(Hour);
		RTC.setMinute(Minute);
		RTC.setSecond(Second);
		Serial.println(F("time set"));
	}
	Serial.println(F("normal startup"));

	//start homing sequence
	pinMode(ALPHA_HOME_PIN, INPUT_PULLUP);
	pinMode(BETA_HOME_PIN, INPUT_PULLUP);
	delay(100);
	alphaMotor.setMaxSpeed(MAX_SPEED/5);
	betaMotor.setMaxSpeed(MAX_SPEED/5);
	alphaMotor.setAcceleration(MAX_ACCEL);
	betaMotor.setAcceleration(MAX_ACCEL);

	//enable both motors
	digitalWrite(ALPHA_MOT_ENA_PIN, LOW); 
	digitalWrite(BETA_MOT_ENA_PIN, LOW);
  
	alpha_homing_on=true;
	pciSetup(ALPHA_HOME_PIN);  
	alphaMotor.runToNewPosition(ANG_TO_STEPS(-50)); 

	float home_a=143.8;
	alphaMotor.setCurrentPosition(ANG_TO_STEPS(home_a));
	alphaMotor.runToNewPosition(ANG_TO_STEPS(home_a-10));  
	alphaMotor.setCurrentPosition(ANG_TO_STEPS(home_a-10));
  
	beta_homing_on=true;
	pciSetup(BETA_HOME_PIN); 
	betaMotor.setCurrentPosition(ANG_TO_STEPS(0));
	betaMotor.runToNewPosition(ANG_TO_STEPS(-90));  
	betaMotor.setCurrentPosition(ANG_TO_STEPS(-6));
	betaMotor.runToNewPosition(ANG_TO_STEPS(0)); 
	betaMotor.setCurrentPosition(ANG_TO_STEPS(0));

	joint_pos_act[0]=home_a-10;
	joint_pos_act[1]=0; 

	forwardKinematics(&cart_pos_act[0], &cart_pos_act[1], joint_pos_act[0], joint_pos_act[1]);
	printActPos();

	//set the execution speed and couple the two motors
	alphaMotor.setMaxSpeed(MAX_SPEED);
	betaMotor.setMaxSpeed(MAX_SPEED);
	multiStepperOject.addStepper(alphaMotor);
	multiStepperOject.addStepper(betaMotor);
  
	delay(500);
}

void coordFrame2World(float x_in, float y_in, float *x_out, float *y_out)
{
   *x_out = x_in-60;
   *y_out = y_in+100;
}

uint8_t cleanerPosLeft=0;
void clearScreen()
{
  screenMotor.setMaxSpeed(10000);
  screenMotor.setAcceleration(MAX_ACCEL);
  
  if(cleanerPosLeft)
  {
    //move right;
    screenMotor.runToNewPosition((long)(SCREEN_MOT_STEPS_PER_REV*5));
    cleanerPosLeft=0;    
  }
  else
  {
   screenMotor.runToNewPosition(0);
   cleanerPosLeft=1;
  }
}

float x_pos,y_pos;
byte lastMinute=100;
void loop()
{  

  /*
  while(1)
  { 
    if (Serial.available())
    {
      float x_d=cart_pos_act[0];
      float y_d=cart_pos_act[1];
      float a_d=joint_pos_act[0];
      float b_d=joint_pos_act[1];
      char c = Serial.read();
      switch(c)
      {
        case 'w': y_d+=5; moveL(x_d, y_d); break;
        case 's': y_d-=5; moveL(x_d, y_d); break;
        case 'a': x_d-=5; moveL(x_d, y_d); break;
        case 'd': x_d+=5; moveL(x_d, y_d); break;    
        case 'r': a_d+=1; move_jpos(a_d, b_d); break;
        case 'f': a_d-=1; move_jpos(a_d, b_d); break;
        case 't': b_d-=1; move_jpos(a_d, b_d); break;
        case 'g': b_d+=1; move_jpos(a_d, b_d); break;    
      }
              
      Serial.print("POS(A,B,x,y)=("); 
      Serial.print(joint_pos_act[0],4);
      Serial.print(" ,"); 
      Serial.print(joint_pos_act[1],4);
      Serial.print(" ,"); 
      Serial.print(cart_pos_act[0],4);
      Serial.print(" ,"); 
      Serial.print(cart_pos_act[1],4);
      Serial.println(")"); 
    }
  }


	coordFrame2World(FRAME_MAX_X, FRAME_MAX_Y, &x_pos, &y_pos);
	moveL(x_pos, y_pos);
	printActPos();

	coordFrame2World(0, FRAME_MAX_Y, &x_pos, &y_pos);
	moveL(x_pos, y_pos);
	printActPos();
	*/

    if(lastMinute != RTC.getMinute())
    {     
		coordFrame2World(-4,0,&x_pos,&y_pos);
		moveL(x_pos, y_pos); 
		printActPos();
    
		char strbuf[6];
		bool foo;
		bool h12, PM;
  
		Hour = RTC.getHour(h12, PM);
		Minute = RTC.getMinute();
		lastMinute=Minute;
		Date= RTC.getDate();
		Month= RTC.getMonth(foo);  
      
		clearScreen();
  
		sprintf(strbuf, "%02d:%02d", Hour, Minute);
		draw_string(strbuf, -65, 135,0.41, 0.55, 22);
  
		sprintf(strbuf, "%02d/%02d", Date, Month);
		draw_string(strbuf, -60, 107, 0.21, 0.3, 15);
     
		coordFrame2World(-4,0,&x_pos,&y_pos);
		moveL(x_pos, y_pos);
		printActPos();
    }     
    delay(2000);

}
