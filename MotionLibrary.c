/*
 * MotionLibrary.c
 *
 * Created: 11/22/2016 2:14:58 PM
 * Author : Owner
 */ 
#define F_CPU 16000000UL
#include <stdlib.h>
#include "m_general.h"
#include "m_bus.h"
#include "m_usb.h"
// #include "m_rf.h"
#include "MotionLibrary.h"
#define GAMMA 0.4

float VoltageScale;
float Heading[2];
int PosX[2];	 //Field-centric Coordinates
int PosY[2];
long BotX;       //Puck-centric Coordinates
long BotY;
// double BotX;
// double BotY;
int FieldCenter[2];
int PuckPosition[4];
int GoalPosition[2];
float DriveHeading;
float angleError[2];
float delError;
float LocalizationTime = 0.02;
float Kp = 1.5;
float Kd = 0.15;
double Vx[2];
double Vy[2];
int velocity;
int MA_POW;
int MB_POW;

int SinkNum = 0;

//ZEUS
int SinkPos[3][2] = { {275,100}, {-175,100}, {0,20} };
//POSEIDON

//HADES
// int SinkPos[3][2] = { {275,-100}, {-175,-100}, {0,-20} };

void GetDriveHeading(int state, int side)
{	
	//////////////////////////////////////// VELOCITY FIELDS ////////////////////////////////////////
	
	Vx[1] = Vx[0];
	Vy[1] = Vy[0];
	
	////////////////****************    GO TO PUCK    ****************////////////////
	if(state == 3)
	{
// 		PuckPosition[0] = -20; //Center?
// 		PuckPosition[1] = 50;
	
// 		PuckPosition[0] = 330;
// 		PuckPosition[1] = 75;

		////*** TRANSLATE INTO PUCK COORDINATES ****////
		BotX = (PosX[0] - PuckPosition[0]);
		BotY = (PosY[0] - PuckPosition[1]);
	
		FieldCenter[0] = 0 - PuckPosition[0];
		FieldCenter[1] = 0 - PuckPosition[1];
		
// 		BotX = BotX/10.0; //Vector field works best when numbers are in lower range
// 		BotY = BotY/10.0;
		
		if(side == 1)	//RIGHT SIDE (Goal on left)
		{
			Vx[0] = (-( (BotX*BotX) - (BotY*BotY) )/(sqrt( (BotX*BotX) + (BotY*BotY) )));
			Vy[0] = -2*(BotY*BotX)/(sqrt( (BotX*BotX) + (BotY*BotY) ));
			Vx[0] = Vx[0] - 0.5*BotX;
// 			Vx[0] = -BotX;
// 			Vy[0] = -BotY;
			if(BotX < 0)
			{
				Vx[0] = -BotX;
				Vy[0] = 0;
			}
		}else //LEFT SIDE (Goal on right)
		{
			Vx[0] = (( (BotX*BotX) - (BotY*BotY) )/(sqrt( (BotX*BotX) + (BotY*BotY) )));
			Vy[0] = 2*(BotY*BotX)/(sqrt( (BotX*BotX) + (BotY*BotY) ));
			Vx[0] = Vx[0] - 0.5*BotX;
// 			Vx[0] = -BotX;
// 			Vy[0] = -BotY;
			if(BotX > 0)
			{
				Vx[0] = -BotX;
				Vy[0] = 0;
			}
		}
	}
	
	
	////////////////****************    GO TO GOAL    ****************////////////////
	else if(state == 4)
	{

		if(side == 1)	//RIGHT SIDE (Goal on left)
		{
			GoalPosition[0] = -330;
			GoalPosition[1] = 25;
			
			BotX = PosX[0] - GoalPosition[0];
			BotY = PosY[0] - GoalPosition[1];
			Vx[0] = (-( (BotX*BotX) - (BotY*BotY) )/(sqrt( (BotX*BotX) + (BotY*BotY) ))) - 2*BotX;
			Vy[0] = -2*(BotY*BotX)/(sqrt( (BotX*BotX) + (BotY*BotY) )) - 3*BotY;

// 			Vx[0] = -BotX;
// 			Vy[0] = -BotY;
		}
		else //LEFT SIDE (Goal on right)
		{
			GoalPosition[0] = 330;
			GoalPosition[1] = 25;
			
			BotX = PosX[0] - GoalPosition[0];
			BotY = PosY[0] - GoalPosition[1];
			Vx[0] = (( (BotX*BotX) - (BotY*BotY) )/(sqrt( (BotX*BotX) + (BotY*BotY) ))) - 2*BotX;
			Vy[0] = 2*(BotY*BotX)/(sqrt( (BotX*BotX) + (BotY*BotY) )) - 3*BotY;
// 			Vx[0] = -BotX;
// 			Vy[0] = -BotY;
		}
	}
	
	////////////////****************    SEARCH FOR PUCK    ****************////////////////
	else if(state == 2)
	{
		
		////**** UPDATE SINK POSITION ****////
		long sinkDist = (BotX)*(BotX) + (BotY)*(BotY);
		if(sinkDist < 800)
		{
			//Rotate through the sink positions
			SinkNum++;
			if(SinkNum > 2)
			{	SinkNum = 0;	}
		}
		
		BotX = PosX[0] - SinkPos[SinkNum][0];
		BotY = PosY[0] - SinkPos[SinkNum][1];
		
		Vx[0] = -BotX;
		Vy[0] = -BotY;
	}

	//////////////////////////////////////// HEADING CALCULATIONS ////////////////////////////////////////
	Vx[0] = GAMMA*Vx[0] + (1-GAMMA)*Vx[1];
	Vy[0] = GAMMA*Vy[0] + (1-GAMMA)*Vy[1];
	
	////**** CALCULATE HEADING ****////
	
	DriveHeading = atan2(Vy[0],Vx[0])*180/M_PI;
	if(DriveHeading >= 0 && DriveHeading <= 90)			//Quadrant I
	{	DriveHeading = 270 + DriveHeading;	}
	else if(DriveHeading > 90 && DriveHeading <= 180)	//Quadrant II
	{	DriveHeading = DriveHeading -90;	}
	else if(DriveHeading <= -90 && DriveHeading >= -180)//Quadrant III
	{	DriveHeading = 270 + DriveHeading;	}
	else if(DriveHeading < 0 && DriveHeading > -90)		//Quadrant IV
	{	DriveHeading = 270 + DriveHeading;	}
		
	angleError[1] = angleError[0];
	angleError[0] = Heading[0] - DriveHeading;
	delError = (angleError[1] - angleError[0])/LocalizationTime;
	
	////**** CORRECT FOR WRAP AROUND ****////
	if(angleError[0] > 180){
		angleError[0] = angleError[0] - 360;
	}else if(angleError[0] < -180) {
		angleError[0] = angleError[0] + 360;
	}
	
	if((BotX*BotX + BotY*BotY) <= 400)
	{	velocity = 0;	}
	else
	{	velocity = 0;	}
	
	
	
	//////////////////////////////////////// DEBUGGING PRINTING ////////////////////////////////////////
	
// 	m_usb_tx_int(Heading[0]);
// 	m_usb_tx_string("    ");
// 	m_usb_tx_int(DriveHeading);
// 	m_usb_tx_string("    ");
// 	m_usb_tx_int(angleError[0]);
// 	m_usb_tx_string("\n");
	
// 	m_usb_tx_int(DriveHeading);
// 	m_usb_tx_string("    ");
// 	m_usb_tx_int(Kp*10);
// 	m_usb_tx_string("    ");
// 	m_usb_tx_int(Kd*10);
// 	m_usb_tx_string("\n");

// 	m_usb_tx_int(Heading[0]);
// 	m_usb_tx_string("    ");	
// 	m_usb_tx_int(DriveHeading);
// 	m_usb_tx_string("       ");
// 	m_usb_tx_int(PosX[0]);
// 	m_usb_tx_string("    ");
// 	m_usb_tx_int(BotX);
// 	m_usb_tx_string("       ");
// 	m_usb_tx_int(PosY[0]);
// 	m_usb_tx_string("    ");
// 	m_usb_tx_int(BotY);
// 	m_usb_tx_string("   ");
// 	m_usb_tx_string("\n");
	
	
	
	////**** ADJUST SPEED ****////
// 	if(angleError[0] < 135){
// 		velocity = (-.0055)*(angleError[0]*angleError[0]) + 100;  //Quadratic decrease as angle increases 
// 	}else {
// 		velocity = 0;  //Stop to turn around quickly
// 	}
}

void SetDriveHeading(float heading)
{
	DriveHeading = heading;
	
	angleError[1] = angleError[0];
	angleError[0] = Heading[0] - DriveHeading;
	
	////**** CORRECT FOR WRAP AROUND ****////
	if(angleError[0] > 180){
		angleError[0] = angleError[0] - 360;
	}else if(angleError[0] < -180) {
		angleError[0] = angleError[0] + 360;
	}
	
	delError = (angleError[1] - angleError[0])/LocalizationTime;
	
		m_usb_tx_int(Heading[0]);
		m_usb_tx_string("    ");
		m_usb_tx_int(DriveHeading);
		m_usb_tx_string("       ");
		m_usb_tx_int(PosX[0]);
		m_usb_tx_string("    ");
		m_usb_tx_int(PosY[0]);
		m_usb_tx_string("    ");
		m_usb_tx_string("\n");
		
	// m_red(TOGGLE);
// 	if(angleError[0] < 135){
// 		velocity = (-.0055)*(angleError[0]*angleError[0]) + 100;  //Quadratic decrease as angle increases
// 	}else {
// 		velocity = 0;  //Stop to turn around quickly
// 	}
}

void Drive(int Speed, int state, int turbo)
{
	int diff = 0;
	
	if(state == 4)  //Go To Goal
	{
		diff = (Kp*angleError[0] - Kd*delError);
		MA_POW = Speed + diff;
		MB_POW = Speed - diff;
	}
	else
	{
		diff = (Kp*angleError[0] - Kd*delError);
		
// 		if((BotX*BotX + BotY*BotY) <= 1000)
// 		{	diff = 0;	}	
		MA_POW = Speed + diff;
		MB_POW = Speed - diff;
	}
	
	
// 	m_usb_tx_int(diff);
// 	m_usb_tx_string("    ");
// 	m_usb_tx_int(MA_POW);
// 	m_usb_tx_string("    ");
// 	m_usb_tx_int(MB_POW);
// 	m_usb_tx_string("\n");

	if(state == 5){
	DriveMotorA(MA_POW, turbo);
	DriveMotorB(MB_POW, turbo);
	}else
	{
		DriveMotorA(0, turbo);
		DriveMotorB(0, turbo);
	}
	
// 	m_usb_tx_int(MA_POW);
// 	m_usb_tx_string("    ");
// 	m_usb_tx_int(MB_POW);
// 	m_usb_tx_string("    ");
// 	m_usb_tx_int(state);
// 	m_usb_tx_string("    ");
	
	// m_green(TOGGLE);
}

void DriveMotorA(int POWER, int turbo)  //Pin B1 => DIR   Pin B5 => PWM	Left Motor
{
	
	if(POWER >= 0)
	{ 	clear(PORTB,1); 	}
	else
	{	set(PORTB,1);	}
	
	if(POWER > 100)
	{	POWER = 100;	}
	else if(POWER < -100)
	{	POWER = -100;	}
	
	if(turbo == 0)
	{
		OCR1A = (abs(POWER)/100.0)*VoltageScale*1023; //scale the voltage to 12V
	} else {
		OCR1A = (abs(POWER)/100.0)*1023;
	}
	
// 	m_usb_tx_int(OCR1A);
// 	m_usb_tx_string("\n");
}

void DriveMotorB(int POWER, int turbo)  //Pin B2 => DIR   Pin B6 => PWM	Right Motor
{
	if(POWER >= 0)
	{	clear(PORTB,2);	}
	else
	{	set(PORTB,2);	}
		
	if(POWER > 100)
	{	POWER = 100;	}
	else if(POWER < -100)
	{	POWER = -100;	}
		
	if(turbo == 0)
	{
		OCR1B = (abs(POWER)/100.0)*VoltageScale*1023; //scale the voltage to 12V
	} else {
		OCR1B = (abs(POWER)/100.0)*1023;
	}
}