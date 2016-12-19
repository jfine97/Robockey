/*
 * Local_in_1.h
 *
 * Created: 11/19/2016 1:51:18 PM
 *  Author: Jake
 */ 

#ifndef Local_in_1__
#define Local_in_1__

volatile float HEADING;
volatile int X;
volatile int Y;
volatile int numstars;
volatile long OldTB;
volatile long OldLR;
volatile float OldHeading;
volatile int OldX;
volatile int OldY;
volatile int BotCenter[2];

int TOP;


void localize(unsigned int* mWiiData, int* BotCenter);

void starCases(int TB, int LR);
long diagDistance(int x1, int y1, int x2, int y2);


#endif