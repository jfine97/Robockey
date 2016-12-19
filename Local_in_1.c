/*
 * Local_in_1.c
 *
 * Created: 11/22/2016 2:27:13 PM
 *  Author: Jake Fine
 */ 

#define F_CPU 16000000UL
#include <math.h>
#include <stdlib.h>
#include "Local_in_1.h"
#include "m_usb.h"
#include "m_general.h"

int TOP;
int BOT;
int LF;
int RT;
int TB;
int LR;

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

 
void localize(unsigned int* mWiiData, int* BotCenter)
{
	TOP = 0;
	BOT = 0;
	LF  = 0;
	RT  = 0;
	
	////**** How Many Stars ****////
		int ctr = 0;
		int ii;
			
		for(ii=0 ; ii<12 ; ii++)
		{
			if(mWiiData[ii] == 1023)
			{
				ctr++;
			}
		}
			
		ctr = ctr*0.5;
			
		if (ctr >= 1)
		{
			numstars = 4 - ctr;
		}
		else
		{
			numstars = 4;
		}
	
	long DistArray[6]; long MAX; long MIN;
	int TB = 0; int LR = 0;
	int CASE;
	int TBVect[4]; 
	int LRVect[4];
	int mWiiDatanew[8];
	int threeStars[6];
	long distances[3][3];
	long distancesSort[3];
	long minDistance; long maxDistance; long midDistance;
	double ratio1; double ratio2; double ratioProd;
	long compA; long compB;
	int maxStar1; long maxStar1_dis;
	int maxStar2; long maxStar2_dis;
	int thirdstar;
	int LEFT; int RIGHT; int BOTTOM;
	float theta;
	
	
	
	
	switch (numstars)
	{
	////**** FOUR STARS SEEN ****////
		case 4:
			//Solve 4 Stars
	
			DistArray[0] =(long)(( mWiiData[3]  - mWiiData[0] )*( mWiiData[3]  - mWiiData[0] ) + ( mWiiData[4]  - mWiiData[1] )*( mWiiData[4]  - mWiiData[1] ));    //Dist 1-2
			DistArray[1] =(long) (( mWiiData[6]  - mWiiData[0] )*( mWiiData[6]  - mWiiData[0] ) + ( mWiiData[7]  - mWiiData[1] )*( mWiiData[7]  - mWiiData[1] ));    //Dist 1-3
			DistArray[2] =(long) (( mWiiData[9]  - mWiiData[0] )*( mWiiData[9]  - mWiiData[0] ) + ( mWiiData[10] - mWiiData[1] )*( mWiiData[10] - mWiiData[1] ));    //Dist 1-4
			DistArray[3] =(long) (( mWiiData[6]  - mWiiData[3] )*( mWiiData[6]  - mWiiData[3] ) + ( mWiiData[7]  - mWiiData[4] )*( mWiiData[7]  - mWiiData[4] ));    //Dist 2-3
			DistArray[4] =(long) (( mWiiData[9]  - mWiiData[3] )*( mWiiData[9]  - mWiiData[3] ) + ( mWiiData[10] - mWiiData[4] )*( mWiiData[10] - mWiiData[4] ));    //Dist 2-4
			DistArray[5] =(long) (( mWiiData[9]  - mWiiData[6] )*( mWiiData[9]  - mWiiData[6] ) + ( mWiiData[10] - mWiiData[7] )*( mWiiData[10] - mWiiData[7] ));    //Dist 3-4
			
			MAX = 0;
			MIN = DistArray[0];
			
			for (int ii=0 ; ii<6 ; ii++ )
			{
				if (DistArray[ii] > MAX)
				{	MAX = DistArray[ii];	}
				if (DistArray[ii] < MIN)
				{	MIN = DistArray[ii];	}
			}
			
			
			
			for (int n=0 ; n<6 ; n++)
			{
				if (DistArray[n] == MAX)
				{
					TB = n + 1;
					break;
				}
			}
			for (int j = 0 ; j<6 ; j++)
			{
				if (DistArray[j] == MIN)
				{
					LR = j + 1;
					break;
				}
			}
			
			starCases(TB,LR);  //stars = [TOP BOT LF RT]		
			
			if (TOP > 0)
			{
				CASE = 1;   //TB and LR Vector are defined
				TBVect[0] = mWiiData[BOT*3-3]; TBVect[1] = mWiiData[BOT*3-2];
				TBVect[2] = mWiiData[TOP*3-3]; TBVect[3] = mWiiData[TOP*3-2];
				LRVect[0] = mWiiData[LF*3-3]; LRVect[1] = mWiiData[LF*3-2];
				LRVect[2] = mWiiData[RT*3-3]; LRVect[3] = mWiiData[RT*3-2];
			}
			else
			{
				CASE = 3;
				TBVect[0] = 0; TBVect[1] = 0; TBVect[2] = 0; TBVect[3] = 0;
				LRVect[0] = 0; LRVect[1] = 0; LRVect[2] = 0; LRVect[3] = 0;
			}
			//
			OldTB = ((long) (TBVect[2] - TBVect[0]))*(TBVect[2] - TBVect[0]) + ((long)(TBVect[3] - TBVect[1]))*(TBVect[3] - TBVect[1]);
			OldLR = (long) (LRVect[2] - LRVect[0])*(LRVect[2] - LRVect[0]) + ((long)(LRVect[3] - LRVect[1])*(LRVect[3] - LRVect[1]));
			break;
			
			
			
	////**** THREE STARS SEEN ****////
		case 3: //only 3 stars present
			//Solve 3 Stars
			mWiiDatanew[0] = mWiiData[0];
			mWiiDatanew[1] = mWiiData[1];
			mWiiDatanew[2] = mWiiData[3];
			mWiiDatanew[3] = mWiiData[4];
			mWiiDatanew[4] = mWiiData[6];
			mWiiDatanew[5] = mWiiData[7];
			mWiiDatanew[6] = mWiiData[9];
			mWiiDatanew[7] = mWiiData[10];
			
			int b = 0;
			for(int i=0;i<8;i++)   //removes missing star values (1023's)
			{
				if (mWiiDatanew[i] != 1023)
				{
					threeStars[b] = mWiiDatanew[i];
					b++;
				}
			}
			
			//find diagonal distances			
			//1-2
			distances[0][0] = diagDistance(threeStars[0], threeStars[1], threeStars[2], threeStars[3]);
			distances[0][1] = 1; distances[0][2] = 2;
			//1-3
			distances[1][0] = diagDistance(threeStars[0], threeStars[1], threeStars[4], threeStars[5]);
			distances[1][1] = 1; distances[1][2] = 3;
			//2-3
			distances[2][0] = diagDistance(threeStars[2], threeStars[3], threeStars[4], threeStars[5]);
			distances[2][1] = 2; distances[2][2] = 3;
			
			//sort in ascending order from top to bottom
			
			float distancesToBeSorted[3][1];
			distancesToBeSorted[0][0] = distances[0][0];
			distancesToBeSorted[1][0] = distances[1][0];
			distancesToBeSorted[2][0] =  distances[2][0];
			int col = 0;
				for (int row = 1 ; row < 3 ; row++)
				{
					if (distancesToBeSorted[row][col] < distancesToBeSorted[row-1][col])
					{
						int temp = distancesToBeSorted[row][col];
						distancesToBeSorted[row][col] = distancesToBeSorted[row-1][col];
						distancesToBeSorted[row-1][col] = temp;
					}
				}
			
			distancesSort[0] = distancesToBeSorted[0][0];
			distancesSort[1] = distancesToBeSorted[1][0];
			distancesSort[2] = distancesToBeSorted[2][0];
			
			minDistance = distancesSort[0];
			maxDistance = distancesSort[2];
			midDistance = distancesSort[1];
			
			ratio1 = ((double) maxDistance)/minDistance;
			ratio2 = ((double) minDistance)/midDistance;
			ratioProd = ((double) ratio1) * ratio2 * 200;
			
			// determine Situation
			// situation 1 = TLR  ---> find B ---> and plot it
			// situation 2 = TRB
			// situation 3 = TLB
			// situation 4 = LRB  ---> find T ---> and plot it
			// situation 5 = Doesn't fit a category so i have no idea what to do
			
			#define del 75  // should be set to half the space between situation 1 and 4
			int sit;
			if (ratioProd >= (float) (943.72 - del) && ratioProd < (float) (943.72 + del))
			{	sit = 2;	}
			else if (ratioProd >= (292.76 - del) && ratioProd < (292.76 + del))
			{	sit = 4;	}
			else
			{
				compA = abs(OldTB - maxDistance);
				compB = abs(OldLR - maxDistance);
				if (compA >= compB)
				{	sit = 1;	}
				else
				{	sit = 3;	}
			}
			
			//find maxStar
			for (int n = 0 ; n < 3 ; n++)
			{
				if (distances[n][0] == maxDistance)
				{
					maxStar1 = distances[n][1];
					maxStar2 = distances[n][2];
				}
			}
			
			//find length from maxStar1 to 3rd star
			for (int p = 0 ; p < 3 ; p++)
			{
				if (distances[p][1] == maxStar1 || distances[p][2] == maxStar1)
				{
					if (distances[p][1] != maxStar2 && distances[p][2] != maxStar2)
					{
						maxStar1_dis = distances[p][0];
					}
				}
			}
			//find length from maxStar2 to 3rd star
			for (int q = 0 ; q < 3 ; q++)
			{
				if (distances[q][1] == maxStar2 || distances[q][2] == maxStar2)
				{
					if (distances[q][1] != maxStar1 && distances[q][2] != maxStar1)
					{
						maxStar2_dis = distances[q][0];
					}
				}
			}
			int A[3] = { 1 , 2 , 3 };
			for (int t = 0 ; t < 3 ; t++)
			{
				if (A[t] != maxStar1 && A[t] != maxStar2)
				{	thirdstar = A[t];	}
			}
			//determine which star are which
			if (sit == 1)
			{
				if (maxStar1_dis > maxStar2_dis)
				{	LEFT = maxStar1; RIGHT = maxStar2; TOP = thirdstar;	}
				else
				{	LEFT = maxStar2; RIGHT = maxStar1; TOP = thirdstar;	}
				CASE = 2;
			}
			else if (sit == 2)
			{
				if (maxStar1_dis > maxStar2_dis)
				{	BOTTOM = maxStar1; TOP = maxStar2; RIGHT = thirdstar;	}
				else
				{	BOTTOM = maxStar2; TOP = maxStar1; RIGHT = thirdstar;	}
				CASE = 1;
			}
			else if (sit == 3)
			{
				if (maxStar1_dis > maxStar2_dis)
				{	BOTTOM = maxStar1; TOP = maxStar2; LEFT = thirdstar;	}
				else
				{	BOTTOM = maxStar2; TOP = maxStar1; LEFT = thirdstar;	}
				CASE = 1;
			}
			else if (sit == 4)
			{
				if (maxStar1_dis > maxStar2_dis)
				{	RIGHT = maxStar1; BOTTOM = maxStar2; LEFT = thirdstar;	}
				else
				{	RIGHT = maxStar2; BOTTOM = maxStar1; LEFT = thirdstar;	}
				CASE = 2;
			}
			else if (sit == 5)
			{
				CASE = 3;
				TBVect[0] = 0;	TBVect[1] = 0;	TBVect[2] = 0;	TBVect[3] = 0;
				LRVect[0] = 0;	LRVect[1] = 0;	LRVect[2] = 0;	LRVect[3] = 0;
			}
			
			//Set the TB/LR Vectors to their appropriate value
			if (CASE == 1)
			{
				TBVect[0] = threeStars[2*BOTTOM - 2];	TBVect[1] = threeStars[2*BOTTOM - 1];
				TBVect[2] = threeStars[2*TOP-2];	TBVect[3] = threeStars[2*TOP - 1];
				LRVect[0] = 0;	LRVect[1] = 0;	LRVect[2] = 0;	LRVect[3] = 0;
				OldTB = (TBVect[2] - TBVect[0])*(TBVect[2] - TBVect[0])+ (TBVect[3] - TBVect[1])*(TBVect[3] - TBVect[1]);
			}
			else if (CASE == 2)
			{
				TBVect[0] = 0;	TBVect[1] = 0;	TBVect[2] = 0;	TBVect[3] = 0;
				LRVect[0] = threeStars[2*LEFT - 2];	LRVect[1] = threeStars[2*LEFT - 1];
				LRVect[2] = threeStars[2*RIGHT-2];	LRVect[3] = threeStars[2*RIGHT - 1];
				OldLR = (LRVect[2] - LRVect[0])*(LRVect[2] - LRVect[0])+ (LRVect[3] - LRVect[1])*(LRVect[3] - LRVect[1]); 
			}
			break;
		case 2: //2 stars
			CASE = 3;	break;
		case 1: //1 star
			CASE = 3;	break;
		case 0: //No stars
			CASE = 3;	break;
	}
	
	////**** Find Position From Stars ****////
	
	int x1; int y1; int x2; int y2;
	int botX;
	int botY;
	float R2D2 = 180/M_PI;
	int botPrime[2];
	int vectPrime[2];
	float center[2];
	float rotationMatrix[4];
	
	switch (CASE)
	{
		case 1: //TBVect is known
			//Get Position
			x1 = TBVect[0]; y1 = TBVect[1]; x2 = TBVect[2]; y2 = TBVect[3];
			
			x2 = x2 - x1;  //translate the star to (0,0) -- vector from BOT to TOP
			y2 = y2 - y1;
			
			botX = BotCenter[0] - x1;
			botY = BotCenter[1] - y1;

			theta = (float) atan2((double) y2, (double) x2)*R2D2;
			
			rotationMatrix[0] = cos(theta/180.0*M_PI);     //  [0 1
			rotationMatrix[1] = sin(theta/180.0*M_PI);	   //   2 3]
			rotationMatrix[2] = -sin(theta/180.0*M_PI);
			rotationMatrix[3] = cos(theta/180.0*M_PI);
			
// 			rotationMatrix[0] = -sin( (double) (theta/180.0)*M_PI);     //  [0 1 
// 			rotationMatrix[1] = cos( (double) (theta/180.0)*M_PI);	   //   2 3]
// 			rotationMatrix[2] = cos( (double) (theta/180.0)*M_PI);
// 			rotationMatrix[3] = sin( (double) (theta/180.0)*M_PI);
			
			//Find and Rotate Coordinates
			botPrime[0] = (botX*rotationMatrix[0]) + (botY*rotationMatrix[1]);
			botPrime[1] = (botX*rotationMatrix[2]) + (botY*rotationMatrix[3]);
			
// 			vectPrime[0] = (x2*rotationMatrix[0]) + (y2*rotationMatrix[1]);
// 			vectPrime[1] = (x2*rotationMatrix[2]) + (y2*rotationMatrix[3]);
			
			//Find new center
			center[0] = 0;
			center[1]  = y2/2.0;
			
			if (theta < 0)
			{	theta = 360 + theta;	}
			
			//Find coordinates
			HEADING = theta;
			Y = botPrime[0] - center[0];    // in field view x is y and y is x
			X = botPrime[1] - center[1];	// ^
// 			X = botPrime[0] - center[0];    // in field view x is y and y is x
// 			Y = botPrime[1] - center[1];	// ^
			OldX = X; OldY = Y; OldHeading = HEADING;

			//
			break;
		case 2: //LRVect is known
			//Get Position
			x1 = LRVect[0]; y1 = LRVect[1]; x2 = LRVect[2]; y2 = LRVect[3];
			x2 = x2 - x1;  //translate the star to (0,0)
			y2 = y2 - y1;
			
			botX = BotCenter[0] - x1;
			botY = BotCenter[1] - y1;

				theta = (float) atan2((double) y2, (double) x2)*R2D2;
				theta = (float) theta + 74.2694;

			if (theta < 0)
			{
				theta = 360 + theta;
			}
			
// 			rotationMatrix[0] = cos(theta/180.0*M_PI);     //  [0 1
// 			rotationMatrix[1] = sin(theta/180.0*M_PI);	   //   2 3]
// 			rotationMatrix[2] = -sin(theta/180.0*M_PI);
// 			rotationMatrix[3] = cos(theta/180.0*M_PI);

			rotationMatrix[0] = -sin(theta/180.0*M_PI);     //  [0 1
			rotationMatrix[1] = cos(theta/180.0*M_PI);	   //   2 3]
			rotationMatrix[2] = cos(theta/180.0*M_PI);
			rotationMatrix[3] = sin(theta/180.0*M_PI);
			
			//Find and Rotate Coordinates
			botPrime[0] = ((float) botX*rotationMatrix[0]) + (botY*rotationMatrix[1]);
			botPrime[1] = ((float) botX*rotationMatrix[2]) + (botY*rotationMatrix[3]);
			
			vectPrime[0] = (x2*rotationMatrix[0]) + (y2*rotationMatrix[1]);
			vectPrime[1] = (x2*rotationMatrix[2]) + (y2*rotationMatrix[3]);
			
			//Find new center
			center[0] = vectPrime[0]*0.4754321;
			center[1] = vectPrime[1]*(-0.3967855);
			
			//Find coordinates
			HEADING = theta;
			// 			Y = botPrime[0] - center[0];    // in field view x is y and y is x
			// 			X = botPrime[1] - center[1];	// ^
			X = botPrime[0] - center[0];    // in field view x is y and y is x
			Y = botPrime[1] - center[1];	// ^
			OldX = X; OldY = Y; OldHeading = HEADING;
			//HEADING = theta;
			break;
		case 3:
			X = OldX; Y = OldY; HEADING = OldHeading; 
			break;		
	}
	
	/// Printing 
// 	char rx_buffer = m_usb_rx_char();  			// Read the packet from the computer
// 
// 	m_usb_rx_flush();  						// Flush the buffer
// 	
// 	if(rx_buffer == 1)						// MATLAB is expecting IMU data
//  	{	//For PLOT THE STARS
// 		m_usb_tx_int(mWiiData[3*TOP-3]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(mWiiData[3*TOP-2]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(mWiiData[3*BOT-3]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(mWiiData[3*BOT-2]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(mWiiData[3*LF-3]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(mWiiData[3*LF-2]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(mWiiData[3*RT-3]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(mWiiData[3*RT-2]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(TB);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(LR);
// 		m_usb_tx_char('\t');



// 		m_usb_tx_int(mWiiData[0]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(mWiiData[1]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(mWiiData[3]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(mWiiData[4]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(mWiiData[6]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(mWiiData[7]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(mWiiData[9]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(mWiiData[10]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(TB);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(LR);
// 		m_usb_tx_char('\t');


//      For HEADING and POSITION
// 			m_usb_tx_int(HEADING);
// 			m_usb_tx_char('\t');
// 			m_usb_tx_int(X);
// 			m_usb_tx_char('\t');
// 			m_usb_tx_int(Y);
// 			m_usb_tx_char('\t');
// 			m_usb_tx_int(numstars);
// 			m_usb_tx_string("\n");
		
		
// 		m_usb_tx_string("BOT X: ");
// 		m_usb_tx_int(botX);
// 		m_usb_tx_string("  ");
// 		m_usb_tx_string("BOT Y: ");
// 		m_usb_tx_int(botY);
// 		m_usb_tx_string("  ");
// 		m_usb_tx_char('\n');
// 		m_usb_tx_string("ROT: ");
// 		m_usb_tx_int(100*rotationMatrix[0]); m_usb_tx_string("  "); m_usb_tx_int(100*rotationMatrix[1]);   
// 		m_usb_tx_string("  ");
// 		m_usb_tx_int(100*rotationMatrix[2]); m_usb_tx_string("  "); m_usb_tx_int(100*rotationMatrix[3]); 
// 		m_usb_tx_string("  ");
// 		m_usb_tx_string("BOT PRIME X: ");
// 		m_usb_tx_int(botPrime[0]);
// 		m_usb_tx_string("  ");
// 		m_usb_tx_string("BOT PRIME Y: ");
// 		m_usb_tx_int(botPrime[1]);
// 		m_usb_tx_string("  ");
// 		
// 		
// 		m_usb_tx_string("\n");					// MATLAB serial command reads 1 line at a time
// 		m_green(TOGGLE);
//  	}
// 
}


void starCases(int TB, int LR) 
{
   TOP = 0;
	switch (TB)
	{
		//// TOP/BOT = 1/2 ////
		case 1: 
			switch (LR)
			{
				case 2:	TOP = 1; BOT = 2; LF  = 4; RT  = 3;	break;
				case 3:	TOP = 1; BOT = 2; LF  = 3; RT  = 4;	break;
				case 4:	TOP = 2; BOT = 1; LF  = 4; RT  = 3;	break;
				case 5:	TOP = 2; BOT = 1; LF  = 3; RT  = 4;	break;
			}
		break;
		//// TOP/BOT = 1/3 ////
		case 2:  
			switch (LR)
			{
				case 1:	TOP = 1; BOT = 3; LF  = 4; RT  = 2;	break;
				case 3:	TOP = 1; BOT = 3; LF  = 2; RT  = 4;	break;
				case 4:	TOP = 3; BOT = 1; LF  = 4; RT  = 2;	break;
				case 6:	TOP = 3; BOT = 1; LF  = 2; RT  = 4;	break;
			}
		break;
		//// TOP/BOT = 1/4 ////
		case 3: 
			switch (LR)
			{
				case 1:	TOP = 1; BOT = 4; LF  = 3; RT  = 2;	break;
				case 2:	TOP = 1; BOT = 4; LF  = 2; RT  = 3;	break;
				case 5:	TOP = 4; BOT = 1; LF  = 3; RT  = 2;	break;
				case 6:	TOP = 4; BOT = 1; LF  = 2; RT  = 3;	break;
			}
		break;
		//// TOP/BOT = 2/3 ////
		case 4: 
			switch (LR)
			{
				case 1:	TOP = 2; BOT = 3; LF  = 4; RT  = 1;	break;
				case 2:	TOP = 3; BOT = 2; LF  = 4; RT  = 1;	break;
				case 5:	TOP = 2; BOT = 3; LF  = 1; RT  = 4;	break;
				case 6:	TOP = 3; BOT = 2; LF  = 1; RT  = 4;	break;
			}
		break;
		//// TOP/BOT = 2/4 ////
		case 5: 
			switch (LR)
			{
				case 1:	TOP = 2; BOT = 4; LF  = 3; RT  = 1;	break;
				case 3:	TOP = 4; BOT = 2; LF  = 3; RT  = 1;	break;
				case 4:	TOP = 2; BOT = 4; LF  = 1; RT  = 3;	break;
				case 6:	TOP = 4; BOT = 2; LF  = 1; RT  = 3;	break;
			}
		break;
		//// TOP/BOT = 3/4 ////
		case 6: 
			switch (LR)
			{
				case 2:	TOP = 3; BOT = 4; LF  = 2; RT  = 1;	break;
				case 3: TOP = 4; BOT = 3; LF  = 2; RT  = 1;	break;
				case 4:	TOP = 3; BOT = 4; LF  = 1; RT  = 2;	break;
				case 5:	TOP = 4; BOT = 3; LF  = 1; RT  = 2;	break;
			}
		break;
	}
}

long diagDistance(int x1, int y1, int x2, int y2)
{
	long x = ((x1 - x2)*(x1 - x2)) + ((y1 - y2)*(y1 - y2));
	return x;
}