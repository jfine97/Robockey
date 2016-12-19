

#define F_CPU 16000000UL
#include <stdlib.h>
#include "m_general.h"
#include "m_bus.h"
#include "m_usb.h"
#include "m_wii.h"
#include "m_rf.h"
#include "m_port.h"
#include "MotionLibrary.h"
#include "Local_in_1.h"

#define ADDRESS 0x10    //Bot #1 = 0x10  Bot #2 = 0x11   Bot #3 = 0x12
#define ALPHA 0.4
#define PHI 0.3

#define STOP 1
#define SEARCH_FOR_PUCK 2
#define GO_TO_PUCK 3
#define GO_TO_GOAL 4
#define CALIBRATE 5

#define maxADC0dis 4.0
#define maxADC0 1020.0
float minADC0;
#define ft2pix 93.3
#define radiusFt 0.1
#define BETA 0.2
#define cutoff 800.0
#define radiusFt 0.1

    volatile int numPeaks;
    volatile int maxValue[3];
    volatile int maxValueTemp;
    volatile int ADCFlag; 
	int tranNum;

    unsigned char address = 0x20;
    int maxTran[3];
    int tranValues[2][16];
    int j;
    int i;
    int temp;
    int temp1;
    int sep[2];
    int sign1;
    int sign2;
    
    // angles at the back of the robot and sweep clockwise around it
    float angleTable[16] = {0.0, 324.0, 288.0, 270.0, 252.0, 234.0, 216.0, 198.0, 180.0, 162.0, 144.0, 126.0, 108.0, 90.0, 72.0, 36.0};
    float distance;
    float angle2Puck;
    float angle2PuckOut[2];
    float oldAngle;
    float puckDistance;
    float oldDistance;
    float maxAngle[3];

    // global variables
    float Heading[2];
    int PosX[2];
    int PosY[2];



void init();
void GetLocalized();
void Calibrate();
void GetPuckPosition();
void convertOutput();
void findDistance();
void setTran(int tranNumD);
float filterAngle(float newAngle, float oldAngle);

int STATE;
int SIDE;
int HavePuck;
int SeePuck;

float VoltageScale;
char buffer[10] = {0,0,0,0,0,0,0,0,0,0};
int wirelessRecieved;
int* locusData;
int NumStars;
int PuckPosition[4] = {0,0,0,0};
int localization_ctr;
int main_code_ctr;
int puckDetection_ctr;
int randomShit_ctr;
int calibration_ctr;
int turboMode = FALSE;

float startAngle;
float checkAngle;
float PuckAngle[2];
int Pdistance[2];
int XCal[2];
int YCal[2];
int BotCal[2];
int angleCtr;
int temp;

/////////// SETTINGS ///////////
int stateOutput = GO_TO_PUCK;



/////////CALIBRATION SETTINGS/////////
int radius = 145;
float offsetTheta = 59;
long offsetVect[2];

unsigned int mWiiData_[12];
unsigned int* mWiiData_ptr = &mWiiData_[0];
int BotCenter_[2] = {512, 384};
int* BotCenter_ptr = &BotCenter_[0];

int main(void)
{
	STATE = STOP;
	
	init();
	
	m_port_clear(0x20,PORTH,3);
	m_port_clear(0x20,PORTH,2);
	m_port_clear(0x20,PORTH,1);
	m_port_set(0x20,PORTH,3);
	m_wait(200);
	m_port_clear(0x20,PORTH,3);
	m_wait(200);
	m_port_set(0x20,PORTH,2);
	m_wait(200);
	m_port_clear(0x20,PORTH,2);
	m_wait(200);
	calibration_ctr = 0;
	
	Calibrate();
	
// 	STATE = GO_TO_PUCK;
// 	SeePuck = TRUE;
// 	HavePuck = FALSE;
	
// 	STATE = SEARCH_FOR_PUCK;
// 	SeePuck = FALSE;
// 	HavePuck = FALSE;
	
// 	STATE = GO_TO_GOAL;
// 	SeePuck = TRUE;
// 	HavePuck = TRUE;
	
	while (1)
	{
		////**** CHECK WIRELESS COMMANDS ****////
		if(wirelessRecieved)
		{
			wirelessRecieved = 0;
			m_rf_read(buffer, 10); //Read wireless signals from field
			switch(buffer[0])
			{
				case 0xA0:										//Comm Test
					for(int i=0;i<3;i++)
					{
						clear(PORTB,(3*SIDE));
						m_wait(250);
						set(PORTB,(3*SIDE));
						m_wait(250);
					}
					break;
				case 0xA1:	STATE = SEARCH_FOR_PUCK;  break;	//Play
				case 0xA2:	STATE = STOP;	break;				//Goal Red
				case 0xA3:	STATE = STOP;	break;				//Goal Blue
				case 0xA4:	STATE = STOP;	break;				//Pause
				case 0xA6:	STATE = STOP;	break;				//Halftime
				case 0xA7:	STATE = STOP;	break;				//Game Over
				default: break;
			}
		}		
		
// 	char rx_buffer = m_usb_rx_char();  			// Read the packet from the computer
// 
// 	m_usb_rx_flush();  						// Flush the buffer
// 
// 	if(rx_buffer == 1)						// MATLAB is expecting IMU data
//  	{										// For PLOT THE STARS
// 		m_usb_tx_int(PuckPosition[0]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(PuckPosition[1]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(PosX[0]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(PosY[0]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(PuckPosition[0]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(PuckPosition[1]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(PosX[0]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(PosY[0]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(Heading[0]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_int(Heading[0]);
// 		m_usb_tx_char('\t');
// 		m_usb_tx_string("\n");					// MATLAB serial command reads 1 line at a time
// 		m_green(TOGGLE);
// 	}
			
	
////////////////////********************    TRANSISTOR STUFF    ********************////////////////////
		if(ADCFlag == 1)
		{	
			if(ADC > cutoff)
			{
		        tranValues[0][tranNum] = ADC; // populate array of transistor values
		        maxValueTemp = ADC;
		        numPeaks = numPeaks + 1;
		    }
		    else
			{
		        tranValues[0][tranNum] = 0; // populate array of transistor values
		        maxValueTemp = 0;
		    }
			
		    if (maxValueTemp > maxValue[0])
			{
		        maxValue[0] = maxValueTemp;
		        maxTran[0] = tranNum;
		    }

		    tranValues[1][tranNum] = tranNum;

		    tranNum = tranNum + 1;

		    if (tranNum == 16)
			{
						  
						    	// if puck is visible
	    		if (numPeaks > 2)
				{

						m_green(TOGGLE);

					// if puck is in front
					if ((maxTran[0] > 2) && (maxTran[0] < 14))
					{
						//sort values in ascending order
						for(j=1;j<16;j++)
						{   for(i=0; i<15; i++)
							{
								if(tranValues[0][i]>tranValues[0][i+1])
								{
									temp = tranValues[0][i];
									tranValues[0][i] = tranValues[0][i+1];
									tranValues[0][i+1] = temp;

									temp1 = tranValues[1][i];
									tranValues[1][i] = tranValues[1][i+1];
									tranValues[1][i+1] = temp1;
								}
							}
						}

						// find 2nd and 3rd max value and corresponding transistors and corresponding angles
						maxAngle[0] = angleTable[maxTran[0]];

						maxValue[1] = tranValues[0][14];
						maxTran[1] = tranValues[1][14];
						maxAngle[1] = angleTable[maxTran[1]];

						maxValue[2] = tranValues[0][13];
						maxTran[2] = tranValues[1][13];
						maxAngle[2] = angleTable[maxTran[2]];

						//cut out non-clustered stuff...
						sep[0] = abs(maxTran[0] - maxTran[1]);
						sep[1] = abs(maxTran[0] - maxTran[2]);

						// find which one is farthest and send to 0 
						if (sep[0] > 2)
						{   maxValue[1] = cutoff;   }   // send to zero so that it has no weight in the average heading

						if (sep[1] > 2)
						{   maxValue[2] = cutoff;   }   // send to zero so that it has no weight in the average heading
	                            
						// convert output
							// weight average of puck values
							angle2Puck = (   ((maxValue[0] - cutoff) * maxAngle[0]) + ((maxValue[1] - cutoff) * maxAngle[1]) + ((maxValue[2] - cutoff) * maxAngle[2])  )/
							( (maxValue[0] - cutoff) + (maxValue[1] - cutoff) + (maxValue[2] - cutoff)); 
							angle2Puck = angle2Puck + 180;
							// find puck distance
							PuckAngle[1] = PuckAngle[0];
							Pdistance[1] = Pdistance[0];
							
							findDistance();
							convertOutput();
							PuckAngle[0] = angle2Puck - 90;
							Pdistance[0] = puckDistance;
							PuckAngle[0] = filterAngle(PuckAngle[0],PuckAngle[1]);
							SeePuck = TRUE;
							
							m_green(TOGGLE);
// 							m_usb_tx_string("\n");
// 							m_usb_tx_int(maxAngle[0]);
// 							m_usb_tx_string("   ");
// 							m_usb_tx_int(maxValue[0]);
// 							m_usb_tx_string("        ");
// 							m_usb_tx_int(maxAngle[1]);
// 							m_usb_tx_string("   ");
// 							m_usb_tx_int(maxValue[1]);
// 							m_usb_tx_string("        ");
// 							m_usb_tx_int(maxAngle[2]);
// 							m_usb_tx_string("   ");
// 							m_usb_tx_int(maxValue[2]);
// 							m_usb_tx_string("        ");
// 							m_usb_tx_int(angle2Puck);
// 							m_usb_tx_string("   ");
// 							m_usb_tx_long(PuckAngle[0]);
														
														
					}

					//puck is behind
					else
					{
						// find angle and distance
	                                
						// convert output
						PuckAngle[1] = PuckAngle[0];
						Pdistance[1] = Pdistance[0];
						angle2Puck = angleTable[maxTran[0]] + 180;
						findDistance(); 
						convertOutput();
						PuckAngle[0] = angle2Puck - 90;
						Pdistance[0] = puckDistance;
						PuckAngle[0] = filterAngle(PuckAngle[0],PuckAngle[1]);
						SeePuck = TRUE;
						
// 						                                m_usb_tx_string("\n");
// 						                                m_usb_tx_int(angle2Puck);
// 						                                m_usb_tx_string("   ");
// 						                                m_usb_tx_long(puckDistance);
					}  
				}

				// else puck is not visible
				else
				{
// 					m_usb_tx_string("\n");
// 					m_usb_tx_int(angle2Puck);
// 					m_usb_tx_string("   ");
// 					m_usb_tx_int(puckDistance);
					
					
					SeePuck = FALSE;
					m_red(TOGGLE);

				}

	        // reset values
	        numPeaks = 0;
	        tranNum = 0;
	        maxValue[0] = 0;
	        maxValueTemp = 0;
	        maxTran[0] = 0;

	    }

	    setTran(tranNum);
	    ADCFlag = 0;

	}



		
////////////////////////** END TRANSISTOR STUFF **////////////////////////////////////////////////////
			
		if(randomShit_ctr >= 300)  //.75Hz
		{
			randomShit_ctr = 0;
			turboMode = FALSE;
		 	if(check(PINC,6) == 1)
		 	{	SIDE = 1; set(PORTB,3); clear(PORTB,0);	}	//Bot is on right side (BLUE)
		 	else
		 	{	SIDE = 0; set(PORTB,0);	clear(PORTB,3);}	//Bot is on left side  (RED)
		}
	
		////**** PUCK DETECTION ****////
// 		if(puckDetection_ctr >= 10) //20Hz
// 		{
// 			
// 			puckDetection_ctr = 0;
// 			GetPuckPosition();
// 			m_green(TOGGLE);
// 			m_usb_tx_string("         ");
// 			m_usb_tx_int(PosX[0]);
// 			m_usb_tx_string("   ");
// 			m_usb_tx_int(PosY[0]);
// 			m_usb_tx_string("   ");
// 			m_usb_tx_int(PuckPosition[0]);
// 			m_usb_tx_string("   ");
// 			m_usb_tx_int(PuckPosition[1]);
// 			m_usb_tx_string("\n");
// 		}
			
		////**** LOCALIZE ****////
		if (localization_ctr >= 4) //50Hz
		{
			localization_ctr = 0;  //reset ctr
			GetLocalized();
		}
	
	
	
		////**** START MAIN CODE ****////
		if (main_code_ctr >= 8)   //25Hz
		{
			main_code_ctr = 0;  //reset ctr
			
			////**** UPDATE STATE CHANGERS ****////
			if(check(PINC,7) == 1)
			{	HavePuck = TRUE;	}
			else
			{	HavePuck = FALSE;	}
				
				
				
				
				
				
				
					
				
			switch (STATE)
			{
				
				
				
	////////////////////********************    STOP    ********************////////////////////
				case STOP: // case 1
// 					m_usb_tx_string("STOP"); m_usb_tx_string("\n");
					OCR1A = 0;
					OCR1B = 0;
					m_port_clear(0x20,PORTH,3);
					m_port_clear(0x20,PORTH,2);
					m_port_clear(0x20,PORTH,1);





// 					m_usb_tx_string("\n");
					break;
				
	////////////////////********************    SEARCH FOR PUCK    ********************////////////////////
				case SEARCH_FOR_PUCK:	//case 2
// 					m_usb_tx_string("SEARCH FOR PUCK"); m_usb_tx_string("\n");

					GetDriveHeading(SEARCH_FOR_PUCK,SIDE);
				
					Drive(velocity,STATE,0);
				
					if(SeePuck == TRUE)
					{	STATE = GO_TO_PUCK;	}
					break;
				
				
	////////////////////********************    GO TO PUCK    ********************////////////////////
				case GO_TO_PUCK:	//case 3
// 					m_usb_tx_string("GO TO PUCK"); m_usb_tx_string("\n");

// 					m_green(TOGGLE);
					SetDriveHeading(PuckAngle[0]);
				

// 					m_usb_tx_int(VoltageScale); m_usb_tx_string("    ");
					Drive(0,STATE,turboMode);
				
					if(HavePuck == TRUE)
					{	STATE = GO_TO_GOAL;	}
					else if(SeePuck == FALSE)
					{	STATE = SEARCH_FOR_PUCK;	}
					break;
				
	////////////////////********************    GO TO GOAL    ********************////////////////////
				case GO_TO_GOAL:	//case 4
// 					m_usb_tx_string("GO TO GOAL"); m_usb_tx_string("\n");
					
					GetDriveHeading(GO_TO_GOAL,SIDE);
					
					Drive(velocity,STATE,turboMode);

					if(HavePuck == FALSE)
					{	STATE = SEARCH_FOR_PUCK;	}
					break;
				
	////////////////////********************    CALIBRATION    ********************////////////////////
				case CALIBRATE:
					
					SetDriveHeading(startAngle);
					Drive(0,CALIBRATE,0);
					
					if(calibration_ctr >= 400) //2 sec
					{
						calibration_ctr = 0;
						if(angleCtr == 0)
						{
							angleCtr++;
							
							startAngle = startAngle + 180;
							if(startAngle > 360)
							{	startAngle = startAngle - 360;	}
							
							XCal[0] = PosX[0];
							YCal[0] = PosY[0];	
							
						}
						else if(angleCtr == 1)
						{
							angleCtr++;
							
							XCal[1] = PosX[0];
							YCal[1] = PosY[0];
							
							BotCal[0] = (XCal[1] + XCal[0])/2;
							BotCal[1] = (YCal[1] + YCal[0])/2;
							
							offsetVect[0] = XCal[0] - BotCal[0];
							offsetVect[1] = YCal[0] - BotCal[1];
							radius = sqrt((offsetVect[0]*offsetVect[0]) + (offsetVect[1]*offsetVect[1]));
							
							offsetTheta = 90 - atan2(offsetVect[1],offsetVect[0])*180/M_PI;
							
							if(SIDE == 1)
							{	startAngle = 90;	}
							else
							{	startAngle = 270;	}
						}
						else
						{
							STATE = stateOutput;
						}
					}

					
// 					m_red(TOGGLE);
// 					m_usb_tx_string("CALIBRATION");
// 					m_usb_tx_string("   ");
// 					m_usb_tx_int(Heading[0]);
// 					m_usb_tx_string("   ");
// 					m_usb_tx_int(checkAngle);
// 					m_usb_tx_string("   ");
// 					m_usb_tx_int(abs(Heading[0]-Heading[1]));
// 					m_usb_tx_string("   ");
// 					m_usb_tx_int(angleCtr);
// 					m_usb_tx_string("\n");
					break;
			}
		}
	}

}

void init()
{
	m_red(ON);
	
	////////			GENERAL INIT			////////
	m_clockdivide(0);
	set(DDRB,1);
	set(DDRB,2);
	
	clear(DDRC,6);  //Side switch
	clear(DDRC,7);  //Limit switch
	m_disableJTAG();
	////////			PERIFERAL INIT			////////
	m_bus_init();
	m_port_init(0x20);
	m_usb_init();
	m_wii_open();
	m_rf_open(1,ADDRESS,10);
	
	////////			TIMER 1 INIT			////////
	clear(TCCR1B,WGM13); //set Timer UP to 03FF, PWM mode
	set(TCCR1B,WGM12);
	set(TCCR1A,WGM11);
	set(TCCR1A,WGM10);
	
	clear(TCCR1B, CS12); //set the prescaler to /8
	set(TCCR1B, CS11);	 //timer running at 1955Hz
	clear(TCCR1B,CS10);

	set(DDRB,5);  //enable pin B5 for output
	set(DDRB,6);  //enable pin B6 for output
	set(PORTD, 6);
	
	set(TCCR1A,COM1A1);   //clear pin B5 at OCR1A, set at rollover
	clear(TCCR1A,COM1A0);
	
	set(TCCR1A,COM1B1);   //clear pin B6 at OCR1B, set at rollover
	clear(TCCR1A,COM1B0);
	
	////////			TIMER 3 INIT			////////
	clear(TCCR3B,WGM33); //count up to OCR3A Mode 4
	set(TCCR3B,WGM32);
	clear(TCCR3A,WGM31);
	clear(TCCR3A,WGM30);
	
	clear(TCCR3B, CS32); //set the prescaler to /8
	set(TCCR3B, CS31);	 //timer running at 2MHz
	clear(TCCR3B,CS30);
	
	set(TIMSK3, OCIE3A);   //call interrupt at OCR3A
	
	OCR3A = 5000;
	
	////////			ADC INIT			////////
	clear(ADMUX,REFS1); //set the reference voltage to Vcc
	set(ADMUX,REFS0);
	
	set(ADCSRA,ADPS2);   //set ADC Prescaler to /128
	set(ADCSRA,ADPS1);
	set(ADCSRA,ADPS0);
	
	set(DIDR0, ADC0D);   //disable the pins digital input for F0
	
	set(ADCSRA, ADATE);  //enable "free-running" mode
	
	clear(ADCSRB,MUX5);   //selecting the F0 channel
	clear(ADMUX,MUX2);
	clear(ADMUX,MUX1);
	clear(ADMUX,MUX0);
	
	set(ADCSRA,ADEN);  //enable ADC subsystem
	set(ADCSRA,ADSC);  //begin conversion

	
	
// 	while(check(ADCSRA,ADSC)) {}  //wait till conversion is done
// 	VoltageScale = 12.0/(4*5*(ADC/1023.0));
	VoltageScale = 12.0/16.5;
	temp = ADC;
	clear(ADCSRA,ADEN);  //disable ADC subsystem
	
	////////   PUCK ADC INIT    ////////
	
	set(DIDR2,ADC8D);	//disable the pins digital input for D4
	
	set(ADCSRB,MUX5);   //selecting the F0 channel
	clear(ADMUX,MUX2);
	clear(ADMUX,MUX1);
	clear(ADMUX,MUX0);
	
	set(ADCSRA,ADEN);  //enable ADC subsystem
	set(ADCSRA, ADIE); // enable ADC_vect interrupt
	
	////////			PORT EXPANDER INIT			////////
	m_port_set(0x20, DDRG, 0);	// enable G0 for output --> S0 on multi
	m_port_set(0x20, DDRG, 1);	// enable G1 for output --> S1 on multi
	m_port_set(0x20, DDRG, 2);	// enable G2 for output --> S2 on multi
	m_port_set(0x20, DDRG, 3);	// enable G3 for output --> S3 on multi
	
	clear(DDRD,4); //enable pin D4 for input
	
	set(DDRD, 6); // enable D5 for output --> Range 0 vs. 1
	
	////////			LAST INIT			////////
	set(DDRB,0);  //Blue LED Enable
	set(DDRB,3);  //Red LED Enable
	
	if(check(PINC,6) == 1)													///////////////set correct pin
	{	SIDE = 1; set(PORTB,0);	}	//Bot is on right side (RED)
	else
	{	SIDE = 0; set(PORTB,3);	}	//Bot is on left side  (BLUE)
	
	m_port_set(0x20, DDRH, 3);  //Green LED
	m_port_set(0x20, DDRH, 2);	//Blue LED
	m_port_set(0x20, DDRH, 1);	// LED

	
	set(DDRB,1);
	set(DDRB,2);
	sei();
	
	OldTB = 0;
	OldLR = 0;
	
	
	tranNum = 0;
	setTran(tranNum);
	maxValue[0] = 0;
	maxValueTemp = 0;
	maxTran[0] = 0;
	numPeaks = 0;
	set(ADCSRA, ADSC); // start ADC conversion

	m_red(OFF);

}

void GetLocalized(){
	////**** STORE LAST POSITION ****///
	/
	Heading[1] = Heading[0];
	PosX[1] = PosX[0];
	PosY[1] = PosY[0];

	////**** STORE NEW POSITION ****////
	m_wii_read(mWiiData_ptr);
	localize(mWiiData_ptr, BotCenter_ptr);
	Heading[0] = HEADING;
	PosX[0] = X + (radius*cos(((offsetTheta - Heading[0])*M_PI/180)));
	PosY[0] = Y - (radius*sin((offsetTheta - Heading[0])*M_PI/180));
	NumStars = numstars;
	
		////**** FILTERING HEADING & POSITION ****////
	int Heading0ALT = Heading[0];
	int Heading1ALT = Heading[1];
	
	if((Heading[0] - Heading[1]) < -180){
		Heading1ALT = Heading[1] - 360;
	}else if((Heading[0] - Heading[1]) > 180) {
		Heading0ALT = Heading[0] - 360;
	}
	
	Heading[0] = ALPHA*Heading0ALT + (1-ALPHA)*Heading1ALT;
	PosX[0] = ALPHA*PosX[0] + (1-ALPHA)*PosX[1];
	PosY[0] = ALPHA*PosY[0] + (1-ALPHA)*PosY[1];
	
	if(Heading[0] < 0)
	{
		Heading[0] = 360 + Heading[0];
	}
		
	if (NumStars == 4)
	{
		m_port_set(0x20,PORTH,3);   //Turn on Green LED
		m_port_clear(0x20,PORTH,2);
	}
	else if (NumStars <= 3)
	{
		m_port_set(0x20,PORTH,2);  //Turn on Blue LED
		m_port_clear(0x20,PORTH,3);
	}
}



void Calibrate()
{
	startAngle = 270;	//Bot is on left side
	angleCtr = 0;
	STATE = CALIBRATE;
}

void setTran(int tranNumD) 
{
    switch(tranNumD) 
    {
        case 0 : // LLLL
            m_port_clear(address, PORTG, 0);
            m_port_clear(address, PORTG, 1);
            m_port_clear(address, PORTG, 2);
            m_port_clear(address, PORTG, 3);
            break;
        case 1 : // HLLL

            m_port_set(address, PORTG, 0);
            m_port_clear(address, PORTG, 1);
            m_port_clear(address, PORTG, 2);
            m_port_clear(address, PORTG, 3);
            break;
        case 2 : // LHLL

            m_port_clear(address, PORTG, 0);
            m_port_set(address, PORTG, 1);
            m_port_clear(address, PORTG, 2);
            m_port_clear(address, PORTG, 3);
            break;
        case 3 : // HHLL

            m_port_set(address, PORTG, 0);
            m_port_set(address, PORTG, 1);
            m_port_clear(address, PORTG, 2);
            m_port_clear(address, PORTG, 3);
            break;
        case 4 : // LLHL

            m_port_clear(address, PORTG, 0);
            m_port_clear(address, PORTG, 1);
            m_port_set(address, PORTG, 2);
            m_port_clear(address, PORTG, 3);
            break;
        case 5 : // HLHL

            m_port_set(address, PORTG, 0);
            m_port_clear(address, PORTG, 1);
            m_port_set(address, PORTG, 2);
            m_port_clear(address, PORTG, 3);
            break;
        case 6 : // LHHL

            m_port_clear(address, PORTG, 0);
            m_port_set(address, PORTG, 1);
            m_port_set(address, PORTG, 2);
            m_port_clear(address, PORTG, 3);
            break;
        case 7 : // HHHL

            m_port_set(address, PORTG, 0);
            m_port_set(address, PORTG, 1);
            m_port_set(address, PORTG, 2);
            m_port_clear(address, PORTG, 3);
            break;
        case 8 : // LLLH

            m_port_clear(address, PORTG, 0);
            m_port_clear(address, PORTG, 1);
            m_port_clear(address, PORTG, 2);
            m_port_set(address, PORTG, 3);
            break;
        case 9 : // HLLH

            m_port_set(address, PORTG, 0);
            m_port_clear(address, PORTG, 1);
            m_port_clear(address, PORTG, 2);
            m_port_set(address, PORTG, 3);
            break;
        case 10 : // LHLH

            m_port_clear(address, PORTG, 0);
            m_port_set(address, PORTG, 1);
            m_port_clear(address, PORTG, 2);
            m_port_set(address, PORTG, 3);
            break;
        case 11 : // HHLH

            m_port_set(address, PORTG, 0);
            m_port_set(address, PORTG, 1);
            m_port_clear(address, PORTG, 2);
            m_port_set(address, PORTG, 3);
            break;
        case 12 : // LLHH

            m_port_clear(address, PORTG, 0);
            m_port_clear(address, PORTG, 1);
            m_port_set(address, PORTG, 2);
            m_port_set(address, PORTG, 3);
            break;
        case 13 : // HLHH

            m_port_set(address, PORTG, 0);
            m_port_clear(address, PORTG, 1);
            m_port_set(address, PORTG, 2);
            m_port_set(address, PORTG, 3);
            break;
        case 14 : // LHHH

            m_port_clear(address, PORTG, 0);
            m_port_set(address, PORTG, 1);  
            m_port_set(address, PORTG, 2);
            m_port_set(address, PORTG, 3);
            break;
        case 15 : // HHHH

            m_port_set(address, PORTG, 0);
            m_port_set(address, PORTG, 1);
            m_port_set(address, PORTG, 2);
            m_port_set(address, PORTG, 3);
            break;
        }
    }

void findDistance()
{
//     oldDistance = distance;

            if ((angle2Puck >= 90.0) || (angle2Puck <= 270.0)){
                minADC0 = 350.0 - ((abs(180.0 - angle2Puck)/90.0) * 40);
            }
            else{
                minADC0 = 880.0;
            }

            puckDistance = (maxADC0dis - ((float)((maxValue[0] - minADC0)/1.0/(maxADC0 - minADC0)) * maxADC0dis) + radiusFt) * ft2pix;

//     distance = BETA*distance + (1-BETA)*oldDistance;
}

void convertOutput()
{
    // re-orient output angle such that it is in reference to the front of the robot

    angle2Puck = angle2Puck + 180.0;
    if (angle2Puck >= 360)
    {   angle2Puck = angle2Puck - 360.0;    }

    // change puck to field reference

//     angle2Puck = angle2Puck + Heading[0]; 

//     if (angle2Puck > 360)
//     {   angle2Puck = angle2Puck - 360.0;    }


    if ((angle2Puck >= 0) && (angle2Puck < 90))
    {   sign1 = -1; sign2 = 1;   }
    else if ((angle2Puck >= 90) && (angle2Puck < 180))
    {   sign1 = -1; sign2 = -1;   }
    else if ((angle2Puck >= 180) && (angle2Puck < 270))
    {   sign1 = 1;  sign2 = -1;   }
    else
    {   sign1 = 1;  sign2 = 1;   }
    PuckPosition[0] = sign1*(puckDistance * sin(angle2Puck * (M_PI/180.0)));
    PuckPosition[0] = PuckPosition[0] - PosX[0];
    PuckPosition[1] = sign2*(puckDistance * cos(angle2Puck * (M_PI/180.0)));
    PuckPosition[1] = PuckPosition[1] + PosY[0];

}

float filterAngle(float newAngle, float oldAngle)
{
		float newAlt = newAngle;
		float oldAlt = oldAngle;
		
		if((newAngle - oldAngle) < -180){
			oldAlt = oldAngle - 360;
		}else if((newAngle - oldAngle) > 180) {
			newAlt = newAngle - 360;
		}
		
		newAngle = PHI*newAlt + (1-PHI)*oldAlt;
		
		if(newAngle < 0)
		{
			newAngle = 360 + newAlt;
		}
		return newAngle;
	
}

ISR(INT2_vect)
{
	wirelessRecieved = 1;
}

ISR(TIMER3_COMPA_vect)
{
	localization_ctr++;
	main_code_ctr++;
	puckDetection_ctr++;
	randomShit_ctr++;
	calibration_ctr++;
}


ISR(ADC_vect){
	ADCFlag = 1;
}






