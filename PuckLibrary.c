/* Name: main.c
 * Author: <Samuel Weintraub>
 */

    #define F_CPU 16000000UL // define processor speed

    #include <stdlib.h>
    #include "m_general.h"
    #include "m_usb.h"
    #include "m_bus.h"
    #include "m_port.h"
    #include "PuckLibrary.h"

    #define tranQuant 16
    #define maxADC0dis 4.0
    #define maxADC1 1020.0
    float minADC0;
    #define maxADC1dis 0.521
    #define ft2pix 93.3
    #define rangeCutoff 1000.0  
    #define radius 0.1
    #define BETA 0.2

    volatile int numPeaks;
    volatile int maxValue[3];
    volatile int maxValueTemp;
    volatile int ADCFlag; 
    volatile int cutoff; 
    volatile int range;
    volatile int resetTran;
    volatile int tranNum;

    unsigned char address = 0x20;
    int maxTran[3];
    int tranValues[2][16];
    int j;
    int i;
    int s;
    int temp;
    int temp1;
    int run;
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
    int PuckPosition[4];
    int SeePuck;
    volatile int runConversion;
    volatile int puckConsistent;


void getPuck(){

    angle2PuckOut[1] = angle2Puck;
    OCR4C = 400; 
    tranNum = 0;
    maxValue[0] = 0;
    maxValue[1] = 0;
    maxValue[2] = 0;
    numPeaks = 0;
    run = 1;
    range = 0;
    maxValueTemp = 0;
    resetTran = 0;
    puckConsistent = 0;
    setTran(tranNum);
    setRange(range);
    runConversion = FALSE;

    while(run)
    {
        if(ADCFlag && runConversion)
        {

            // cut out ambient noise
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

            // fix transistor 4
            if(tranNum == 4)
            {
                tranValues[0][tranNum] = 0; // populate array of transistor values
                numPeaks = numPeaks - 1;
                maxValueTemp = 0;
            }

            // keep running max
            if (maxValueTemp > maxValue[0])
            {
                maxValue[0] = maxValueTemp;
                maxTran[0] = tranNum;

                // if maxValue is too high send to low range
                if (maxValue[0] > rangeCutoff)
                {
                    // reset variables
                    range = 1;
                    setRange(range);
                    numPeaks = 0;
                    maxValue[0] = 0;
                    maxValue[1] = 0;
                    maxValue[2] = 0;
                    maxTran[0] = 0;
                    maxValueTemp = 0;

                    // used to set transistor back to 0
                    resetTran = 1;

                    // flush ADC values
                    for(s = 0; s <= tranNum; s++)
                    {
                        tranValues[0][s] = 0;
                    }
                }
            }

            tranValues[1][tranNum] = tranNum; 

            // resent tran to 0 if changing to low range
            if (resetTran)
            {
                tranNum = 0;
                resetTran = 0;
            }
            else
            {   tranNum = tranNum + 1;  }

            // if there is value for all transistors
            if(tranNum == tranQuant)
            {
                // if puck is visible
                if ((numPeaks > 2) || (range == 1))
                {

                    puckConsistent = puckConsistent + 1;
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

                     m_usb_tx_string("\n");
                     m_usb_tx_int(maxAngle[0]);
                     m_usb_tx_string("   ");
                     m_usb_tx_int(maxValue[0]);
                     m_usb_tx_string("        ");
                     m_usb_tx_int(maxAngle[1]);
                     m_usb_tx_string("   ");
                     m_usb_tx_int(maxValue[1]);
                     m_usb_tx_string("        ");
                     m_usb_tx_int(maxAngle[2]);
                     m_usb_tx_string("   ");
                     m_usb_tx_int(maxValue[2]);
                     m_usb_tx_string("        ");
                     m_usb_tx_int(angle2Puck);
                     m_usb_tx_string("   ");
                     m_usb_tx_int(range);
                     m_usb_tx_string("   ");
                     m_usb_tx_long(puckDistance);
                            
                        // convert output
                        if (puckConsistent == 2){
                            // weight average of puck values
                            angle2Puck = (   ((maxValue[0] - cutoff) * maxAngle[0]) + ((maxValue[1] - cutoff) * maxAngle[1]) + ((maxValue[2] - cutoff) * maxAngle[2])  )/
                            ( (maxValue[0] - cutoff) + (maxValue[1] - cutoff) + (maxValue[2] - cutoff)); 

                            // find puck distance
                            puckDistance = findDistance(maxValue[0], range, angle2Puck);
                            convertOutput(angle2Puck, puckDistance);
                            SeePuck = TRUE;
                            run = 0;
                            break;
                        }
                    }

                    //puck is behind
                    else
                    {
                        // find angle and distance

                         m_usb_tx_string("\n");
                         m_usb_tx_int(angle2Puck);
                         m_usb_tx_string("   ");
                         m_usb_tx_long(puckDistance);
                                
                        // convert output
                        if (puckConsistent == 2){
                        angle2Puck = angleTable[maxTran[0]];
                        puckDistance = findDistance(maxValue[0], range, angle2Puck); 
                        convertOutput(angle2Puck, puckDistance);
                        SeePuck = TRUE;
                        run = 0;
                        break;
                    }
                    }  
                }
                // else puck is not visible
                else
                {
                        // angle2Puck = -1;
                        // puckDistance = -1;
                     m_usb_tx_string("\n");
                     m_usb_tx_int(angle2Puck);
                     m_usb_tx_string("   ");
                     m_usb_tx_int(puckDistance);
                    if (puckConsistent == 2){
                    SeePuck = FALSE;
                    run = 0;
                    break;
                    }
                }

                    tranNum = 0;
                    numPeaks = 0;
                    maxValue[0] = 0;
                    maxValueTemp = 0;   
                    resetTran = 0;
                    // send back to high 
                    range = 0;
                    setRange(range);

            }
            ADCFlag = 0;
            setTran(tranNum);
            runConversion = FALSE;
            TCNT4 = 1;

        }

            if ((TCNT4 % 3) == 0)
                {
                    if(!runConversion)
                    {
                        runConversion = TRUE;
                        set(ADCSRA,ADSC);
                    }
                }

        }
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

void setRange(int rangeD)
{
        switch(rangeD) // use D5
        {
            case 0 : // high 
                set(PORTD, 6);
                cutoff = 830.0; //High pass filter cutoff
                m_green(OFF);
                m_red(ON);
                
                break;
            case 1 : // low
                clear(PORTD, 6);
                cutoff = 20.0;
                m_green(ON);
                m_red(OFF);
                
                break;
        }
}

float findDistance(int maxValueD, int rangeT, float angle2Puck)
{
    oldDistance = distance;
    switch(rangeT)
    {
        case 0: //high

            if ((angle2Puck >- 90.0) || (angle2Puck <= 270.0)){
                minADC0 = 940.0 - ((abs(180.0 - angle2Puck)/90.0) * 60);
            }
            else{
                minADC0 = 880.0;
            }

            distance = (maxADC0dis - ((float)((maxValueD - minADC0)/1.0/(rangeCutoff - minADC0)) * maxADC0dis) + maxADC1dis) *ft2pix;
            break;
        case 1: //low
            distance = (maxADC1dis - ((float)((maxValueD)/1.0 / (maxADC1))*maxADC1dis) + radius) *ft2pix;
            break;
    }
    distance = BETA*distance + (1-BETA)*oldDistance;
    return distance; 
}

void convertOutput(float angle2Puck, float puckDistance)
{
    // re-orient output angle such that it is in reference to the front of the robot
    oldAngle = angle2Puck;
    angle2Puck = angle2Puck + 180.0;
    if (angle2Puck >= 360)
    {   angle2Puck = angle2Puck - 360.0;    }

    // change puck to field reference

    angle2Puck = angle2Puck + Heading[0]; 

    if (angle2Puck > 360)
    {   angle2Puck = angle2Puck - 360.0;    }

    int angle2PuckAlt = angle2Puck;
    int oldAngleAlt = oldAngle;
    
    if((angle2Puck - oldAngle) < -180){
        oldAngleAlt = oldAngle - 360;
    }else if((angle2Puck - oldAngle) > 180) {
        angle2PuckAlt = angle2Puck - 360;
    }
    
    angle2Puck = BETA*angle2PuckAlt + (1-BETA)*oldAngleAlt;

    angle2PuckOut[0] = angle2Puck;

    if(angle2Puck < 0)
    {   angle2Puck = angle2Puck + 360;  }
    
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


