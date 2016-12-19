#ifndef PuckLibrary__
#define PuckLibrary__

	void getPuck();
    void setTran(int tranNumD);
    void setRange(int rangeD);
    float findDistance(int maxValueD, int rangeT, float angle2Puck);
    void convertOutput(float angle2Puck, float puckDistance);

	float Heading[2];
	int PosX[2];
	int PosY[2];
	int PuckPosition[4];
	int SeePuck;
	float angle2PuckOut[2];
	volatile int ADCFlag;
	volatile int runConversion;

#endif