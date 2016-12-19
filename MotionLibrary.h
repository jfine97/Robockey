
#ifndef MotionLibrary__
#define MotionLibrary__

float VoltageScale;
int velocity;
float Heading[2];
int PosX[2];
int PosY[2];
int PuckPosition[4];
float Kp;
float Kd;

void DriveMotorA(int POWER, int turbo);
void DriveMotorB(int POWER, int turbo);
void Drive(int Speed, int state, int turbo);
void GetDriveHeading(int state, int side);
void SetDriveHeading(float heading);

#endif