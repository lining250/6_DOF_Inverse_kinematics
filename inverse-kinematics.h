#ifndef __INVERSE_KINEMATICS_H__
#define __INVERSE_KINEMATICS_H__

void inverseKinematics(long double x,long double y,long double z,long double t[3][3],double angles[7]);
int getServoTick(float val, int servoNumber);
void cross(float *vec1, float *vec2, float *res);
void sendStuff(int fd, short val);
void commandArduino(int fd, double angles[7]);
void normalize(float *vec);

#endif
