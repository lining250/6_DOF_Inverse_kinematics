#include <string.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include "arduino-serial-lib.h"
#include "inverse-kinematics.h"
#define pi  3.14159265358979323846264338327950288419716939937510L
#define degtorad 0.0174532925
#define radtodeg 57.29577951
float i;
double angles[7];
int ticks[7];
int fd;
long double temp;
long double r[3][3]= {{0,1,0},    //the target rotation matrix R
                      {0,0,1},
                      {-1,0,0}};
long double w[3][3]= {{0,1,0},    //the target rotation matrix R
                      {-1,0,0},
                      {0,0,-1}};
/*TODO: XYZ euler matrix with gripper always parallel to ground (roll=0)*/
/*give (non normalized ) aproach and sliding vector, make rotation matrix */
/*improve smoothness motion*/
/*implement acceleration and decelleration*/
/*jacobian?*/

void nsleep(long ms)
{
    long us;
    us = 1000*ms;
    struct timespec wait;
    //printf("Will sleep for is %ld\n", diff); //This will take extra ~70 microseconds

    wait.tv_sec = us / (1000 * 1000);
    wait.tv_nsec = (us % (1000 * 1000)) * 1000;
    nanosleep(&wait, NULL);
}


int main()
{
    int delay=80;
    //inverseKinematics(-10,30,15,r,angles);
    fd = serialport_init("/dev/ttyUSB0", 9600);
    sleep(2);
    inverseKinematics(10,30,10,r,angles); /* dumps the results in angles */
    commandArduino(fd, angles);
    nsleep(1000);
    for(i=20;i<=40;i++){
        inverseKinematics(10,30,i/2,r,angles); /* dumps the results in angles */
        commandArduino(fd, angles);
        nsleep(delay);
    }
    for(i=20;i>=-20;i--){
        inverseKinematics(i/2,30,20,r,angles); /* dumps the results in angles */
        commandArduino(fd, angles);
        nsleep(delay);
    }
    for(i=40;i>=20;i--){
        inverseKinematics(-10,30,i/2,r,angles); /* dumps the results in angles */
        commandArduino(fd, angles);
        nsleep(delay);
    }
    for(i=-20;i<=20;i++){
        inverseKinematics(i/2,30,10,r,angles); /* dumps the results in angles */
        commandArduino(fd, angles);
        nsleep(delay);
    }

    sleep(2);
    serialport_close(fd);
}
