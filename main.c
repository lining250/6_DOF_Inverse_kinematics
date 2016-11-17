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
#define precission 4   /* amount of steps per centimeter */
#define mindelay 30.0    /* determines the max velocity of the end effector */
#define increase 4.0      /* I choose to lowest speed to be a factor of 4 smaller than the max speed */
#define ramp 1.0          /* distance over which to increase/decrease speed */
int i;
double angles[7];
int ticks[7];
int fd;
long double temp;
long double t[3][3]= {{0,1,0},    //the target rotation matrix R
                      {0,0,1},
                      {1,0,0}};
long double w[3][3]= {{0,1,0},    //the target rotation matrix R
                      {1,0,0},
                      {0,0,-1}};
/*TODO: XYZ euler matrix with gripper always parallel to ground (roll=0)*/
/*give (non normalized ) aproach and sliding vector, make rotation matrix */
/*improve smoothness motion*/
/*implement acceleration and decelleration*/
/*jacobian?*/

struct Pos{
    long double x; long double y; long double z;
    long double alpha; long double beta; long double gamma;
};

void nsleep(long ms)
{
    long us;
    us = 1000*ms;
    struct timespec wait;
    wait.tv_sec = us / (1000 * 1000);
    wait.tv_nsec = (us % (1000 * 1000)) * 1000;
    nanosleep(&wait, NULL);
}


void line(struct Pos start, struct Pos stop){
    double j;
    double dx,dy,dz,r,dr,x,y,z;
    double delay,current_r;
    int steps;
    dx = stop.x - start.x;
    dy = stop.y - start.y;
    dz = stop.z - start.z;

    r = sqrt(dx*dx+dy*dy+dz*dz); /* total path length */
    steps = r*precission;
    dr = r/steps;

    /* v=a*r²+b for r<=ramp, v=-c*r²+d for r>= ramp*/
    double a = (1.0/(ramp*ramp) )*( (dr/mindelay)-(dr/(increase*mindelay)) );
    double b = dr/(increase*mindelay);
    double c = ( (dr/mindelay) - (dr/(increase*mindelay)) ) / (2*r*ramp - ramp*ramp);
    double d = (dr/(increase*mindelay)) + c*r*r;

    for(j = 0; j <= steps; j++){
        x = start.x + ((j/steps)*dx);
        y = start.y + ((j/steps)*dy);
        z = start.z + ((j/steps)*dz);
        current_r = dr*j;
        inverseKinematics(x,y,z,t,angles);
        commandArduino(fd,angles);

    /* mess due to speed regulation */
    /* v = dr/delay so delay = dr/v */
    if(r<=2*ramp){ /* path too short, v = constant */
        nsleep(mindelay);
    }
    else if(r>2*ramp){  /* v = a*current_r² + b*/
        if (current_r <= ramp){
            delay = dr / ( a*current_r*current_r + b);
            nsleep(delay);
        }
        else if(current_r>ramp && current_r<r-ramp){
            nsleep(delay);
        }
        else if(current_r >= r-ramp){ /*v = -c*current_r² + d */
            delay = dr/( -c*current_r*current_r + d);
            nsleep(delay);
        }
    }
    /* end of mess */
    }

}


int main()
{
    struct Pos start;
    struct Pos stop;
    start.x = -10; start.y = 30; start.z = 25;
    stop.x  = 10; stop.y  = 30; stop.z  = 25;

    struct Pos bottom1;
    struct Pos bottom2;
    bottom1.x = -10; bottom1.y = 30; bottom1.z = 5;
    bottom2.x = 10; bottom2.y = 30; bottom2.z = 5;

    fd = serialport_init("/dev/ttyUSB0", 115200);
    sleep(2);
    inverseKinematics(-10,30,5,t,angles);
    commandArduino(fd, angles);
    sleep(1);

    line(bottom1,start);
    line(start,stop);
    line(stop,start);
    line(start,stop);
    line(stop,start);
    line(start,bottom1);

//    line(bottom1,start,fd);
//    line(start,stop,fd);
//    line(stop,bottom2,fd);
//    line(bottom2,stop,fd);
//    line(stop,start,fd);
//    line(start,bottom1,fd);

    sleep(2);
    serialport_close(fd);

//    int delay = 80;
//    fd = serialport_init("/dev/ttyUSB0", 9600);
//
//    sleep(2);
//    inverseKinematics(10,30,10,r,angles); /* dumps the results in angles */
//    commandArduino(fd, angles);
//    nsleep(1000);
//    for(i=20;i<=40;i++){
//        inverseKinematics(10,30,i/2,r,angles); /* dumps the results in angles */
//        commandArduino(fd, angles);
//        nsleep(delay);
//    }
//    for(i=20;i>=-20;i--){
//        inverseKinematics(i/2,30,20,r,angles); /* dumps the results in angles */
//        commandArduino(fd, angles);
//        nsleep(delay);
//    }
//    for(i=40;i>=20;i--){
//        inverseKinematics(-10,30,i/2,r,angles); /* dumps the results in angles */
//        commandArduino(fd, angles);
//        nsleep(delay);
//    }
//    for(i=-20;i<=20;i++){
//        inverseKinematics(i/2,30,10,r,angles); /* dumps the results in angles */
//        commandArduino(fd, angles);
//        nsleep(delay);
//    }
//
//    sleep(2);
//    serialport_close(fd);
}
