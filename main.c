#include <string.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include "arduino-serial-lib.h"
#include "inverse-kinematics.h"

#define pi  3.14159265358979323846264338327950288419716939937510
#define degtorad 0.0174532925
#define radtodeg 57.29577951
#define precision 4   /* amount of steps per centimeter */
#define mindelay 30.0    /* determines the max velocity of the end effector */
#define increase 4.0      /* I choose to lowest speed to be a factor of 4 smaller than the max speed */
#define ramp 1.0          /* distance over which to increase/decrease speed */
double  dr = 1.0/precision; /* step size for the line function*/
int i;
double angles[7];
int ticks[7];
int fd;  /* for serial communication */
long double temp;
long double t[3][3]= {{0,1,0},    //the target rotation matrix R
                      {0,0,1},
                      {1,0,0}};
long double w[3][3]= {{0,1,0},    //the target rotation matrix R
                      {1,0,0},
                      {0,0,-1}};

/*to do: jacobian*/

/* roll pitch yaw matrix with the columns permutated z->y, y->x x->z */
/* so that by default the gripper is in the y_0 direction and the  slider is in the x_0 direction */
void eulerMatrix(long double alpha, long double beta, long double gamma, long double m[3][3]){
    long double ca,cb,cg,sa,sb,sg;
    ca = cosl(alpha); cb = cosl(beta); cg = cosl(gamma);
    sa = sinl(alpha); sb = sinl(beta); sg = sinl(gamma);
    m[0][0] = sb;
    m[1][0] = -cb*sg;
    m[2][0] = cb*cg;

    m[0][1] = ca*cb;
    m[1][1] = cg*sa+ca*sb*sg;
    m[2][1] = -ca*cg*sb+sa*sg;

    m[0][2] = -cb*sa;
    m[1][2] = ca*cg-sa*sb*sg;
    m[2][2] = sa*sb+ca*sg;
}

struct Pos{  /* I decided to put the 6 inputs for position in 1 struct for clarity */
    long double x; long double y; long double z;
    long double alpha; long double beta; long double gamma; /* euler angles for the target orientation */
};

void msleep(long ms)  /* delay function in miliseconds*/
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
    double dx,dy,dz,r,x,y,z;
    double delay,current_r;
    int steps;
    dx = stop.x - start.x;
    dy = stop.y - start.y;
    dz = stop.z - start.z;
    r = sqrt(dx*dx+dy*dy+dz*dz); /* total path length */
    steps = r*precision;
    /* v=a*r²+b for current_r<=ramp, v=-c*r²+d for current_r>= r - ramp*/
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

    /* mess due to accelleration */
    if(r<=2*ramp){ /* path too short, v = constant */
        msleep(mindelay);
    }
    else if(r>2*ramp){  /* v = a*current_r² + b*/
        if (current_r <= ramp){
            delay = dr / ( a*current_r*current_r + b);  /* v = dr/delay so delay = dr/v */
            msleep(delay);
        }
        else if(current_r>ramp && current_r<r-ramp){
            msleep(delay);
        }
        else if(current_r >= r-ramp){ /*v = -c*current_r² + d */
            delay = dr/( -c*current_r*current_r + d);
            msleep(delay);
        }
    }
    /* end of mess */
    }
}

int main()
{
    int i1;
    double j;
    double dummy = 70;
    /* different positions to play with */
    struct Pos start;
    struct Pos stop;
    struct Pos bottom1;
    struct Pos bottom2;
    struct Pos mid;
    struct Pos midlow;
    struct Pos midfront;
    struct Pos midhigh;
    struct Pos stopmid;
    start.x = -10; start.y = 30; start.z = 25;
    stop.x  = 10; stop.y  = 30; stop.z  = 25;
    bottom1.x = -10; bottom1.y = 30; bottom1.z = 5;
    bottom2.x = 10; bottom2.y = 30; bottom2.z = 5;
    mid.x = 0; mid.y=25; mid.z=20;
    midlow.x=0; midlow.y=20; midlow.z=10;
    midfront.x=0; midfront.y=35; midfront.z=10;
    midhigh.x=0; midhigh.y=20; midhigh.z=30;
    stopmid.x=10; stopmid.y=30; stopmid.z=15;

    fd = serialport_init("/dev/ttyUSB0", 115200); /* open the serial port */
    sleep(2);

    inverseKinematics(0,20,10,t,angles);
    commandArduino(fd,angles);
    sleep(1);

    line(midlow,mid);
    line(mid,bottom1);
    line(bottom1,start);
    line(start,stop);
    line(stop,start);
    line(start,stop);
    line(stop,bottom2);
    line(bottom2,mid);

    /* no function yet to go from 1 orientation to another */
    for (j=0; j<=dummy; j++){
        eulerMatrix(0,0,(j/dummy)*pi/4,t);
        inverseKinematics(0,25,20,t,angles);
        commandArduino(fd,angles);
        msleep(20);
    }

    for (j=-dummy; j<=dummy; j++){
        eulerMatrix(0,0,-(j/dummy)*pi/4,t);
        inverseKinematics(0,25,20,t,angles);
        commandArduino(fd,angles);
        msleep(20);
    }

    for (j=dummy; j>=0; j--){
        eulerMatrix(0,0,-(j/dummy)*pi/4,t);
        inverseKinematics(0,25,20,t,angles);
        commandArduino(fd,angles);
        msleep(20);
    }

    for (j=0; j>=-dummy; j--){
        eulerMatrix((j/dummy)*pi/4,0,0,t);
        inverseKinematics(0,25,20,t,angles);
        commandArduino(fd,angles);
        msleep(20);
    }

    for (j=-dummy; j<=dummy; j++){
        eulerMatrix((j/dummy)*pi/4,0,0,t);
        inverseKinematics(0,25,20,t,angles);
        commandArduino(fd,angles);
        msleep(20);
    }

    for (j=0; j<=dummy; j++){
        eulerMatrix(cos((j/dummy)*pi)*pi/4,0,sin((j/dummy)*pi)*pi/4,   t);
        inverseKinematics(0,25,20,t,angles);
        commandArduino(fd,angles);
        msleep(50);
    }

    for (j=0; j<=dummy; j++){
        eulerMatrix(-cos((j/dummy)*pi)*pi/4,0,-sin((j/dummy)*pi)*pi/4,   t);
        inverseKinematics(0,25,20,t,angles);
        commandArduino(fd,angles);
        msleep(50);
    }
    msleep(500);
    for (j=dummy; j>=0; j--){
        eulerMatrix((j/dummy)*pi/4,0,0,t);
        inverseKinematics(0,25,20,t,angles);
        commandArduino(fd,angles);
        msleep(20);
    }

    line(mid,midlow);
    line(midlow,midfront);
    line(midfront,midlow);
    line(midlow,midhigh);
    line(midhigh,midlow);

    sleep(2);
    serialport_close(fd);
}
