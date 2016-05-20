/* Regression testing program for the orbital motion C-Library.
*
* Author:         Hanspeter Schaub
* Date;           June 19, 2005
* Organization:   Virginia Tech, Blacksburg, VA 24091
*/

#include <stdio.h>
#include <stdlib.h>
#include "orbitalMotion.h"
#include "astroConstants.h"
#include "vector3D.h"
#include "RigidBodyKinematics.h"

int round_double(double, double, int);

int error_count = 0;

void testRigidBodyKinematics(void);


int round_double(double v1, double v2, int num)
{
    int error = 0;
    double v;

    v = fabs(v1 - v2);

    if (v > fabs(v2)*pow(10.,-num)) error = 1;

    return error;
}

void testRigidBodyKinematics(void)
{
    double C[4][4], C2[4][4], v1[5], v2[5], v3[5], om[4], a;

    // set4(0.45226701686665, 0.75377836144441, 0.15075567228888, 0.45226701686665, v1);
    // set4(-0.18663083698528, 0.46657709246321, 0.83983876643378, -0.20529392068381, v2);
    // addEP(v1,v2,v3);
    // set4(-0.46986547690254, -0.34044145332460, 0.71745926113861, 0.38545850500388, v1);
    // if (!(V4equalCheck(v3, v1, 10))) {
    //     printf("addEP failed\n");
    //     printVector4("answer", v3);
    //     error_count++;
    // }

    // set3(10*D2R, 20*D2R, 30*D2R, v1);
    // set3(-30*D2R, 200*D2R, 81*D2R, v2);
    // addEuler121(v1,v2,v3);
    // set3(-2.96705972839036, 2.44346095279206, 1.41371669411541,v1);
    // if (!(VequalCheck(v3, v1, 10))) {
    //     printf("addEuler121 failed\n");
    //     printVector("answer", v3);
    //     error_count++;
    // }

    // set3(10*D2R, 20*D2R, 30*D2R, v1);
    // set3(-30*D2R, 200*D2R, 81*D2R, v2);
    // addEuler123(v1,v2,v3);
    // set3(2.65556257351773, -0.34257634487528, -2.38843896474589,v1);
    // if (!(VequalCheck(v3, v1, 10))) {
    //     printf("addEuler123 failed\n");
    //     printVector("answer", v3);
    //     error_count++;
    // }

    // set3(10*D2R, 20*D2R, 30*D2R, v1);
    // set3(-30*D2R, 200*D2R, 81*D2R, v2);
    // addEuler131(v1,v2,v3);
    // set3(-2.96705972839036, 2.44346095279206, 1.41371669411541,v1);
    // if (!(VequalCheck(v3, v1, 10))) {
    //     printf("addEuler123 failed\n");
    //     printVector("answer", v3);
    //     error_count++;
    // }

    // set3(10*D2R, 20*D2R, 30*D2R, v1);
    // set3(-30*D2R, 200*D2R, 81*D2R, v2);
    // addEuler132(v1,v2,v3);
    // set3(2.93168877067466, -0.89056295435594, -2.11231276758895,v1);
    // if (!(VequalCheck(v3, v1, 10))) {
    //     printf("addEuler132 failed\n");
    //     printVector("answer", v3);
    //     error_count++;
    // }

    // set3(10*D2R, 20*D2R, 30*D2R, v1);
    // set3(-30*D2R, 200*D2R, 81*D2R, v2);
    // addEuler212(v1,v2,v3);
    // set3(-2.96705972839036,   2.44346095279206,   1.41371669411541,v1);
    // if (!(VequalCheck(v3, v1, 10))) {
    //     printf("addEuler212 failed\n");
    //     printVector("answer", v3);
    //     error_count++;
    // }

    // set3(10*D2R, 20*D2R, 30*D2R, v1);
    // set3(-30*D2R, 200*D2R, 81*D2R, v2);
    // addEuler213(v1,v2,v3);
    // set3(2.93168877067466,  -0.89056295435594,  -2.11231276758895,v1);
    // if (!(VequalCheck(v3, v1, 10))) {
    //     printf("addEuler213 failed\n");
    //     printVector("answer", v3);
    //     error_count++;
    // }

    // set3(10*D2R, 20*D2R, 30*D2R, v1);
    // set3(-30*D2R, 200*D2R, 81*D2R, v2);
    // addEuler231(v1,v2,v3);
    // set3(2.65556257351773,  -0.34257634487528,  -2.38843896474589,v1);
    // if (!(VequalCheck(v3, v1, 10))) {
    //     printf("addEuler231 failed\n");
    //     printVector("answer", v3);
    //     error_count++;
    // }

    // set3(10*D2R, 20*D2R, 30*D2R, v1);
    // set3(-30*D2R, 200*D2R, 81*D2R, v2);
    // addEuler232(v1,v2,v3);
    // set3(-2.96705972839036,   2.44346095279206,   1.41371669411541,v1);
    // if (!(VequalCheck(v3, v1, 10))) {
    //     printf("addEuler232 failed\n");
    //     printVector("answer", v3);
    //     error_count++;
    // }

    // set3(10*D2R, 20*D2R, 30*D2R, v1);
    // set3(-30*D2R, 200*D2R, 81*D2R, v2);
    // addEuler312(v1,v2,v3);
    // set3(2.65556257351773,  -0.34257634487528,  -2.38843896474589,v1);
    // if (!(VequalCheck(v3, v1, 10))) {
    //     printf("addEuler312 failed\n");
    //     printVector("answer", v3);
    //     error_count++;
    // }

    // set3(10*D2R, 20*D2R, 30*D2R, v1);
    // set3(-30*D2R, 200*D2R, 81*D2R, v2);
    // addEuler313(v1,v2,v3);
    // set3(-2.96705972839036,   2.44346095279206,   1.41371669411541,v1);
    // if (!(VequalCheck(v3, v1, 10))) {
    //     printf("addEuler313 failed\n");
    //     printVector("answer", v3);
    //     error_count++;
    // }

    // set3(10*D2R, 20*D2R, 30*D2R, v1);
    // set3(-30*D2R, 200*D2R, 81*D2R, v2);
    // addEuler321(v1,v2,v3);
    // set3(2.93168877067466,  -0.89056295435594,  -2.11231276758895,v1);
    // if (!(VequalCheck(v3, v1, 10))) {
    //     printf("addEuler321 failed\n");
    //     printVector("answer", v3);
    //     error_count++;
    // }

    // set3(10*D2R, 20*D2R, 30*D2R, v1);
    // set3(-30*D2R, 200*D2R, 81*D2R, v2);
    // addEuler323(v1,v2,v3);
    // set3(-2.96705972839036,   2.44346095279206,   1.41371669411541,v1);
    // if (!(VequalCheck(v3, v1, 10))) {
    //     printf("addEuler323 failed\n");
    //     printVector("answer", v3);
    //     error_count++;
    // }

    // set3( 1.5, 0.5, 0.5, v1);
    // set3(-0.5, 0.25, 0.15, v2);
    // addGibbs(v1, v2, v3);
    // set3(0.61290322580645,   0.17741935483871,   0.82258064516129, v1);
    // if (!(VequalCheck(v3, v1, 10))) {
    //     printf("addGibbs failed\n");
    //     printVector("answer", v3);
    //     error_count++;
    // }

    // set3( 1.5, 0.5, 0.5, v1);
    // set3(-0.5, 0.25, 0.15, v2);
    // addMRP(v1, v2, v3);
    // set3(0.58667769962764,  -0.34919321472900,   0.43690525444766, v1);
    // if (!(VequalCheck(v3, v1, 10))) {
    //     printf("addMRP failed\n");
    //     printVector("answer", v3);
    //     error_count++;
    // }

    // set3( 1.5, 0.5, 0.5, v1);
    // set3(-0.5, 0.25, 0.15, v2);
    // addPRV(v1, v2, v3);
    // set3(1.00227389370983,   0.41720669426711,   0.86837149207759, v1);
    // if (!(VequalCheck(v3, v1, 10))) {
    //     printf("addPRV failed\n");
    //     printVector("answer", v3);
    //     error_count++;
    // }

    // set3(30*D2R, -40*D2R, 15*D2R, v1);
    // BinvEuler121(v1,C);
    // set3(0.76604444311898, 0.0, 1.0, C2[1]);
    // set3(-0.16636567534280, 0.96592582628907, 0., C2[2]);
    // set3(-0.62088515301485, -0.25881904510252, 0., C2[3]);
    // if (!(MequalCheck(C,C2, 10))) {
    //     printf("BinvEuler121 failed\n");
    //     printMatrix("answer", C2);
    //     error_count++;
    // }

    // set3(30*D2R, -40*D2R, 15*D2R, v1);
    // BinvEuler123(v1,C);
    // set3(0.73994211169385,   0.25881904510252,                  0, C2[1]);
    // set3(-0.19826689127415,   0.96592582628907,                  0, C2[2]);
    // set3(-0.64278760968654,                  0,   1.00000000000000, C2[3]);
    // if (!(MequalCheck(C, C2, 10))) {
    //     printf("BinvEuler123 failed\n");
    //     printMatrix("answer", C);
    //     error_count++;
    // }

    // set3(30*D2R, -40*D2R, 15*D2R, v1);
    // BinvEuler131(v1,C);
    // set3(0.76604444311898,                  0,   1.00000000000000, C2[1]);
    // set3(0.62088515301485,   0.25881904510252,                  0, C2[2]);
    // set3(-0.16636567534280,   0.96592582628907,                  0, C2[3]);
    // if (!(MequalCheck(C, C2, 10))) {
    //     printf("BinvEuler131 failed\n");
    //     printMatrix("answer", C);
    //     error_count++;
    // }

    // set3(30*D2R, -40*D2R, 15*D2R, v1);
    // BinvEuler132(v1,C);
    // set3(0.73994211169385,  -0.25881904510252,                  0, C2[1]);
    // set3(0.64278760968654,                  0,   1.00000000000000, C2[2]);
    // set3(0.19826689127415,   0.96592582628907,                  0, C2[3]);
    // if (!(MequalCheck(C, C2, 10))) {
    //     printf("BinvEuler132 failed\n");
    //     printMatrix("answer", C);
    //     error_count++;
    // }

    // set3(30*D2R, -40*D2R, 15*D2R, v1);
    // BinvEuler212(v1,C);
    // set3(-0.16636567534280,   0.96592582628907,                  0, C2[1]);
    // set3(0.76604444311898,                  0,   1.00000000000000, C2[2]);
    // set3(0.62088515301485,   0.25881904510252,                  0, C2[3]);
    // if (!(MequalCheck(C, C2, 10))) {
    //     printf("BinvEuler212 failed\n");
    //     printMatrix("answer", C);
    //     error_count++;
    // }

    // set3(30*D2R, -40*D2R, 15*D2R, v1);
    // BinvEuler213(v1,C);
    // set3(0.19826689127415,   0.96592582628907,                  0, C2[1]);
    // set3(0.73994211169385,  -0.25881904510252,                  0, C2[2]);
    // set3(0.64278760968654,                  0,   1.00000000000000, C2[3]);
    // if (!(MequalCheck(C, C2, 10))) {
    //     printf("BinvEuler213 failed\n");
    //     printMatrix("answer", C);
    //     error_count++;
    // }

    // set3(30*D2R, -40*D2R, 15*D2R, v1);
    // BinvEuler231(v1,C);
    // set3(-0.64278760968654,                  0,   1.00000000000000, C2[1]);
    // set3(0.73994211169385,   0.25881904510252,                  0, C2[2]);
    // set3(-0.19826689127415,   0.96592582628907,                  0, C2[3]);
    // if (!(MequalCheck(C, C2, 10))) {
    //     printf("BinvEuler231 failed\n");
    //     printMatrix("answer", C);
    //     error_count++;
    // }

    // set3(30*D2R, -40*D2R, 15*D2R, v1);
    // BinvEuler232(v1,C);
    // set3(-0.62088515301485,  -0.25881904510252,                  0, C2[1]);
    // set3(0.76604444311898,                  0,   1.00000000000000, C2[2]);
    // set3(-0.16636567534280,   0.96592582628907,                  0, C2[3]);
    // if (!(MequalCheck(C, C2, 10))) {
    //     printf("BinvEuler232 failed\n");
    //     printMatrix("answer", C);
    //     error_count++;
    // }

    // set3(30*D2R, -40*D2R, 15*D2R, v1);
    // BinvEuler312(v1,C);
    // set3(-0.19826689127415,   0.96592582628907,                  0, C2[1]);
    // set3(-0.64278760968654,                  0,   1.00000000000000, C2[2]);
    // set3(0.73994211169385,   0.25881904510252,                  0, C2[3]);
    // if (!(MequalCheck(C, C2, 10))) {
    //     printf("BinvEuler312 failed\n");
    //     printMatrix("answer", C);
    //     error_count++;
    // }

    // set3(30*D2R, -40*D2R, 15*D2R, v1);
    // BinvEuler313(v1,C);
    // set3(-0.16636567534280,   0.96592582628907,                  0, C2[1]);
    // set3(-0.62088515301485,  -0.25881904510252,                  0, C2[2]);
    // set3(0.76604444311898,                  0,   1.00000000000000, C2[3]);
    // if (!(MequalCheck(C, C2, 10))) {
    //     printf("BinvEuler313 failed\n");
    //     printMatrix("answer", C);
    //     error_count++;
    // }

    // set3(30*D2R, -40*D2R, 15*D2R, v1);
    // BinvEuler321(v1,C);
    // set3(0.64278760968654,                  0,   1.00000000000000, C2[1]);
    // set3(0.19826689127415,   0.96592582628907,                  0, C2[2]);
    // set3(0.73994211169385,  -0.25881904510252,                  0, C2[3]);
    // if (!(MequalCheck(C, C2, 10))) {
    //     printf("BinvEuler321 failed\n");
    //     printMatrix("answer", C);
    //     error_count++;
    // }

    // set3(30*D2R, -40*D2R, 15*D2R, v1);
    // BinvEuler323(v1,C);
    // set3(0.62088515301485,   0.25881904510252,                  0, C2[1]);
    // set3(-0.16636567534280,   0.96592582628907,                  0, C2[2]);
    // set3(0.76604444311898,                  0,   1.00000000000000, C2[3]);
    // if (!(MequalCheck(C, C2, 10))) {
    //     printf("BinvEuler323 failed\n");
    //     printMatrix("answer", C);
    //     error_count++;
    // }

    set3(0.25,0.5,-0.5, v1);
    BinvGibbs(v1,C);
    set3(0.64,-0.32,-0.32, C2[1]);
    set3(0.32, 0.64, 0.16, C2[2]);
    set3(0.32, -0.16, 0.64, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BinvGibbs failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(0.25,0.5,-0.5, v1);
    BinvMRP(v1,C);
    set3(0.2304, -0.3072, -0.512, C2[1]);
    set3(0.512, 0.384, 0, C2[2]);
    set3(0.3072, -0.4096, 0.3840, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BinvMRP failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(0.25,0.5,-0.5, v1);
    BinvPRV(v1,C);
    set3(0.91897927113877,  -0.21824360100796,  -0.25875396543858, C2[1]);
    set3(0.25875396543858,   0.94936204446173,   0.07873902718102, C2[2]);
    set3(0.21824360100796,  -0.15975975604225,   0.94936204446173, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BinvPRV failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(30*D2R, -40*D2R, 15*D2R, v1);
    BmatEuler121(v1,C);
    set3(0,  -0.40265095531125,  -1.50271382293774, C2[1]);
    set3(0,   0.96592582628907,  -0.25881904510252, C2[2]);
    set3(1.00000000000000,   0.30844852683273,   1.15114557365953, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BmatEuler121 failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(30*D2R, -40*D2R, 15*D2R, v1);
    BmatEuler123(v1,C);
    set3(1.26092661459205,  -0.33786426809485,                  0, C2[1]);
    set3(0.25881904510252,   0.96592582628907,                  0, C2[2]);
    set3(0.81050800458377,  -0.21717496528718,   1.00000000000000, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BmatEuler123 failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(30*D2R, -40*D2R, 15*D2R, v1);
    BmatEuler131(v1,C);
    set3(0,   1.50271382293774,  -0.40265095531125, C2[1]);
    set3(0,   0.25881904510252,   0.96592582628907, C2[2]);
    set3(1.00000000000000,  -1.15114557365953,   0.30844852683273, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BmatEuler131 failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(30*D2R, -40*D2R, 15*D2R, v1);
    BmatEuler132(v1,C);
    set3(1.26092661459205,                  0,   0.33786426809485, C2[1]);
    set3(-0.25881904510252,                  0,   0.96592582628907, C2[2]);
    set3(-0.81050800458377,   1.00000000000000,  -0.21717496528718, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BmatEuler132 failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(30*D2R, -40*D2R, 15*D2R, v1);
    BmatEuler212(v1,C);
    set3(-0.40265095531125,                  0,   1.50271382293774, C2[1]);
    set3(0.96592582628907,                  0,   0.25881904510252, C2[2]);
    set3(0.30844852683273,   1.00000000000000,  -1.15114557365953, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BmatEuler212 failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(30*D2R, -40*D2R, 15*D2R, v1);
    BmatEuler213(v1,C);
    set3(0.33786426809485,   1.26092661459205,                  0, C2[1]);
    set3(0.96592582628907,  -0.25881904510252,                  0, C2[2]);
    set3(-0.21717496528718,  -0.81050800458377,   1.00000000000000, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BmatEuler213 failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(30*D2R, -40*D2R, 15*D2R, v1);
    BmatEuler231(v1,C);
    set3(0,   1.26092661459205,  -0.33786426809485, C2[1]);
    set3(0,   0.25881904510252,   0.96592582628907, C2[2]);
    set3(1.00000000000000,   0.81050800458377,  -0.21717496528718, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BmatEuler231 failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(30*D2R, -40*D2R, 15*D2R, v1);
    BmatEuler232(v1,C);
    set3(-1.50271382293774,                  0,  -0.40265095531125, C2[1]);
    set3(-0.25881904510252,                  0,   0.96592582628907, C2[2]);
    set3( 1.15114557365953,   1.00000000000000,   0.30844852683273, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BmatEuler232 failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(30*D2R, -40*D2R, 15*D2R, v1);
    BmatEuler312(v1,C);
    set3(-0.33786426809485,                  0,   1.26092661459205, C2[1]);
    set3(0.96592582628907,                  0,   0.25881904510252, C2[2]);
    set3(-0.21717496528718,   1.00000000000000,   0.81050800458377, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BmatEuler312 failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(30*D2R, -40*D2R, 15*D2R, v1);
    BmatEuler313(v1,C);
    set3(-0.40265095531125,  -1.50271382293774,                  0, C2[1]);
    set3(0.96592582628907,  -0.25881904510252,                  0, C2[2]);
    set3(0.30844852683273,   1.15114557365953,   1.00000000000000, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BmatEuler313 failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(30*D2R, -40*D2R, 15*D2R, v1);
    BmatEuler321(v1,C);
    set3(0,   0.33786426809485,   1.26092661459205, C2[1]);
    set3(0,   0.96592582628907,  -0.25881904510252, C2[2]);
    set3(1.00000000000000,  -0.21717496528718,  -0.81050800458377, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BmatEuler321 failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(30*D2R, -40*D2R, 15*D2R, v1);
    BmatEuler323(v1,C);
    set3(1.50271382293774,  -0.40265095531125,                  0, C2[1]);
    set3(0.25881904510252,   0.96592582628907,                  0, C2[2]);
    set3(-1.15114557365953,   0.30844852683273,   1.00000000000000, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BmatEuler323 failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(0.25,0.5,-0.5, v1);
    BmatGibbs(v1,C);
    set3(1.06250000000000,   0.62500000000000,   0.37500000000000, C2[1]);
    set3(-0.37500000000000,   1.25000000000000,  -0.50000000000000, C2[2]);
    set3(-0.62500000000000,                  0,   1.25000000000000, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BmatGibbs failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(0.25,0.5,-0.5, v1);
    BmatMRP(v1,C);
    set3(0.56250000000000,   1.25000000000000,   0.75000000000000, C2[1]);
    set3(-0.75000000000000,   0.93750000000000,  -1.00000000000000, C2[2]);
    set3(-1.25000000000000,                  0,   0.93750000000000, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BmatMRP failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(0.25,0.5,-0.5, v1);
    BmatPRV(v1,C);
    set3(0.95793740211924,   0.26051564947019,   0.23948435052981, C2[1]);
    set3(-0.23948435052981,   0.97371087632453,  -0.14603129894038, C2[2]);
    set3(-0.26051564947019,   0.10396870105962,   0.97371087632453, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("BmatPRV failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(-0.506611258027956, -0.05213449187759728, 0.860596902153381, C[1]);
    set3(-0.7789950887797505, -0.4000755572346052, -0.4828107291273137, C[2]);
    set3(0.3694748772194938, -0.9149981110691346, 0.1620702682281828, C[3]);
    C2EP(C,v1);
    set4(0.2526773896521122, 0.4276078901804977, -0.4859180570232927, \
         0.7191587243944733, v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2EP failed\n");
        printVector4("answer", v1);
        error_count++;
    }

    set3(-0.506611258027956, -0.05213449187759728, 0.860596902153381, C[1]);
    set3(-0.7789950887797505, -0.4000755572346052, -0.4828107291273137, C[2]);
    set3(0.3694748772194938, -0.9149981110691346, 0.1620702682281828, C[3]);
    C2Euler121(C,v1);
    set3(-3.081087141428621, 2.102046098550739, -1.127921895439695,  v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2Euler121 failed\n");
        printVector("answer", v1);
        error_count++;
    }
    C2Euler123(C,v1);
    set3(1.395488250243478, 0.3784438476398376, 2.147410157986089,  v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2Euler123 failed\n");
        printVector("answer", v1);
        error_count++;
    }
    C2Euler131(C,v1);
    set3(1.631301838956069, 2.102046098550739, 0.4428744313552013,  v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2Euler131 failed\n");
        printVector("answer", v1);
        error_count++;
    }
    C2Euler132(C,v1);
    set3(-2.262757475208626, 0.8930615653924096, 2.511467464302149,  v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2Euler132 failed\n");
        printVector("answer", v1);
        error_count++;
    }
    C2Euler212(C,v1);
    set3(-2.125637903992466, 1.982395614047245, -0.05691616561213509,  v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2Euler212 failed\n");
        printVector("answer", v1);
        error_count++;
    }
    C2Euler213(C,v1);
    set3(1.157420789791818, 1.155503238813826, -3.012011225795042,  v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2Euler213 failed\n");
        printVector("answer", v1);
        error_count++;
    }
    C2Euler231(C,v1);
    set3(-2.102846464319881, -0.05215813778076988, 1.982990154077466,  v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2Euler231 failed\n");
        printVector("answer", v1);
        error_count++;
    }
    C2Euler232(C,v1);
    set3(-0.5548415771975691, 1.982395614047245, -1.627712492407032,  v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2Euler232 failed\n");
        printVector("answer", v1);
        error_count++;
    }
    C2Euler312(C,v1);
    set3(2.045248068737305, -0.5038614866151004, -1.384653359078797,  v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2Euler312 failed\n");
        printVector("answer", v1);
        error_count++;
    }
    C2Euler313(C,v1);
    set3(0.3837766626244829, 1.408008028147626, 2.082059614484753,  v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2Euler313 failed\n");
        printVector("answer", v1);
        error_count++;
    }
    C2Euler321(C,v1);
    set3(-3.039045355374235, -1.036440549977791, -1.246934586231547,  v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2Euler321 failed\n");
        printVector("answer", v1);
        error_count++;
    }
    C2Euler323(C,v1);
    set3(-1.187019664170414, 1.408008028147626, -2.630329365899936,  v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2Euler323 failed\n");
        printVector("answer", v1);
        error_count++;
    }

    set3(0.25,0.5,-0.5, v1);
    C2Gibbs(C,v1);
    set3(1.692307692307693, -1.923076923076923, 2.846153846153846,  v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2Gibbs failed\n");
        printVector("answer", v1);
        error_count++;
    }
    C2MRP(C,v1);
    set3(0.3413551595269481, -0.3879035903715318, 0.5740973137498672,  v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2MRP failed\n");
        printVector("answer", v1);
        error_count++;
    }
    C2PRV(C,v1);
    set3(1.162634795241009, -1.321175903682964, 1.955340337450788,  v2);
    if (!(VequalCheck(v1, v2, 10))) {
        printf("C2PRV failed\n");
        printVector("answer", v1);
        error_count++;
    }

    set4(0.2526773896521122, 0.4276078901804977, -0.4859180570232927, 0.7191587243944733, v1);
    set3(0.2,0.1,-0.5,om);
    dEP(v1,om,v3);
    set4(0.1613247949317332, 0.1107893170013107, 0.1914517144671774, 0.006802852798326098,v1);
    if (!(V4equalCheck(v1, v3, 10))) {
        printf("dEP failed\n");
        printVector4("answer", v3);
        error_count++;
    }

    set3(30*D2R, -40*D2R, 15*D2R, v1);
    set3(0.2,0.1,-0.5,om);
    dEuler121(v1,om,v3);
    set3(0.7110918159377425, 0.2260021051801672, -0.3447279341464908, v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("dEuler121 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    dEuler123(v1,om,v3);
    set3(0.2183988961089258, 0.148356391649411, -0.3596158956119647, v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("dEuler123 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    dEuler131(v1,om,v3);
    set3(0.3515968599493992, -0.4570810086342821, -0.06933882078231876, v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("dEuler131 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    dEuler132(v1,om,v3);
    set3(0.08325318887098565, -0.5347267221650382, 0.04648588172683711, v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("dEuler132 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    dEuler212(v1,om,v3);
    set3(-0.8318871025311179, 0.06377564270655334, 0.7372624921963103, v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("dEuler212 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    dEuler213(v1,om,v3);
    set3(0.1936655150781755, 0.1673032607475616, -0.6244857935158128, v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("dEuler213 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    dEuler231(v1,om,v3);
    set3(0.2950247955066306, -0.4570810086342821, 0.3896382831019671, v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("dEuler231 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    dEuler232(v1,om,v3);
    set3(-0.09921728693192147, -0.5347267221650384, 0.1760048513155397, v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("dEuler232 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    dEuler312(v1,om,v3);
    set3(-0.6980361609149971, 0.06377564270655331, -0.3486889953493196, v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("dEuler312 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    dEuler313(v1,om,v3);
    set3(-0.2308015733560238, 0.1673032607475616, -0.3231957372675008, v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("dEuler312 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    dEuler321(v1,om,v3);
    set3(-0.596676880486542, 0.2260021051801672, 0.5835365057631652, v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("dEuler312 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    dEuler323(v1,om,v3);
    set3(0.260277669056422, 0.148356391649411, -0.6993842620486324, v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("dEuler312 failed\n");
        printVector("answer", v3);
        error_count++;
    }

    dGibbs(v1,om,v3);
    set3(0.236312018677072, 0.2405875488560276, -0.1665723597065136, v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("dGibbs failed\n");
        printVector("answer", v3);
        error_count++;
    }
    dMRP(v1,om,v3);
    set3(0.144807895231133, 0.1948354871330581, 0.062187948908334, v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("dMRP failed\n");
        printVector("answer", v3);
        error_count++;
    }
    dPRV(v1,om,v3);
    set3(0.34316538031149, 0.255728121815202, -0.3710557691157747, v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("dPRV failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set4(0.9110886174894189, 0.5746957711326909, -0.7662610281769212, 0.2873478855663454, v1);
    elem2PRV(v1,v3);
    set3(0.5235987755982988, -0.6981317007977318, 0.2617993877991494 ,v2);
    if (!(VequalCheck(v3, v2, 10))) {
        printf("elem2PRV failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set4(0.2526773896521122, 0.4276078901804977, -0.4859180570232927, 0.7191587243944733 , v1);
    EP2C(v1,C);
    set3(-0.506611258027956, -0.05213449187759728, 0.860596902153381, C2[1]);
    set3(-0.7789950887797505, -0.4000755572346052, -0.4828107291273137, C2[2]);
    set3(0.3694748772194938, -0.9149981110691346, 0.1620702682281828, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("EP2C failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    EP2Euler121(v1,v2);
    set3(3.202098165750965, 2.102046098550739, -1.127921895439695, v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("EP2Euler121 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    EP2Euler123(v1,v2);
    set3(1.395488250243478, 0.3784438476398376, 2.147410157986089, v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("EP2Euler123 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    EP2Euler131(v1,v2);
    set3(1.631301838956069, 2.102046098550739, 0.4428744313552013, v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("EP2Euler131 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    EP2Euler132(v1,v2);
    set3(-2.262757475208626, 0.8930615653924096, 2.511467464302149, v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("EP2Euler132 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    EP2Euler212(v1,v2);
    set3(-2.125637903992466, 1.982395614047245, -0.05691616561213508, v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("EP2Euler212 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    EP2Euler213(v1,v2);
    set3(1.157420789791818, 1.155503238813826, -3.012011225795042, v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("EP2Euler213 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    EP2Euler231(v1,v2);
    set3(-2.102846464319881, -0.05215813778076988, 1.982990154077466, v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("EP2Euler231 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    EP2Euler232(v1,v2);
    set3(-0.5548415771975691, 1.982395614047245, -1.627712492407032, v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("EP2Euler232 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    EP2Euler312(v1,v2);
    set3(2.045248068737305, -0.5038614866151004, -1.384653359078797, v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("EP2Euler312 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    EP2Euler313(v1,v2);
    set3(0.3837766626244828, 1.408008028147627, 2.082059614484753, v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("EP2Euler313 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    EP2Euler321(v1,v2);
    set3(-3.039045355374235, -1.036440549977791, -1.246934586231547, v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("EP2Euler321 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    EP2Euler323(v1,v2);
    set3(-1.187019664170414, 1.408008028147627, 3.65285594127965, v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("EP2Euler323 failed\n");
        printVector("answer", v3);
        error_count++;
    }
    EP2Gibbs(v1,v2);
    set3(1.692307692307693, -1.923076923076923, 2.846153846153846, v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("EP2Gibbs failed\n");
        printVector("answer", v3);
        error_count++;
    }
    EP2MRP(v1,v2);
    set3(0.3413551595269481, -0.3879035903715319, 0.5740973137498672, v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("EP2MRP failed\n");
        printVector("answer", v3);
        error_count++;
    }
    EP2PRV(v1,v2);
    set3(1.162634795241009, -1.321175903682965, 1.955340337450788, v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("EP2PRV failed\n");
        printVector("answer", v3);
        error_count++;
    }

    Euler1(1.3,C);
    set3(1, 0, 0, C2[1]);
    set3(0, 0.2674988286245874, 0.963558185417193, C2[2]);
    set3(0, -0.963558185417193, 0.2674988286245874, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler1 failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    Euler2(1.3,C);
    set3(0.2674988286245874, 0, -0.963558185417193, C2[1]);
    set3(0, 1, 0, C2[2]);
    set3(0.963558185417193, 0, 0.2674988286245874, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler2 failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    Euler3(1.3,C);
    set3(0.2674988286245874, 0.963558185417193, 0, C2[1]);
    set3(-0.963558185417193, 0.2674988286245874, 0, C2[2]);
    set3(0, 0, 1, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler3 failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(0.5746957711326909, -0.7662610281769212, 0.2873478855663454 ,v1);
    Euler1212C(v1, C);
    set3(0.7205084754311385, -0.3769430728235922, 0.5820493593177511, C2[1]);
    set3(-0.1965294640304305, 0.6939446195986547, 0.692688266609151, C2[2]);
    set3(-0.6650140649638986, -0.6134776155495705, 0.4259125598286639, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler1212C failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    Euler1212EP(v1, v2);
    set4(0.8426692196316502, 0.3875084824890354, -0.3699741829975614, -0.05352444488005169 ,v3);
    if (!(V4equalCheck(v2, v3, 10))) {
        printf("Euler1212EP failed\n");
        printVector4("answer", v2);
        error_count++;
    }
    Euler1212Gibbs(v1, v2);
    set3(0.4598583565902931, -0.4390503110571495, -0.06351774057138154 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler1212Gibbs failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler1212MRP(v1, v2);
    set3(0.2102973655610845, -0.2007816590497557, -0.02904723447366817 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler1212MRP failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler1212PRV(v1, v2);
    set3(0.8184049632304388, -0.7813731087574279, -0.1130418386266624 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler1212PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }

    set3(0.5746957711326909, -0.7662610281769212, 0.2873478855663454 ,v1);
    Euler1232C(v1, C);
    set3(0.6909668228739537, -0.1236057418710468, 0.7122404581768593, C2[1]);
    set3(-0.2041991989591971, 0.9117724894309838, 0.3563335721781613, C2[2]);
    set3(-0.6934461311680212, -0.391653607277317, 0.6047643467291773, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler1232C failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    Euler1232EP(v1, v2);
    set4(0.8954752451958283, 0.2088240806958052, -0.3924414987701519, 0.02250019124496444 ,v3);
    if (!(V4equalCheck(v2, v3, 10))) {
        printf("Euler1232EP failed\n");
        printVector4("answer", v2);
        error_count++;
    }
    Euler1232Gibbs(v1, v2);
    set3(0.2331991663824702, -0.4382494109977661, 0.02512653628972619 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler1232Gibbs failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler1232MRP(v1, v2);
    set3(0.1101697746911123, -0.2070412155288303, 0.01187047485953311 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler1232MRP failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler1232PRV(v1, v2);
    set3(0.4328366663508259, -0.8134266388215754, 0.04663690000825693 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler1232PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }

    set3(0.5746957711326909, -0.7662610281769212, 0.2873478855663454 ,v1);
    Euler1312C(v1, C);
    set3(0.7205084754311385, -0.5820493593177511, -0.3769430728235922, C2[1]);
    set3(0.6650140649638986, 0.4259125598286639, 0.6134776155495705, C2[2]);
    set3(-0.1965294640304305, -0.692688266609151, 0.6939446195986547, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler1312C failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    Euler1312EP(v1, v2);
    set4(0.8426692196316502, 0.3875084824890354, 0.05352444488005169, -0.3699741829975614 ,v3);
    if (!(V4equalCheck(v2, v3, 10))) {
        printf("Euler1312EP failed\n");
        printVector4("answer", v2);
        error_count++;
    }
    Euler1312Gibbs(v1, v2);
    set3(0.4598583565902931, 0.06351774057138154, -0.4390503110571495 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler1312Gibbs failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler1312MRP(v1, v2);
    set3(0.2102973655610845, 0.02904723447366817, -0.2007816590497557 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler1312MRP failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler1312PRV(v1, v2);
    set3(0.8184049632304388, 0.1130418386266624, -0.7813731087574279 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler1312PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }

    set3(0.5746957711326909, -0.7662610281769212, 0.2873478855663454 ,v1);
    Euler1322C(v1, C);
    set3(0.6909668228739537, -0.404128912281835, -0.5993702294453531, C2[1]);
    set3(0.6934461311680212, 0.6047643467291773, 0.391653607277317, C2[2]);
    set3(0.2041991989591971, -0.6862506154337003, 0.6981137299618809, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler1322C failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    Euler1322EP(v1, v2);
    set4(0.8651365354042408, 0.3114838463640192, 0.2322088466732818, -0.3171681574333834 ,v3);
    if (!(V4equalCheck(v2, v3, 10))) {
        printf("Euler1322EP failed\n");
        printVector4("answer", v2);
        error_count++;
    }
    Euler1322Gibbs(v1, v2);
    set3(0.3600401018996109, 0.2684071671586273, -0.3666105226791566 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler1322Gibbs failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler1322MRP(v1, v2);
    set3(0.1670032410235906, 0.1244996504360223, -0.1700509058789317 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler1322MRP failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler1322PRV(v1, v2);
    set3(0.6525765328552258, 0.4864908592507521, -0.6644854907437873 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler1322PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }

    set3(0.5746957711326909, -0.7662610281769212, 0.2873478855663454 ,v1);
    Euler2122C(v1, C);
    set3(0.6939446195986547, -0.1965294640304305, -0.692688266609151, C2[1]);
    set3(-0.3769430728235922, 0.7205084754311385, -0.5820493593177511, C2[2]);
    set3(0.6134776155495705, 0.6650140649638986, 0.4259125598286639, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler2122C failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    Euler2122EP(v1, v2);
    set4(0.8426692196316502, -0.3699741829975614, 0.3875084824890354, 0.05352444488005169 ,v3);
    if (!(V4equalCheck(v2, v3, 10))) {
        printf("Euler2122EP failed\n");
        printVector4("answer", v2);
        error_count++;
    }
    Euler2122Gibbs(v1, v2);
    set3(-0.4390503110571495, 0.4598583565902931, 0.06351774057138154 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler2122Gibbs failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler2122MRP(v1, v2);
    set3(-0.2007816590497557, 0.2102973655610845, 0.02904723447366817 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler2122MRP failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler2122PRV(v1, v2);
    set3(-0.7813731087574279, 0.8184049632304388, 0.1130418386266624 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler2122PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }

    set3(0.5746957711326909, -0.7662610281769212, 0.2873478855663454 ,v1);
    Euler2132C(v1, C);
    set3(0.6981137299618809, 0.2041991989591971, -0.6862506154337003, C2[1]);
    set3(-0.5993702294453531, 0.6909668228739537, -0.404128912281835, C2[2]);
    set3(0.391653607277317, 0.6934461311680212, 0.6047643467291773, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler2132C failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    Euler2132EP(v1, v2);
    set4(0.8651365354042408, -0.3171681574333834, 0.3114838463640192, 0.2322088466732818 ,v3);
    if (!(V4equalCheck(v2, v3, 10))) {
        printf("Euler2132EP failed\n");
        printVector4("answer", v2);
        error_count++;
    }
    Euler2132Gibbs(v1, v2);
    set3(-0.3666105226791566, 0.3600401018996109, 0.2684071671586273 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler2132Gibbs failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler2132MRP(v1, v2);
    set3(-0.1700509058789317, 0.1670032410235906, 0.1244996504360223 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler2132MRP failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler2132PRV(v1, v2);
    set3(-0.6644854907437873, 0.6525765328552258, 0.4864908592507521 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler2132PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }

    set3(0.5746957711326909, -0.7662610281769212, 0.2873478855663454 ,v1);
    Euler2312C(v1, C);
    set3(0.6047643467291773, -0.6934461311680212, -0.391653607277317, C2[1]);
    set3(0.7122404581768593, 0.6909668228739537, -0.1236057418710468, C2[2]);
    set3(0.3563335721781613, -0.2041991989591971, 0.9117724894309838, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler2312C failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    Euler2312EP(v1, v2);
    set4(0.8954752451958283, 0.02250019124496444, 0.2088240806958052, -0.3924414987701519 ,v3);
    if (!(V4equalCheck(v2, v3, 10))) {
        printf("Euler2312EP failed\n");
        printVector4("answer", v2);
        error_count++;
    }
    Euler2312Gibbs(v1, v2);
    set3(0.02512653628972619, 0.2331991663824702, -0.4382494109977661 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler2312Gibbs failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler2312MRP(v1, v2);
    set3(0.01187047485953311, 0.1101697746911123, -0.2070412155288303 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler2312MRP failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler2312PRV(v1, v2);
    set3(0.04663690000825693, 0.4328366663508259, -0.8134266388215754 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler2312PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }

    set3(0.5746957711326909, -0.7662610281769212, 0.2873478855663454 ,v1);
    Euler2322C(v1, C);
    set3(0.4259125598286639, -0.6650140649638986, -0.6134776155495705, C2[1]);
    set3(0.5820493593177511, 0.7205084754311385, -0.3769430728235922, C2[2]);
    set3(0.692688266609151, -0.1965294640304305, 0.6939446195986547, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler2322C failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    Euler2322EP(v1, v2);
    set4(0.8426692196316502, -0.05352444488005169, 0.3875084824890354, -0.3699741829975614 ,v3);
    if (!(V4equalCheck(v2, v3, 10))) {
        printf("Euler2322EP failed\n");
        printVector4("answer", v2);
        error_count++;
    }
    Euler2322Gibbs(v1, v2);
    set3(-0.06351774057138154, 0.4598583565902931, -0.4390503110571495 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler2322Gibbs failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler2322MRP(v1, v2);
    set3(-0.02904723447366817, 0.2102973655610845, -0.2007816590497557 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler2322MRP failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler2322PRV(v1, v2);
    set3(-0.1130418386266624, 0.8184049632304388, -0.7813731087574279 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler2322PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }

    set3(0.5746957711326909, -0.7662610281769212, 0.2873478855663454 ,v1);
    Euler3122C(v1, C);
    set3(0.9117724894309838, 0.3563335721781613, -0.2041991989591971, C2[1]);
    set3(-0.391653607277317, 0.6047643467291773, -0.6934461311680212, C2[2]);
    set3(-0.1236057418710468, 0.7122404581768593, 0.6909668228739537, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler3122C failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    Euler3122EP(v1, v2);
    set4(0.8954752451958283, -0.3924414987701519, 0.02250019124496444, 0.2088240806958052 ,v3);
    if (!(V4equalCheck(v2, v3, 10))) {
        printf("Euler3122EP failed\n");
        printVector4("answer", v2);
        error_count++;
    }
    Euler3122Gibbs(v1, v2);
    set3(-0.4382494109977661, 0.02512653628972619, 0.2331991663824702 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler3122Gibbs failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler3122MRP(v1, v2);
    set3(-0.2070412155288303, 0.01187047485953311, 0.1101697746911123 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler3122MRP failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler3122PRV(v1, v2);
    set3(-0.8134266388215754, 0.04663690000825693, 0.4328366663508259 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler3122PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }

    set3(0.5746957711326909, -0.7662610281769212, 0.2873478855663454 ,v1);
    Euler3132C(v1, C);
    set3(0.6939446195986547, 0.692688266609151, -0.1965294640304305, C2[1]);
    set3(-0.6134776155495705, 0.4259125598286639, -0.6650140649638986, C2[2]);
    set3(-0.3769430728235922, 0.5820493593177511, 0.7205084754311385, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler3132C failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    Euler3132EP(v1, v2);
    set4(0.8426692196316502, -0.3699741829975614, -0.05352444488005169, 0.3875084824890354 ,v3);
    if (!(V4equalCheck(v2, v3, 10))) {
        printf("Euler3132EP failed\n");
        printVector4("answer", v2);
        error_count++;
    }
    Euler3132Gibbs(v1, v2);
    set3(-0.4390503110571495, -0.06351774057138154, 0.4598583565902931 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler3132Gibbs failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler3132MRP(v1, v2);
    set3(-0.2007816590497557, -0.02904723447366817, 0.2102973655610845 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler3132MRP failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler3132PRV(v1, v2);
    set3(-0.7813731087574279, -0.1130418386266624, 0.8184049632304388 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler3132PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }

    set3(0.5746957711326909, -0.7662610281769212, 0.2873478855663454 ,v1);
    Euler3212C(v1, C);
    set3(0.6047643467291773, 0.391653607277317, 0.6934461311680212, C2[1]);
    set3(-0.6862506154337003, 0.6981137299618809, 0.2041991989591971, C2[2]);
    set3(-0.404128912281835, -0.5993702294453531, 0.6909668228739537, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler3212C failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    Euler3212EP(v1, v2);
    set4(0.8651365354042408, 0.2322088466732818, -0.3171681574333834, 0.3114838463640192 ,v3);
    if (!(V4equalCheck(v2, v3, 10))) {
        printf("Euler3212EP failed\n");
        printVector4("answer", v2);
        error_count++;
    }
    Euler3212Gibbs(v1, v2);
    set3(0.2684071671586273, -0.3666105226791566, 0.3600401018996109 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler3212Gibbs failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler3212MRP(v1, v2);
    set3(0.1244996504360223, -0.1700509058789317, 0.1670032410235906 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler3212MRP failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler3212PRV(v1, v2);
    set3(0.4864908592507521, -0.6644854907437873, 0.6525765328552258 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler3212PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }

    set3(0.5746957711326909, -0.7662610281769212, 0.2873478855663454 ,v1);
    Euler3232C(v1, C);
    set3(0.4259125598286639, 0.6134776155495705, 0.6650140649638986, C2[1]);
    set3(-0.692688266609151, 0.6939446195986547, -0.1965294640304305, C2[2]);
    set3(-0.5820493593177511, -0.3769430728235922, 0.7205084754311385, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler3232C failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    Euler3232EP(v1, v2);
    set4(0.8426692196316502, 0.05352444488005169, -0.3699741829975614, 0.3875084824890354 ,v3);
    if (!(V4equalCheck(v2, v3, 10))) {
        printf("Euler3232EP failed\n");
        printVector4("answer", v2);
        error_count++;
    }
    Euler3232Gibbs(v1, v2);
    set3(0.06351774057138154, -0.4390503110571495, 0.4598583565902931 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler3232Gibbs failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler3232MRP(v1, v2);
    set3(0.02904723447366817, -0.2007816590497557, 0.2102973655610845 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler3232MRP failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Euler3232PRV(v1, v2);
    set3(0.1130418386266624, -0.7813731087574279, 0.8184049632304388 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Euler3232PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }

    set3(0.5746957711326909, -0.7662610281769212, 0.2873478855663454 ,v1);
    Gibbs2C(v1, C);
    set3(0.3302752293577981, -0.1530190869107189, 0.9313986428558203, C2[1]);
    set3(-0.7277148580434096, 0.5871559633027522, 0.3545122848941588, C2[2]);
    set3(-0.6011234134980221, -0.794879257371223, 0.08256880733944938, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Gibbs2C failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    Gibbs2EP(v1, v2);
    set4(0.7071067811865475, 0.4063712768871578, -0.5418283691828771, 0.2031856384435789 ,v3);
    if (!(V4equalCheck(v2, v3, 10))) {
        printf("Gibbs2EP failed\n");
        printVector4("answer", v2);
        error_count++;
    }
    Gibbs2Euler121(v1, v2);
    set3(3.304427597008361, 1.234201174364066, -2.26121636963008 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Gibbs2Euler121 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Gibbs2Euler123(v1, v2);
    set3(1.467291629150036, -0.6449061163953342, 1.144743256726005 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Gibbs2Euler123 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Gibbs2Euler131(v1, v2);
    set3(1.733631270213465, 1.234201174364066, -0.6904200428351842 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Gibbs2Euler131 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Gibbs2Euler132(v1, v2);
    set3(0.54319335066115, 0.8149843403384446, -1.068390851022488 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Gibbs2Euler132 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Gibbs2Euler212(v1, v2);
    set3(-1.117474807766432, 0.9432554204540935, -0.1901795897648197,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Gibbs2Euler212 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Gibbs2Euler213(v1, v2);
    set3(-1.434293025994105, 0.9188085603647974, -0.2549399408440935 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Gibbs2Euler213 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Gibbs2Euler231(v1, v2);
    set3(-1.230028192223063, -0.1536226209659692, 0.9345839026955233 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Gibbs2Euler231 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Gibbs2Euler232(v1, v2);
    set3(0.4533215190284649, 0.9432554204540935, -1.760975916559716 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Gibbs2Euler232 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Gibbs2Euler312(v1, v2);
    set3(0.8918931304028546, 0.3623924238788913, -1.482377127697951 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Gibbs2Euler312 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Gibbs2Euler313(v1, v2);
    set3(-0.6474859022891233, 1.488133410155628, 1.207104533714101 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Gibbs2Euler313 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Gibbs2Euler321(v1, v2);
    set3(-0.4338654111289937, -1.198236565236741, 1.341967642658489 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Gibbs2Euler321 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Gibbs2Euler323(v1, v2);
    set3(-2.21828222908402, 1.488133410155628, 2.777900860508998 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Gibbs2Euler321 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Gibbs2MRP(v1, v2);
    set3(0.2380467826416248, -0.3173957101888331, 0.1190233913208124 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Gibbs2MRP failed\n");
        printVector("answer", v2);
        error_count++;
    }
    Gibbs2PRV(v1, v2);
    set3(0.9027300063197914, -1.203640008426389, 0.4513650031598956 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("Gibbs2PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }

    set3(0.2, -0.25, 0.3 ,v1);
    MRP2C(v1, C);
    set3(0.1420873822677549, 0.4001248192538094, 0.9053790945330048, C2[1]);
    set3(-0.9626904702257736, 0.2686646537364468, 0.03234752493088797, C2[2]);
    set3(-0.2303003133666478, -0.876196001388834, 0.4233702077537369, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("MRP2C failed\n");
        printMatrix("answer", C);
        error_count++;
    }
    MRP2EP(v1, v2);
    set4(0.6771488469601677, 0.3354297693920336, -0.419287211740042, 0.5031446540880503 ,v3);
    if (!(V4equalCheck(v2, v3, 10))) {
        printf("MRP2EP failed\n");
        printVector4("answer", v2);
        error_count++;
    }
    MRP2Euler121(v1, v2);
    set3(2.725460144813494, 1.428226451915784, -1.805609061169705 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2Euler121 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    MRP2Euler123(v1, v2);
    set3(1.120685944613971, -0.2323862804943196, 1.424260216144192 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2Euler123 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    MRP2Euler131(v1, v2);
    set3(1.154663818018597, 1.428226451915784, -0.2348127343748092 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2Euler131 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    MRP2Euler132(v1, v2);
    set3(0.1198243320629901, 1.296774918090265, -1.017995395279125 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2Euler132 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    MRP2Euler212(v1, v2);
    set3(-1.537207795170527, 1.298789879764913, 0.4283796513241308 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2Euler212 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    MRP2Euler213(v1, v2);
    set3(-0.4982011776145131, 1.067911809027856, 0.979488037955722 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2Euler213 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    MRP2Euler231(v1, v2);
    set3(-1.415129132201094, 0.4116530390866675, 1.273271587093173 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2Euler231 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    MRP2Euler232(v1, v2);
    set3(0.03358853162436948, 1.298789879764913, -1.142416675470766 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2Euler232 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    MRP2Euler312(v1, v2);
    set3(1.298643836753137, 0.03235316879424937, -1.133389474325039 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2Euler312 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    MRP2Euler313(v1, v2);
    set3(-0.257027406977469, 1.133634172515794, 1.535083362165219 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2Euler313 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    MRP2Euler321(v1, v2);
    set3(1.22957853325386, -1.13227169191098, 0.0762566635156139 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2Euler321 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    MRP2Euler323(v1, v2);
    set3(-1.827823733772366, 1.133634172515794, 3.105879688960115 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2Euler321 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    MRP2Gibbs(v1, v2);
    set3(0.4953560371517029, -0.6191950464396285, 0.7430340557275542 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2Gibbs failed\n");
        printVector("answer", v2);
        error_count++;
    }
    MRP2PRV(v1, v2);
    set3(0.7538859486650076, -0.9423574358312593, 1.130828922997511 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }

    MRPswitch(v1, 1, v2);
    set3(0.2, -0.25, 0.3 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }
    MRPswitch(v1, 0.4, v2);
    set3(-1.038961038961039, 1.298701298701299, -1.558441558441558 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("MRP2PRV failed\n");
        printVector("answer", v2);
        error_count++;
    }

    if (!((a = Picheck(1.2)) == 1.2)) {
        printf("Picheck(1.2) failed\n");
        printf("answer = %g", a);
        error_count++;
    }
    if (!((a = Picheck(4.2)) == -2.083185307179586)) {
        printf("Picheck(1.2) failed\n");
        printf("answer = %g", a);
        error_count++;
    }
    if (!((a = Picheck(-4.2)) == 2.083185307179586)) {
        printf("Picheck(1.2) failed\n");
        printf("answer = %g", a);
        error_count++;
    }

    
    PRV2Euler121(v1, v2);
    set3(2.366822457545908, 0.3898519008736288, -2.164246748437291 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("PRV2Euler121 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    PRV2Euler123(v1, v2);
    set3(0.2381830975647435, -0.2141676691157164, 0.3283009769818029 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("PRV2Euler123 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    PRV2Euler131(v1, v2);
    set3(0.796026130751012, 0.3898519008736288, -0.5934504216423945 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("PRV2Euler131 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    PRV2Euler132(v1, v2);
    set3(0.1659141638227202, 0.3205290820781828, -0.2258549616703266 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("PRV2Euler132 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    PRV2Euler212(v1, v2);
    set3(-1.109161329065078, 0.3596045976550934, 0.8564261174295806 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("PRV2Euler212 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    PRV2Euler213(v1, v2);
    set3(-0.2201931522496843, 0.2326398873102022, 0.2767451364802878 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("PRV2Euler213 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    PRV2Euler231(v1, v2);
    set3(-0.2855829177825758, 0.269101825006778, 0.2414947191533679 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("PRV2Euler231 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    PRV2Euler232(v1, v2);
    set3(0.4616349977298192, 0.3596045976550934, -0.714370209365316 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("PRV2Euler232 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    PRV2Euler312(v1, v2);
    set3(0.3246867163622526, 0.1573915425330904, -0.2785654591200913 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("PRV2Euler312 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    PRV2Euler313(v1, v2);
    set3(-0.7447668031423726, 0.3189446151924337, 1.047343966000315 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("PRV2Euler313 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    PRV2Euler321(v1, v2);
    set3(0.2798880637473677, -0.2750321114914171, 0.1635922230133545 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("PRV2Euler321 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    PRV2Euler323(v1, v2);
    set3(-2.315563129937269, 0.3189446151924337, 2.618140292795212 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("PRV2Euler321 failed\n");
        printVector("answer", v2);
        error_count++;
    }
    PRV2Gibbs(v1, v2);
    set3(0.1016356603597079, -0.1270445754496348, 0.1524534905395618 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("PRV2Gibbs failed\n");
        printVector("answer", v2);
        error_count++;
    }
    PRV2MRP(v1, v2);
    set3(0.05020149056224809, -0.06275186320281011, 0.07530223584337212 ,v3);
    if (!(VequalCheck(v2, v3, 10))) {
        printf("PRV2MRP failed\n");
        printVector("answer", v2);
        error_count++;
    }

    set4(0.45226701686665, 0.75377836144441, 0.15075567228888, 0.45226701686665, v1);
    set4(-0.18663083698528, 0.46657709246321, 0.83983876643378, -0.20529392068381, v2);
    subEP(v1,v2,v3);
    set4(0.3010515331052196, -0.762476312817895, -0.0422034859493331, 0.5711538431809339, v1);
    if (!(V4equalCheck(v3, v1, 10))) {
        printf("subEP failed\n");
        printVector4("answer", v3);
        error_count++;
    }

    set3(10*D2R, 20*D2R, 30*D2R, v1);
    set3(-30*D2R, 200*D2R, 81*D2R, v2);
    subEuler121(v1,v2,v3);
    set3(2.969124082346242, 2.907100217278789, 2.423943306316236,v1);
    if (!(VequalCheck(v3, v1, 10))) {
        printf("subEuler121 failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set3(10*D2R, 20*D2R, 30*D2R, v1);
    set3(-30*D2R, 200*D2R, 81*D2R, v2);
    subEuler123(v1,v2,v3);
    set3(3.116108453572625, -0.6539785291371149, -0.9652248604105184,v1);
    if (!(VequalCheck(v3, v1, 10))) {
        printf("subEuler123 failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set3(10*D2R, 20*D2R, 30*D2R, v1);
    set3(-30*D2R, 200*D2R, 81*D2R, v2);
    subEuler131(v1,v2,v3);
    set3(2.969124082346242, 2.907100217278789, 2.423943306316236 ,v1);
    if (!(VequalCheck(v3, v1, 10))) {
        printf("subEuler131 failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set3(10*D2R, 20*D2R, 30*D2R, v1);
    set3(-30*D2R, 200*D2R, 81*D2R, v2);
    subEuler132(v1,v2,v3);
    set3(2.932019083757663, 0.6246626379494424, -1.519867235625338 ,v1);
    if (!(VequalCheck(v3, v1, 10))) {
        printf("subEuler132 failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set3(10*D2R, 20*D2R, 30*D2R, v1);
    set3(-30*D2R, 200*D2R, 81*D2R, v2);
    subEuler212(v1,v2,v3);
    set3(2.969124082346242, 2.907100217278789, 2.423943306316236 ,v1);
    if (!(VequalCheck(v3, v1, 10))) {
        printf("subEuler212 failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set3(10*D2R, 20*D2R, 30*D2R, v1);
    set3(-30*D2R, 200*D2R, 81*D2R, v2);
    subEuler213(v1,v2,v3);
    set3(2.932019083757663, 0.6246626379494424, -1.519867235625338 ,v1);
    if (!(VequalCheck(v3, v1, 10))) {
        printf("subEuler213 failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set3(10*D2R, 20*D2R, 30*D2R, v1);
    set3(-30*D2R, 200*D2R, 81*D2R, v2);
    subEuler231(v1,v2,v3);
    set3(3.116108453572625, -0.6539785291371149, -0.9652248604105185 ,v1);
    if (!(VequalCheck(v3, v1, 10))) {
        printf("subEuler231 failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set3(10*D2R, 20*D2R, 30*D2R, v1);
    set3(-30*D2R, 200*D2R, 81*D2R, v2);
    subEuler232(v1,v2,v3);
    set3(2.969124082346242, 2.907100217278789, 2.423943306316236 ,v1);
    if (!(VequalCheck(v3, v1, 10))) {
        printf("subEuler232 failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set3(10*D2R, 20*D2R, 30*D2R, v1);
    set3(-30*D2R, 200*D2R, 81*D2R, v2);
    subEuler312(v1,v2,v3);
    set3(3.116108453572625, -0.653978529137115, -0.9652248604105184,v1);
    if (!(VequalCheck(v3, v1, 10))) {
        printf("subEuler312 failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set3(10*D2R, 20*D2R, 30*D2R, v1);
    set3(-30*D2R, 200*D2R, 81*D2R, v2);
    subEuler313(v1,v2,v3);
    set3(2.969124082346242, 2.907100217278789, 2.423943306316236 ,v1);
    if (!(VequalCheck(v3, v1, 10))) {
        printf("subEuler313 failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set3(10*D2R, 20*D2R, 30*D2R, v1);
    set3(-30*D2R, 200*D2R, 81*D2R, v2);
    subEuler321(v1,v2,v3);
    set3(2.932019083757663, 0.6246626379494424, -1.519867235625338 ,v1);
    if (!(VequalCheck(v3, v1, 10))) {
        printf("subEuler321 failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set3(10*D2R, 20*D2R, 30*D2R, v1);
    set3(-30*D2R, 200*D2R, 81*D2R, v2);
    subEuler323(v1,v2,v3);
    set3(2.969124082346242, 2.907100217278789, 2.423943306316236 ,v1);
    if (!(VequalCheck(v3, v1, 10))) {
        printf("subEuler323 failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set3( 1.5, 0.5, 0.5, v1);
    set3(-0.5, 0.25, 0.15, v2);
    subGibbs(v1, v2, v3);
    set3(4.333333333333333, -0.5, 2.166666666666667 , v1);
    if (!(VequalCheck(v3, v1, 10))) {
        printf("subGibbs failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set3( 1.5, 0.5, 0.5, v1);
    set3(-0.5, 0.25, 0.15, v2);
    subMRP(v1, v2, v3);
    set3(0.02739726027397266, -0.2191780821917807, 2.246575342465753, v1);
    if (!(VequalCheck(v3, v1, 10))) {
        printf("subMRP failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set3( 1.5, 0.5, 0.5, v1);
    set3(-0.5, 0.25, 0.15, v2);
    subPRV(v1, v2, v3);
    set3(1.899971363060601, 0.06138537390284331, 0.7174863730592785, v1);
    if (!(VequalCheck(v3, v1, 10))) {
        printf("subPRV failed\n");
        printVector("answer", v3);
        error_count++;
    }

    set3(30*D2R, 30*D2R, 45*D2R, v1);
    Euler3132C(v1,C);
    set3(0.306186217848, 0.883883476483, 0.353553390593, C2[1]);
    set3(-0.918558653544, 0.176776695297, 0.353553390593, C2[2]);
    set3(0.250000000000, -0.433012701892, 0.866025403784, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler3132C failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    set3(60*D2R, 30*D2R, 45*D2R, v1);
    Euler3212C(v1,C);
    set3(0.433012701892, 0.750000000000, -0.500000000000, C2[1]);
    set3(-0.435595740399, 0.659739608441, 0.612372435696, C2[2]);
    set3(0.789149130992, -0.0473671727454, 0.612372435696, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Euler3212C failed\n");
        printMatrix("Euler3212C", C);
        error_count++;
    }

    Mi(30*D2R,3,C);
    set3(0.8660254037844387,   0.4999999999999999,                  0, C2[1]);
    set3(-0.4999999999999999,  0.8660254037844387,                  0, C2[2]);
    set3(0,                                     0,   1.00000000000000, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Mi(30 deg, 3, C) failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    Mi(30*D2R,2,C);
    set3( 0.8660254037844387,                 0,-0.4999999999999999, C2[1]);
    set3(                  0,  1.00000000000000,                  0, C2[2]);
    set3( 0.4999999999999999,                 0, 0.8660254037844387, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Mi(30 deg, 2, C) failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    Mi(30*D2R,1,C);
    set3( 1.0000000000000000,                   0,                   0, C2[1]);
    set3(                  0,  0.8660254037844387,  0.4999999999999999, C2[2]);
    set3(                  0, -0.4999999999999999,  0.8660254037844387, C2[3]);
    if (!(MequalCheck(C, C2, 10))) {
        printf("Mi(30 deg, 1, C) failed\n");
        printMatrix("answer", C);
        error_count++;
    }

    return;
}