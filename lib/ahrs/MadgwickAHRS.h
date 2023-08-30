//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//---------------------------------------------------------------------------------------------------
// Function declarations
typedef struct 
{
    volatile float q0;
    volatile float q1;
    volatile float q2;
    volatile float q3;
} Quaternion;
// struct Quaternion_s
// {
//     volatile float q0;
//     volatile float q1;
//     volatile float q2;
//     volatile float q3;
// } Quaternion_default = {.q0 = 1, .q1 = 0, .q2 = 0, .q3 = 0};
// typedef struct Quaternion_s Quaternion;

typedef struct 
{
    Quaternion* volatile q;
    float beta;
    float sampleFreq;
} Madgwick_state;

Madgwick_state *MadgwickAHRSinit(Madgwick_state *state);
Madgwick_state *MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, Madgwick_state *state);
Madgwick_state *MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, Madgwick_state *state);
void MadgwickGetEulerAnglesDegrees(float *heading, float *pitch, float *roll, Madgwick_state *state);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
