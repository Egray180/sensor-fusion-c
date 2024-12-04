#include "arm_math.h"

typedef struct {
    float64_t accel_x;
    float64_t accel_y;
    float64_t accel_z;
    float64_t gyro_x;
    float64_t gyro_y;
    float64_t gyro_z;
    float64_t gps_lat;
    float64_t gps_lon;
    float64_t gps_alt;
    float64_t gps_vN;
    float64_t gps_vE;
    float64_t gps_vD;
    float64_t q0;
    float64_t q1;
    float64_t q2;
    float64_t q3;
    float64_t baro_alt;
    float64_t time;

} dataBlock;

typedef struct {
    float64_t e_a;
    float64_t e_g;
    float64_t e_gps_p;
    float64_t e_gps_alt;
    float64_t e_gps_v;
    float64_t e_tilt;
} KalmanParams;

typedef struct {
    float64_t a; // semimajor axis length of WGS-84 ellipsoid
    float64_t e; // eccentricity of WGS-84 ellipsoid
    float64_t w_ie; // earth angular velocity

    dataBlock data;

    arm_matrix_instance_f64 pos;
    arm_matrix_instance_f64 vel;
    arm_matrix_instance_f64 quat;

    arm_matrix_instance_f64 P;
    arm_matrix_instance_f64 Q;
    arm_matrix_instance_f64 R;
    arm_matrix_instance_f64 H;
    float64_t* BNO_tilt_error;

} KalmanFilter;

/*
FUNCTIONS
*/

KalmanFilter* init(KalmanParams params ,dataBlock D);

void filter_data(KalmanFilter* filter, dataBlock D);

void predict(KalmanFilter* filter, dataBlock D, float64_t T_s);

void update(KalmanFilter* filter, dataBlock D);

void destroy(KalmanFilter* instance);