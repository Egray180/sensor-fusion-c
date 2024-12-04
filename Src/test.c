#include "../Inc/filter.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.141592653589793

dataBlock fillData(char* line) {
    dataBlock newBlock;

    // Remove newline character if present
    line[strcspn(line, "\n")] = '\0';

    // Tokenize the line using comma as a delimiter
    double numbers[64];
    int i = 0;
    char *token = strtok(line, ",");
    while (token) {
        // Convert token to a number
        numbers[i] = strtod(token, NULL);

        // Get the next token
        token = strtok(NULL, ",");
        i++;
    }

    /* D contains body acceleration (m/s^2), body angular velocity (rad/s), gps latitude (rad), gps longitude (rad),
    gps altitude (m), gps north velocity (m/s), gps east velocity (m/s), gps down velocity (m/s), quaternions from BN0
    (where [q0 q1 q2] is the vector part and q3 is the scalar part), timestamp (ms), barometer altitude (m) */

    /*
    gps_lat (deg),gps_lon (deg),gps_alt (m),gps_vN (m/s),gps_vE (m/s),gps_vD (m/s),accel_x (m/s^2),accel_y (m/s^2),accel_z (m/s^2),gyro_x (deg/s),gyro_y (deg/s),gyro_z (deg/s),mag_x (uT),mag_y (uT),mag_z (uT),
    time (sec),q0 (),q1,q2,q3,ref_Yaw (deg),ref_Pitch (deg),ref_Roll (deg),ref_pos_lat (deg),ref_pos_lon (deg),ref_pos_alt (m)
    */

    newBlock.gps_lat = numbers[0] * PI/180; // convert to rad
    newBlock.gps_lon = numbers[1] * PI/180; // convert to rad
    newBlock.gps_alt = numbers[2];
    newBlock.baro_alt = numbers[2];
    newBlock.gps_vN = numbers[3];
    newBlock.gps_vE = numbers[4];
    newBlock.gps_vD = numbers[5];
    newBlock.accel_x = numbers[6];
    newBlock.accel_y = numbers[7];
    newBlock.accel_z = numbers[8];
    newBlock.gyro_x = numbers[9] * PI/180; // convert to rad/s
    newBlock.gyro_y = numbers[10] * PI/180; // convert to rad/s
    newBlock.gyro_z = numbers[11] * PI/180; // convert to rad/s
    newBlock.time = numbers[15] * 1000; // convert to ms
    newBlock.q0 = numbers[17];
    newBlock.q1 = numbers[18];
    newBlock.q2 = numbers[19];
    newBlock.q3 = numbers[16];

    return newBlock;
}

void main() {
    FILE* dataIn = fopen("test_data/imu_gps.csv", "r");
    char line[1024];
    dataBlock D;
    fgets(line, sizeof(line), dataIn); // skip headers
    fgets(line, sizeof(line), dataIn); // first line
    D = fillData(line);
    KalmanParams params = {5e-5, 5e-5, 1e-6, 1, 25e-4, 8e-4};
    KalmanFilter* filter = init(params, D);

    // open file for writing filter ouput
    FILE *dataOut = fopen("test_data/output.csv", "w");
    fprintf(dataOut, "filter_lat (rad),filter_lon (rad),filter_alt (m),filter_vN (m/s),filter_vE (m/s),filter_vD (m/s),filter_q0,filter_q1,filter_q2,filter_q3\n");
    
    int i = 1;
    while (fgets(line, sizeof(line), dataIn) != NULL) {
        D = fillData(line);
        float64_t T_s = 1e-3*(D.time-filter->data.time);
        predict(filter, filter->data, T_s);
        if (i % 10 == 0) {
            update(filter, D);
        }
        
        fprintf(dataOut, "%.15f,%.15f,%f,%f,%f,%f,%f,%f,%f,%f\n", filter->pos.pData[0], filter->pos.pData[1], filter->pos.pData[2], 
        filter->vel.pData[0], filter->vel.pData[1], filter->vel.pData[2], 
        filter->quat.pData[0], filter->quat.pData[1], filter->quat.pData[2], filter->quat.pData[3]); // [vector, scalar] for quat

        filter->data = D;
        i++;
    }

    fclose(dataIn);
    fclose(dataOut);


    return;
}

/*
mingw32-make
./test_program.exe
*/