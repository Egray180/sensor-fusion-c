/* includes */
#include "../Inc/filter.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

KalmanFilter* init(KalmanParams params ,dataBlock D) {
    /* D contains body acceleration (m/s^2), body angular velocity (rad/s), gps latitude (rad), gps longitude (rad),
    gps altitude (m), gps north velocity (m/s), gps east velocity (m/s), gps down velocity (m/s), quaternions from BN0
    (where [q0 q1 q2] is the vector part and q3 is the scalar part), timestamp (ms), barometer altitude (m) */
    
    KalmanFilter* instance = (KalmanFilter*)malloc(sizeof(KalmanFilter));
    if (instance == NULL) {
        return NULL;
    }
    instance->a = 6378137.0;
    instance->e = 0.0818;
    instance->w_ie = 7.2921e-5;
    instance->data = D;

    float64_t* p = (float64_t*)malloc(3*sizeof(float64_t));
    p[0] = D.gps_lat;
    p[1] = D.gps_lon;
    p[2] = D.baro_alt;
    arm_mat_init_f64(&(instance->pos), 3, 1, p);

    float64_t* v = (float64_t*)malloc(3*sizeof(float64_t));
    v[0] = D.gps_vN;
    v[1] = D.gps_vE;
    v[2] = D.gps_vD;
    arm_mat_init_f64(&(instance->vel), 3, 1, v);

    float64_t* q = (float64_t*)malloc(4*sizeof(float64_t));
    q[0] = D.q0;
    q[1] = D.q1;
    q[2] = D.q2;
    q[3] = D.q3;
    arm_mat_init_f64(&(instance->quat), 4, 1, q);

    // P matrix (9x9)
    float64_t P_source[9 * 9] = {
        1e-6, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1e-6, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 25e-4, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 25e-4, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 25e-4, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 8e-3, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 8e-3, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 8e-3
    };
    float64_t* P_data = (float64_t*)malloc(9*9*sizeof(float64_t));
    memcpy(P_data, P_source, 9*9*sizeof(float64_t));
    arm_mat_init_f64(&(instance->P), 9, 9, P_data);

    // Q matrix (9x9)
    float64_t Q_source[9 * 9] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, params.e_a, 0, 0, 0, 0, 0,
        0, 0, 0, 0, params.e_a, 0, 0, 0, 0,
        0, 0, 0, 0, 0, params.e_a, 0, 0, 0,
        0, 0, 0, 0, 0, 0, params.e_g, 0, 0,
        0, 0, 0, 0, 0, 0, 0, params.e_g, 0,
        0, 0, 0, 0, 0, 0, 0, 0, params.e_g
    };
    float64_t* Q_data = (float64_t*)malloc(9*9*sizeof(float64_t));
    memcpy(Q_data, Q_source, 9*9*sizeof(float64_t));
    arm_mat_init_f64(&(instance->Q), 9, 9, Q_data);

    // R matrix (6x6)
    float64_t R_source[6 * 6] = {
        params.e_gps_p, 0, 0, 0, 0, 0,
        0, params.e_gps_p, 0, 0, 0, 0,
        0, 0, params.e_gps_alt, 0, 0, 0,
        0, 0, 0, params.e_gps_v, 0, 0,
        0, 0, 0, 0, params.e_gps_v, 0,
        0, 0, 0, 0, 0, params.e_gps_v
    };
    float64_t* R_data = (float64_t*)malloc(6*6*sizeof(float64_t));
    memcpy(R_data, R_source, 6*6*sizeof(float64_t));
    arm_mat_init_f64(&(instance->R), 6, 6, R_data);

    // H matrix (6x9)
    float64_t H_source[6 * 9] = {
        1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0
    };
    float64_t* H_data = (float64_t*)malloc(6*9*sizeof(float64_t));
    memcpy(H_data, H_source, 6*9*sizeof(float64_t));
    arm_mat_init_f64(&(instance->H), 6, 9, H_data);    

    float64_t* tilt_error = (float64_t*)malloc(3*sizeof(float64_t));
    tilt_error[0] = params.e_tilt;
    tilt_error[1] = params.e_tilt;
    tilt_error[2] = params.e_tilt;
    instance->BNO_tilt_error = tilt_error;

    instance->att_thresh = params.att_thresh;

    return instance;
}

void filter_data(KalmanFilter* filter, dataBlock D) {
    float64_t T_s = 1e-3*(D.time-filter->data.time);
    predict(filter, filter->data, T_s);
    /*
    if (condition) {
        update(filter, D);
    }
    */
   filter->data = D;
   return;
}

void predict(KalmanFilter* filter, dataBlock D, float64_t T_s) {

    // Quaternion components
    float64_t b1 = filter->quat.pData[0];
    float64_t b2 = filter->quat.pData[1];
    float64_t b3 = filter->quat.pData[2];
    float64_t b4 = filter->quat.pData[3];

    // Intermediate values
    float64_t p1 = b1 * b1;
    float64_t p2 = b2 * b2;
    float64_t p3 = b3 * b3;
    float64_t p4 = b4 * b4;
    float64_t p5 = p2 + p3;

    // Calculate p6 based on condition
    float64_t p6;
    if (p5 + p4 + p1 != 0) {
        p6 = 2.0f / (p5 + p4 + p1);
    } else {
        p6 = 0.0f;
    }

    // Calculate rotation matrix elements
    float64_t R_11 = 1.0f - p6 * p5;
    float64_t R_22 = 1.0f - p6 * (p1 + p3);
    float64_t R_33 = 1.0f - p6 * (p1 + p2);
    p1 = p6 * b1;
    p2 = p6 * b2;
    p5 = p6 * b3 * b4;
    p6 = p1 * b2;
    float64_t R_12 = p6 - p5;
    float64_t R_21 = p6 + p5;
    p5 = p2 * b4;
    p6 = p1 * b3;
    float64_t R_13 = p6 + p5;
    float64_t R_31 = p6 - p5;
    p5 = p1 * b4;
    p6 = p2 * b3;
    float64_t R_23 = p6 - p5;
    float64_t R_32 = p6 + p5;

    float64_t R_p2n_source[] = {R_11, R_12, R_13, R_21, R_22, R_23, R_31, R_32, R_33};
    arm_matrix_instance_f64 R_p2n = {3, 3, R_p2n_source};

    float64_t R_n2p_source[9];
    arm_matrix_instance_f64 R_n2p = {3, 3, R_n2p_source};
    arm_status status = arm_mat_inverse_f64(&R_p2n, &R_n2p);
    if (status != ARM_MATH_SUCCESS) {
        printf("%s\n", "Inversion Failed");
        return;
    }
    float64_t fix_R_p2n[] = {R_11, R_12, R_13, R_21, R_22, R_23, R_31, R_32, R_33}; // arm_mat_inverse_f64 modifies source matrix
    for (int i = 0; i < 9; i++) {
        R_p2n_source[i] = fix_R_p2n[i];
    }


    float64_t lat = filter->pos.pData[0];
    float64_t sin_lat = sin(lat);
    float64_t cos_lat = cos(lat);
    float64_t tan_lat = tan(lat);
    float64_t h = filter->pos.pData[2];

    float64_t R_meridian = filter->a * (1 - pow(filter->e, 2)) / pow((1 - pow(filter->e, 2) * pow(sin_lat, 2)), 1.5) + h;
    float64_t R_transverse = filter->a / pow((1 - pow(filter->e, 2) * pow(sin_lat, 2)), 0.5) + h;
    float64_t R_e = sqrt(R_meridian * R_transverse); // assume that R_meridian ~ R_transverse ~ R_e moving forward

    float64_t f_p_source[] = {D.accel_x, D.accel_y, D.accel_z};
    arm_matrix_instance_f64 f_p = {3, 1, f_p_source};
    float64_t f_n_source[3];
    arm_matrix_instance_f64 f_n = {3, 1, f_n_source};
    arm_mat_mult_f64(&R_p2n, &f_p, &f_n);
    
    float64_t v_N = filter->vel.pData[0];
    float64_t v_E = filter->vel.pData[1];
    float64_t v_D = filter->vel.pData[2];
    float64_t f_N = f_n.pData[0];
    float64_t f_E = f_n.pData[1];
    float64_t f_D = f_n.pData[2];

    float64_t omega_N = filter->w_ie * cos_lat;
    float64_t omega_D = -filter->w_ie * sin_lat;
    float64_t rho_N = v_E / R_e;
    float64_t rho_E = -v_N / R_e;
    float64_t rho_D = -v_E * tan_lat / R_e;
    float64_t w_N = omega_N + rho_N;
    float64_t w_E = rho_E;
    float64_t w_D = omega_D + rho_D;

    float64_t k_D = v_D / R_e;
    float64_t F_41 = -2 * omega_N * v_E - (rho_N * v_E) / pow(cos_lat, 2);
    float64_t F_43 = rho_E * k_D - rho_N * rho_D;
    float64_t F_51 = 2 * (omega_N * v_N + omega_D * v_D) + (rho_N * v_N) / pow(cos_lat, 2);
    float64_t F_53 = -rho_E * rho_D * k_D * rho_N;
    float64_t F_55 = k_D - rho_E * tan_lat;
    float64_t F_63 = pow(rho_N, 2) + pow(rho_E, 2); // - 2 * g / R_e

    float64_t F_source[] = {
        0, 0, rho_E / R_e, 1 / R_e, 0, 0, 0, 0, 0,
        -rho_D / cos_lat, 0, -rho_N / (R_e * cos_lat), 0, 1 / (R_e * cos_lat), 0, 0, 0, 0,
        0, 0, 0, 0, 0, -1, 0, 0, 0,
        F_41, 0, F_43, k_D, 2 * w_D, -rho_E, 0, f_D, -f_E,
        F_51, 0, F_53, -(w_D + omega_D), F_55, w_N + omega_N, -f_D, 0, f_N,
        -2 * v_E * omega_D, 0, F_63, 2 * rho_E, -2 * w_N, 0, f_E, f_N, 0,
        -omega_D, 0, rho_N / R_e, 0, -1 / R_e, 0, 0, w_D, -w_E,
        0, 0, rho_E / R_e, 1 / R_e, 0, 0, -w_D, 0, w_N,
        omega_N + rho_N / pow(cos_lat, 2), 0, rho_D / R_e, 0, tan_lat / R_e, 0, w_E, -w_N, 0
    };
    arm_matrix_instance_f64 F = {9, 9, F_source};

    float64_t pos_transform_source[] = {
        1 / R_meridian, 0, 0,
        0, 1 / (R_transverse * cos_lat), 0,
        0, 0, -1
    };
    arm_matrix_instance_f64 pos_transform = {3, 3, pos_transform_source};
    float64_t p_dot_source[3];
    arm_matrix_instance_f64 p_dot = {3, 1, p_dot_source};
    arm_mat_mult_f64(&pos_transform, &(filter->vel), &p_dot);

    float64_t lat_dot = p_dot.pData[0];
    float64_t long_dot = p_dot.pData[1];
    float64_t g_source[] = {0, 0, 9.79};
    arm_matrix_instance_f64 g = {3, 1, g_source};
    float64_t vel_transform_source[] = {
        0, -(long_dot + 2 * filter->w_ie) * sin_lat, lat_dot,
        (long_dot + 2 * filter->w_ie) * sin_lat, 0, (long_dot + 2 * filter->w_ie) * cos_lat,
        -lat_dot, -(long_dot + 2 * filter->w_ie) * cos_lat, 0
    };
    arm_matrix_instance_f64 vel_transform = {3, 3, vel_transform_source};
    float64_t v_dot_source[3];
    arm_matrix_instance_f64 v_dot = {3, 1, v_dot_source};
    float64_t fn_g_source[3];
    arm_matrix_instance_f64 fn_g = {3, 1, fn_g_source};
    float64_t vt_v_source[3];
    arm_matrix_instance_f64 vt_v = {3, 1, vt_v_source};
    arm_mat_add_f64(&f_n, &g, &fn_g);
    arm_mat_mult_f64(&vel_transform, &(filter->vel), &vt_v);
    arm_mat_add_f64(&fn_g, &vt_v, &v_dot);

    float64_t gyro_mod_source[] = {(long_dot + filter->w_ie) * cos_lat, -lat_dot, -(long_dot + filter->w_ie) * sin_lat};
    arm_matrix_instance_f64 gyro_mod = {3, 1, gyro_mod_source};
    float64_t gyro_source[3];
    arm_matrix_instance_f64 gyro = {3, 1, gyro_source};
    float64_t gyro_ip_source[] = {D.gyro_x, D.gyro_y, D.gyro_z};
    arm_matrix_instance_f64 gyro_ip = {3, 1, gyro_ip_source};
    float64_t Rn2p_gmod_source[3];
    arm_matrix_instance_f64 Rn2p_gmod = {3, 1, Rn2p_gmod_source};
    arm_mat_mult_f64(&R_n2p, &gyro_mod, &Rn2p_gmod);
    arm_mat_sub_f64(&gyro_ip, &Rn2p_gmod, &gyro);
    float64_t p = gyro.pData[0];
    float64_t q = gyro.pData[1];
    float64_t r = gyro.pData[2];
    float64_t norm = sqrt(p * p + q * q + r * r);
    float64_t omega_source[] = {
        0, r, -q, p,
        -r, 0, p, q,
        q, -p, 0, r,
        -p, -q, -r, 0
    };
    arm_matrix_instance_f64 omega = {4, 4, omega_source};

    float64_t Ts_pdot_source[3];
    arm_matrix_instance_f64 Ts_pdot = {3, 1, Ts_pdot_source};
    float64_t pos_Ts_pdot_source[3];
    arm_matrix_instance_f64 pos_Ts_pdot = {3, 1, pos_Ts_pdot_source};
    arm_mat_scale_f64(&p_dot, T_s, &Ts_pdot);
    arm_mat_add_f64(&(filter->pos), &Ts_pdot, &pos_Ts_pdot);
    filter->pos.pData[0] = pos_Ts_pdot.pData[0];
    filter->pos.pData[1] = pos_Ts_pdot.pData[1];
    filter->pos.pData[2] = pos_Ts_pdot.pData[2];

    float64_t Ts_vdot_source[3];
    arm_matrix_instance_f64 Ts_vdot = {3, 1, Ts_vdot_source};
    float64_t vel_Ts_vdot_source[3];
    arm_matrix_instance_f64 vel_Ts_vdot = {3, 1, vel_Ts_vdot_source};
    arm_mat_scale_f64(&v_dot, T_s, &Ts_vdot);
    arm_mat_add_f64(&(filter->vel), &Ts_vdot, &vel_Ts_vdot);
    filter->vel.pData[0] = vel_Ts_vdot.pData[0];
    filter->vel.pData[1] = vel_Ts_vdot.pData[1];
    filter->vel.pData[2] = vel_Ts_vdot.pData[2];

    if (norm > 1e-6) {
        float64_t identity_4x4_source[] = {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        };
        arm_matrix_instance_f64 identity_4x4 = {4, 4, identity_4x4_source};
        float64_t scaled_identity_source[4 * 4];
        arm_matrix_instance_f64 scaled_identity = {4, 4, scaled_identity_source};
        float64_t scaled_omega_source[4 * 4];
        arm_matrix_instance_f64 scaled_omega = {4, 4, scaled_omega_source};
        float64_t quat_transform_source[4 * 4];
        arm_matrix_instance_f64 quat_transform = {4, 4, quat_transform_source};
        float64_t new_quat_source[4];
        arm_matrix_instance_f64 new_quat = {4, 1, new_quat_source};
        arm_mat_scale_f64(&identity_4x4, cos(0.5 * T_s * norm), &scaled_identity);
        arm_mat_scale_f64(&omega, 1 / norm * sin(0.5 * T_s * norm), &scaled_omega);
        arm_mat_add_f64(&scaled_identity, &scaled_omega, &quat_transform);
        arm_mat_mult_f64(&quat_transform, &(filter->quat), &new_quat);
        filter->quat.pData[0] = new_quat.pData[0];
        filter->quat.pData[1] = new_quat.pData[1];
        filter->quat.pData[2] = new_quat.pData[2];
        filter->quat.pData[3] = new_quat.pData[3];
    }

    float64_t identity_9x9_source[] = {
        1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1
    };
    arm_matrix_instance_f64 identity_9x9 = {9, 9, identity_9x9_source};
    float64_t Ts_F_source[9 * 9];
    arm_matrix_instance_f64 Ts_F = {9, 9, Ts_F_source};
    float64_t I_Ts_F_source[9 * 9];
    arm_matrix_instance_f64 I_Ts_F = {9, 9, I_Ts_F_source};
    float64_t I_Ts_F_P_source[9 * 9];
    arm_matrix_instance_f64 I_Ts_F_P = {9, 9, I_Ts_F_P_source};
    float64_t I_Ts_F_trans_source[9 * 9];
    arm_matrix_instance_f64 I_Ts_F_trans = {9, 9, I_Ts_F_trans_source};
    float64_t TPTt_source[9 * 9];
    arm_matrix_instance_f64 TPTt = {9, 9, TPTt_source};
    float64_t Ts_Q_source[9 * 9];
    arm_matrix_instance_f64 Ts_Q = {9, 9, Ts_Q_source};
    float64_t new_P_source[9 * 9];
    arm_matrix_instance_f64 new_P = {9, 9, new_P_source};
    arm_mat_scale_f64(&F, T_s, &Ts_F);
    arm_mat_add_f64(&identity_9x9, &Ts_F, &I_Ts_F);
    arm_mat_mult_f64(&I_Ts_F, &(filter->P), &I_Ts_F_P);
    arm_mat_trans_f64(&I_Ts_F, &I_Ts_F_trans);
    arm_mat_mult_f64(&I_Ts_F_P, &I_Ts_F_trans, &TPTt);
    arm_mat_scale_f64(&(filter->Q), T_s, &Ts_Q);
    arm_mat_add_f64(&TPTt, &Ts_Q, &new_P);
    memcpy(filter->P.pData, new_P.pData, 9*9*sizeof(float64_t));

    return;
}

void update(KalmanFilter* filter, dataBlock D) {
    // y = gps - H * [pos; vel; zeros(3,1)]
    float64_t gps_data[] = { D.gps_lat, D.gps_lon, D.baro_alt, D.gps_vN, D.gps_vE, D.gps_vD };
    arm_matrix_instance_f64 gps_matrix = {6, 1, gps_data};

    float64_t state_data[9];
    memcpy(state_data, filter->pos.pData, 3 * sizeof(float64_t));
    memcpy(&state_data[3], filter->vel.pData, 3 * sizeof(float64_t));
    memset(&state_data[6], 0, 3 * sizeof(float64_t));
    arm_matrix_instance_f64 state_matrix = {9, 1, state_data};

    float64_t Hx_data[6];
    arm_matrix_instance_f64 Hx = {6, 1, Hx_data};
    arm_mat_mult_f64(&(filter->H), &state_matrix, &Hx);

    float64_t y_data[6];
    arm_matrix_instance_f64 y = {6, 1, y_data};
    arm_mat_sub_f64(&gps_matrix, &Hx, &y);

    // K = P * H' * (H * P * H' + R)^-1
    float64_t Ht_data[9 * 6];
    arm_matrix_instance_f64 Ht = {9, 6, Ht_data};
    arm_mat_trans_f64(&(filter->H), &Ht);

    float64_t HP_data[6 * 9];
    arm_matrix_instance_f64 HP = {6, 9, HP_data};
    arm_mat_mult_f64(&(filter->H), &(filter->P), &HP);

    float64_t HPHt_data[6 * 6];
    arm_matrix_instance_f64 HPHt = {6, 6, HPHt_data};
    arm_mat_mult_f64(&HP, &Ht, &HPHt);

    float64_t HPHt_R_data[6 * 6];
    arm_matrix_instance_f64 HPHt_R = {6, 6, HPHt_R_data};
    arm_mat_add_f64(&HPHt, &(filter->R), &HPHt_R);

    float64_t HPHt_R_inv_data[6 * 6];
    arm_matrix_instance_f64 HPHt_R_inv = {6, 6, HPHt_R_inv_data};
    arm_status status = arm_mat_inverse_f64(&HPHt_R, &HPHt_R_inv);
    if (status != ARM_MATH_SUCCESS) {
        printf("%s\n", "Inversion Failed"); 
        return;
    }

    float64_t PHt_data[9 * 6];
    arm_matrix_instance_f64 PHt = {9, 6, PHt_data};
    arm_mat_mult_f64(&(filter->P), &Ht, &PHt);

    float64_t K_data[9 * 6];
    arm_matrix_instance_f64 K = {9, 6, K_data};
    arm_mat_mult_f64(&PHt, &HPHt_R_inv, &K);

    // Modify the position and velocity states
    float64_t K_y_data[9];
    arm_matrix_instance_f64 K_y = {9, 1, K_y_data};
    arm_mat_mult_f64(&K, &y, &K_y);

    filter->pos.pData[0] += K_y.pData[0];
    filter->pos.pData[1] += K_y.pData[1];
    filter->pos.pData[2] += K_y.pData[2];
    filter->vel.pData[0] += K_y.pData[3];
    filter->vel.pData[1] += K_y.pData[4];
    filter->vel.pData[2] += K_y.pData[5];
    

    // Update covariance: P = (I - K * H) * P
    float64_t I_KH_data[9 * 9];
    arm_matrix_instance_f64 I_KH = {9, 9, I_KH_data};

    float64_t KH_data[9 * 9];
    arm_matrix_instance_f64 KH = {9, 9, KH_data};
    arm_mat_mult_f64(&K, &(filter->H), &KH);

    for (int i = 0; i < 9; i++) {
        for (int j = 9*i; j < 9*i+9; j++) {
            I_KH_data[j] = -KH_data[j];
        }
        I_KH_data[i * 10] += 1.0f;  // Diagonal elements
    }

    float64_t P_new_data[9 * 9];
    arm_matrix_instance_f64 P_new = {9, 9, P_new_data};
    arm_mat_mult_f64(&I_KH, &(filter->P), &P_new);
    memcpy(filter->P.pData, P_new.pData, 9*9*sizeof(float64_t));

    // Update quaternion for attitude correction
    float64_t q_data[] = { D.q0, D.q1, D.q2, D.q3 };
    arm_matrix_instance_f64 q = {4, 1, q_data};

    float64_t q0 = filter->quat.pData[0];
    float64_t q1 = filter->quat.pData[1];
    float64_t q2 = filter->quat.pData[2];
    float64_t q3 = filter->quat.pData[3];
    float64_t Q_conj_data[] = {
        q3,  q2, -q1,  q0,
        -q2,  q3,  q0,  q1,
        q1, -q0,  q3,  q2,
        -q0, -q1, -q2,  q3
    };
    arm_matrix_instance_f64 Q_conj = {4, 4, Q_conj_data};

    float64_t Q_conj_inv_data[4 * 4];
    arm_matrix_instance_f64 Q_conj_inv = {4, 4, Q_conj_inv_data};
    status = arm_mat_inverse_f64(&Q_conj, &Q_conj_inv);
    if (status != ARM_MATH_SUCCESS) {
        printf("%s\n", "Inversion Failed"); 
        return;
    }
    float64_t fix_Q_conj[] = {
        q3,  q2, -q1,  q0,
        -q2,  q3,  q0,  q1,
        q1, -q0,  q3,  q2,
        -q0, -q1, -q2,  q3
    }; // arm_mat_inverse_f64 modifies source matrix
    for (int i = 0; i < 16; i++) {
        Q_conj_data[i] = fix_Q_conj[i];
    }

    float64_t q_mod_data[4];
    arm_matrix_instance_f64 q_mod = {4, 1, q_mod_data};
    arm_mat_mult_f64(&Q_conj_inv, &q, &q_mod);

    float64_t mag = 2 * acos(q_mod.pData[3]);

    float64_t q_mod_prime_data[4];
    arm_matrix_instance_f64 q_mod_prime = {4, 1, q_mod_prime_data};
    for (int i = 0; i < 4; i++) {
        q_data[i] *= -1;
    }
    arm_mat_mult_f64(&Q_conj_inv, &q, &q_mod_prime);

    float64_t mag_prime = 2 * acos(q_mod_prime.pData[3]);

    if (mag > mag_prime) {
        memcpy(q_mod.pData, q_mod_prime.pData, 4*sizeof(float64_t));
        mag = mag_prime;
    }

    if (mag < 1e-3) {
        return; // don't bother correcting if error is really small
    } 

    float64_t new_tilt_error[3];
    float64_t P_tilt_error[] = {filter->P.pData[9*6+6], filter->P.pData[9*7+7], filter->P.pData[9*8+8]};
    if (mag > filter->att_thresh) { // adopt BNO quaternions if discrepancy is too large
        for (int i = 0; i < 4; i++) {
            q_data[i] *= -1;
        }
        memcpy(filter->quat.pData, q_data, 4*sizeof(float64_t));
        for (int i = 0; i < 3; i++) {
            new_tilt_error[i] = filter->BNO_tilt_error[i];
        }
    } else {
        float64_t scale = mag / sin(mag / 2);
        float64_t tilt_error[] = {scale * q_mod.pData[0], scale * q_mod.pData[1], scale * q_mod.pData[2]};

        // Scale tilt error according to covariance
        tilt_error[0] = tilt_error[0] * P_tilt_error[0] / (P_tilt_error[0] + filter->BNO_tilt_error[0]);
        tilt_error[1] = tilt_error[1] * P_tilt_error[1] / (P_tilt_error[1] + filter->BNO_tilt_error[1]);
        tilt_error[2] = tilt_error[2] * P_tilt_error[2] / (P_tilt_error[2] + filter->BNO_tilt_error[2]);

        // Update attitude
        mag = sqrt(tilt_error[0]*tilt_error[0] + tilt_error[1]*tilt_error[1] + tilt_error[2]*tilt_error[2]);
        scale = sin(mag / 2) / mag;
        for (int i = 0; i < 3; i++) {
            q_mod.pData[i] = scale * tilt_error[i];
        }
        q_mod.pData[3] = cos(mag / 2);
        arm_mat_mult_f64(&Q_conj, &q_mod, &(filter->quat));

        float64_t new_tilt_error[3];
        for (int i = 0; i < 3; i++) {
            if (P_tilt_error[i] <= filter->BNO_tilt_error[i]) {
                new_tilt_error[i] = P_tilt_error[i];
            } else {
                new_tilt_error[i] = filter->BNO_tilt_error[i];
            }
        }
    }
    // Update P (zones A & C)
    for (int i = 0; i < 6; i++) {
        for (int j = 6; j < 9; j++) {
            filter->P.pData[i * 9 + j] *= sqrt(new_tilt_error[j - 6] / P_tilt_error[j - 6]);
            filter->P.pData[j * 9 + i] = filter->P.pData[i * 9 + j];
        }
    }
    // Update (zone B)
    for (int i = 6; i < 9; i++) {
        for (int j = 6; j < 9; j++) {
            filter->P.pData[i * 9 + j] *= sqrt(new_tilt_error[i - 6] / P_tilt_error[i - 6]) *
                                sqrt(new_tilt_error[j - 6] / P_tilt_error[j - 6]);
        }
    }

    return;
}

void destroy(KalmanFilter* instance) {
    free(instance->pos.pData);
    free(instance->vel.pData);
    free(instance->quat.pData);
    free(instance->P.pData);
    free(instance->Q.pData);
    free(instance->R.pData);
    free(instance->H.pData);
    free(instance->BNO_tilt_error);
    free(instance);
    return;
}

