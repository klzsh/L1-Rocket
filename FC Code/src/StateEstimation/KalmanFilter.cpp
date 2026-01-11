/*
  genuinely some of the worst code I have written.
*/

#include "KalmanFilter.h"

// control input matrix
static arm_matrix_instance_f32 U;
static float32_t U_buf[CONTROL_SIZE] = {0};

// state vector
static arm_matrix_instance_f32 X;
static float32_t X_buf[STATE_SIZE] = {0};

// observation matrix
static arm_matrix_instance_f32 H;
static float32_t H_buf[MEASUREMENT_SIZE * STATE_SIZE] = {H_MATRIX_COEFS};

// Estimated covariance matrix
static arm_matrix_instance_f32 P;
static float32_t P_buf[STATE_SIZE * STATE_SIZE] = {0};

// measurement uncertainty matrix
static arm_matrix_instance_f32 R;
static float32_t R_buf[MEASUREMENT_SIZE * MEASUREMENT_SIZE] = {R_MATRIX_COEFS};

// Kalman gain
static arm_matrix_instance_f32 K;
static float32_t K_buf[STATE_SIZE * MEASUREMENT_SIZE] = {0};

// process noise matrix
static arm_matrix_instance_f32 Q;
static float32_t Q_buf[STATE_SIZE * STATE_SIZE] = {Q_MATRIX_COEFS};
// measurement vector
static arm_matrix_instance_f32 Z;
static float32_t Z_buf[MEASUREMENT_SIZE] = {0};

// covariance update
static arm_matrix_instance_f32 Identity_matrix;
static float32_t Identity_buf[STATE_SIZE * STATE_SIZE] = {0};

uint32_t previousTimestamp = 0;

void initializeKF(uint32_t timestamp)
{

  arm_mat_init_f32(&U, CONTROL_SIZE, 1, U_buf);
  arm_mat_init_f32(&X, STATE_SIZE, 1, X_buf);
  arm_mat_init_f32(&H, MEASUREMENT_SIZE, STATE_SIZE, H_buf);
  arm_mat_init_f32(&P, STATE_SIZE, STATE_SIZE, P_buf);
  arm_mat_init_f32(&R, MEASUREMENT_SIZE, MEASUREMENT_SIZE, R_buf);
  arm_mat_init_f32(&K, STATE_SIZE, MEASUREMENT_SIZE, K_buf);
  arm_mat_init_f32(&Q, STATE_SIZE, STATE_SIZE, Q_buf);
  arm_mat_init_f32(&Z, MEASUREMENT_SIZE, 1, Z_buf);

  for (int i = 0; i < STATE_SIZE; i++)
  {
    Identity_buf[STATE_SIZE * i + i] = 1;
  }
  // calculate initial state
  arm_mat_init_f32(&Identity_matrix, STATE_SIZE, STATE_SIZE, Identity_buf);
  //! arbitrary number
  iterateFilter(0.02);
  previousTimestamp = timestamp/1'000'000.0f;
}

void predict(const arm_matrix_instance_f32 *F, const arm_matrix_instance_f32 *G)
{
  // {Matrix 1}_OPERATOR_{Matrix 2}
  // STATE_SIZE * 1
  arm_matrix_instance_f32 F_T_X;
  float32_t buf1[STATE_SIZE * 1] = {0};
  arm_mat_init_f32(&F_T_X, STATE_SIZE, 1, buf1);

  // STATE_SIZE * 1
  arm_matrix_instance_f32 G_T_U;
  float32_t buf2[STATE_SIZE * 1] = {0};
  arm_mat_init_f32(&G_T_U, STATE_SIZE, 1, buf2);
  
  arm_mat_mult_f32(F, &X, &F_T_X);
  arm_mat_mult_f32(G, &U, &G_T_U);
  arm_mat_add_f32(&F_T_X, &G_T_U, &X);
}
void estimateState()
{
  // 1
  // MEASUREMENT_SIZE * 1
  arm_matrix_instance_f32 H_T_X;
  float32_t buf1[MEASUREMENT_SIZE * 1] = {0};
  arm_mat_init_f32(&H_T_X, MEASUREMENT_SIZE, 1, buf1);
  
  // 2
  // MEASUREMENT_SIZE *1
  arm_matrix_instance_f32 Z_S_1;
  float32_t buf2[MEASUREMENT_SIZE * 1] = {0};
  arm_mat_init_f32(&Z_S_1, MEASUREMENT_SIZE, 1, buf2);

  // 3
  // MEASUREMENT_SIZE *1
  arm_matrix_instance_f32 K_T_2;
  float32_t buf3[STATE_SIZE * 1] = {0};
  arm_mat_init_f32(&K_T_2, STATE_SIZE, 1, buf3);

  // 4
  // STATE_SIZE * 1
  arm_matrix_instance_f32 X_P_3;
  float32_t buf4[STATE_SIZE * 1] = {0};
  arm_mat_init_f32(&X_P_3, STATE_SIZE, 1, buf4);



  arm_mat_mult_f32(&H, &X, &H_T_X);
  arm_mat_sub_f32(&Z, &H_T_X, &Z_S_1);
  arm_mat_mult_f32(&K, &Z_S_1, &K_T_2);
  arm_mat_add_f32(&X, &K_T_2, &X_P_3);
  memcpy(X.pData, X_P_3.pData, STATE_SIZE * sizeof(float32_t));
}
void calculateKalmanGain()
{
  // HPH'+R
  // inv(HPH'+R)
  // P*H'
  // P*H'*inv
  // STATE_SIZE * MEASUREMENT_SIZE
  arm_matrix_instance_f32 H_Transpose;
  float32_t buf1[STATE_SIZE * MEASUREMENT_SIZE] = {0};
  arm_mat_init_f32(&H_Transpose, STATE_SIZE, MEASUREMENT_SIZE, buf1);

  // 1
  // MEASUREMENT_SIZE * STATE_SIZE
  arm_matrix_instance_f32 H_T_P;
  float32_t buf2[MEASUREMENT_SIZE * STATE_SIZE] = {0};
  arm_mat_init_f32(&H_T_P, MEASUREMENT_SIZE, STATE_SIZE, buf2);

  // 2
  // MEASUREMENT_SIZE * MEASUREMENT_SIZE
  arm_matrix_instance_f32 m1_T_H_Transpose;
  float32_t buf3[MEASUREMENT_SIZE * MEASUREMENT_SIZE] = {0};
  arm_mat_init_f32(&m1_T_H_Transpose, MEASUREMENT_SIZE, MEASUREMENT_SIZE, buf3);


  // 3
  // MEASUREMENT_SIZE * MEASUREMENT_SIZE
  arm_matrix_instance_f32 m2_P_R;
  float32_t buf4[MEASUREMENT_SIZE * MEASUREMENT_SIZE] = {0};
  arm_mat_init_f32(&m2_P_R, MEASUREMENT_SIZE, MEASUREMENT_SIZE, buf4);
 
  // 4
  // MEASUREMENT_SIZE * MEASUREMENT_SIZE
  arm_matrix_instance_f32 inverse_3;
  float32_t buf5[MEASUREMENT_SIZE * MEASUREMENT_SIZE] = {0};
  arm_mat_init_f32(&inverse_3, MEASUREMENT_SIZE, MEASUREMENT_SIZE, buf5);
 
  // 5
  // STATE_SIZE * MEASUREMENT_SIZE
  arm_matrix_instance_f32 P_T_H_transpose;
  float32_t buf6[STATE_SIZE * MEASUREMENT_SIZE] = {0};
  arm_mat_init_f32(&P_T_H_transpose, STATE_SIZE, MEASUREMENT_SIZE, buf6);

  arm_mat_trans_f32(&H, &H_Transpose);
  arm_mat_mult_f32(&H, &P, &H_T_P);
  arm_mat_mult_f32(&H_T_P, &H_Transpose, &m1_T_H_Transpose);
  arm_mat_add_f32(&m1_T_H_Transpose, &R, &m2_P_R);
  arm_mat_inverse_f32(&m2_P_R, &inverse_3);
  arm_mat_mult_f32(&P, &H_Transpose, &P_T_H_transpose);
  arm_mat_mult_f32(&P_T_H_transpose, &inverse_3, &K);
}
void updateCovariance()
{
  // k*h
  //  identity - kh
  //  *p
  //  (identity -kh)'
  //   k*r
  //  kr * k'
  // 1
  // STATE_SIZE * STATE_SIZE
  arm_matrix_instance_f32 K_T_H;
  float32_t buf1[STATE_SIZE * STATE_SIZE] = {0};
  arm_mat_init_f32(&K_T_H, STATE_SIZE, STATE_SIZE, buf1);
 
  // 2
  // STATE_SIZE * STATE_SIZE
  arm_matrix_instance_f32 I_S_1;
  float32_t buf2[STATE_SIZE * STATE_SIZE] = {0};
  arm_mat_init_f32(&I_S_1, STATE_SIZE, STATE_SIZE, buf2);

  // STATE_SIZE * STATE_SIZE
  arm_matrix_instance_f32 m2_transpose;
  float32_t buf3[STATE_SIZE * STATE_SIZE] = {0};
  arm_mat_init_f32(&m2_transpose, STATE_SIZE, STATE_SIZE, buf3);

  // 3
  // STATE_SIZE * STATE_SIZE
  arm_matrix_instance_f32 m2_T_P;
  float32_t buf4[STATE_SIZE * STATE_SIZE] = {0};
  arm_mat_init_f32(&m2_T_P, STATE_SIZE, STATE_SIZE, buf4);

  // 4
  // STATE_SIZE * STATE_SIZE
  arm_matrix_instance_f32 m3_T_m2_transpose;
  float32_t buf5[STATE_SIZE * STATE_SIZE] = {0};
  arm_mat_init_f32(&m3_T_m2_transpose, STATE_SIZE, STATE_SIZE, buf5);

  // 5
  // STATE_SIZE * MEASUREMENT_SIZE
  arm_matrix_instance_f32 K_T_R;
  float32_t buf6[STATE_SIZE * MEASUREMENT_SIZE] = {0};
  arm_mat_init_f32(&K_T_R, STATE_SIZE, MEASUREMENT_SIZE, buf6);

  // MEASUREMENT_SIZE * STATE_SIZE
  arm_matrix_instance_f32 k_transpose;
  float32_t buf7[MEASUREMENT_SIZE * STATE_SIZE] = {0};
  arm_mat_init_f32(&k_transpose, MEASUREMENT_SIZE, STATE_SIZE, buf7);

  // 6
  // STATE_SIZE * STATE_SIZE
  arm_matrix_instance_f32 m5_T_k_transpose;
  float32_t buf8[STATE_SIZE * STATE_SIZE] = {0};
  arm_mat_init_f32(&m5_T_k_transpose, STATE_SIZE, STATE_SIZE, buf8);

  arm_mat_mult_f32(&K, &H, &K_T_H);
  arm_mat_sub_f32(&Identity_matrix, &K_T_H, &I_S_1);
  arm_mat_trans_f32(&I_S_1, &m2_transpose);
  arm_mat_mult_f32(&I_S_1, &P, &m2_T_P);
  arm_mat_mult_f32(&m2_T_P, &m2_transpose, &m3_T_m2_transpose);
  arm_mat_mult_f32(&K, &R, &K_T_R);
  arm_mat_trans_f32(&K, &k_transpose);
  arm_mat_mult_f32(&K_T_R, &k_transpose, &m5_T_k_transpose);
  arm_mat_add_f32(&m3_T_m2_transpose, &m5_T_k_transpose, &P);
}
void extrapolateCovariance(const arm_matrix_instance_f32 *F)
{
  // 1
  // STATE_SIZE * STATE_SIZE
  arm_matrix_instance_f32 F_T_P;
  float32_t buf1[STATE_SIZE * STATE_SIZE] = {0};
  arm_mat_init_f32(&F_T_P, STATE_SIZE, STATE_SIZE, buf1);
  
  // 2
  // STATE_SIZE * STATE_SIZE
  arm_matrix_instance_f32 m1_T_F_transpose;
  float32_t buf2[STATE_SIZE * STATE_SIZE] = {0};
  arm_mat_init_f32(&m1_T_F_transpose, STATE_SIZE, STATE_SIZE, buf2);
  
  // 3
  // STATE_SIZE * STATE_SIZE
  arm_matrix_instance_f32 F_transpose;
  float32_t buf3[STATE_SIZE * STATE_SIZE] = {0};
  arm_mat_init_f32(&F_transpose, STATE_SIZE, STATE_SIZE, buf3);

  arm_mat_mult_f32(F, &P, &F_T_P);
  arm_mat_trans_f32(F, &F_transpose);
  arm_mat_mult_f32(&F_T_P, &F_transpose, &m1_T_F_transpose);
  arm_mat_add_f32(&m1_T_F_transpose, &Q, &P);
}
void calculateQ(const arm_matrix_instance_f32 *G)
{
  // CONTROL_SIZE * STATE_SIZE
  arm_matrix_instance_f32 G_transpose;
  float32_t buf1[CONTROL_SIZE * STATE_SIZE] = {0};
  arm_mat_init_f32(&G_transpose, CONTROL_SIZE, STATE_SIZE, buf1);

  // STATE_SIZE * CONTROL_SIZE
  arm_matrix_instance_f32 scaled_G;
  float32_t buf2[STATE_SIZE * CONTROL_SIZE] = {0};
  arm_mat_init_f32(&scaled_G, STATE_SIZE, CONTROL_SIZE, buf2);


  arm_mat_trans_f32(G, &G_transpose);
  arm_mat_scale_f32(G, PROCESS_VARIANCE * PROCESS_VARIANCE, &scaled_G);
  arm_mat_mult_f32(&scaled_G, &G_transpose, &Q);
}

void iterateFilter(uint32_t timestamp)
{
  // dt is always recorded in seconds
  float32_t dt = (timestamp - previousTimestamp)/ 1'000'000.0f;
  previousTimestamp = timestamp;

  // State transition matrix
  arm_matrix_instance_f32 F;
  float32_t F_buf[STATE_SIZE * STATE_SIZE] = {F_MATRIX_COEFS(dt)};

  // control matrix
  arm_matrix_instance_f32 G;
  float32_t G_buf[STATE_SIZE * CONTROL_SIZE] = {G_MATRIX_COEFS(dt)};
  arm_mat_init_f32(&F, STATE_SIZE, STATE_SIZE, F_buf);
  arm_mat_init_f32(&G, STATE_SIZE, CONTROL_SIZE, G_buf);

  calculateQ(&G);
  calculateKalmanGain();
  estimateState();
  updateCovariance();
  predict(&F, &G);
  extrapolateCovariance(&F);
}

kalmanState_t getKalmanState()
{

  // copy data so we do not change the actual state on accident
  return (kalmanState_t){
      .position = {
          .x = X.pData[0],
          .y = X.pData[1],
          .z = X.pData[2]},
      .velocity = {
        .x = X.pData[3], 
        .y = X.pData[4], 
        .z = X.pData[5]}};

}
void setKalmanMeasurement(float32_t *data)
{
  memcpy(Z.pData, data, sizeof(Z_buf));
}
void setKalmanControl(float32_t *data)
{
  memcpy(U.pData, data, sizeof(U_buf));
}
