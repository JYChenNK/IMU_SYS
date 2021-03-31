#ifndef __FILTER_H
#define __FILTER_H

#include <BasicLinearAlgebra.h>
#include "Arduino.h"
#include "math.h"

float dt      = 0.005;
float Q_angle = 0.001;
float Q_gyro  = 0.003;
float R_angle = 0.5;
float K1      = 0.05;

float Angle_Kalman, Bias_Kalman, Angle_FOCF;

#define Matrixs 1

#if Matrixs

// All the functions in BasicLinearAlgebra are wrapped up inside the namespace BLA, so specify that we're using it like so:
using namespace BLA;

BLA::Matrix<2,1> X= {0,
                      0};
BLA::Matrix<1,1> Y = {0};
BLA::Matrix<1,1> U = {0};
BLA::Matrix<2,2> A = {1,dt,
                      0,1};
BLA::Matrix<2,1> B = {dt,
                      0};
BLA::Matrix<1,2> C = {1,0};

BLA::Matrix<2,2> P = {0.1,0.1,
                      0.1,0.1};
BLA::Matrix<2,1> K= {0.1,
                     0.1};
BLA::Matrix<2,2> Q= {Q_angle,0,
                      0,Q_gyro};
BLA::Matrix<1,1> R= {R_angle};


void Kalman_Filter(float Gyro, float Accel_Angle)
{
  U(0,0) = Gyro;
  Y(0,0) = Accel_Angle;
  
  X = A*X + B*U;
  P = A*P*~A + Q;
  K = P*~C*((C*P*~C+R).Inverse());
  X += K*(Y - C*X);
  P -= K*C*P;

  Angle_Kalman = X(0, 0), Bias_Kalman = X(1, 0);
}

#else 

float Angle_X_Final, Q_bias_x, Angle_err_x;
float PCt_0, PCt_1, K_0, K_1, t_0, t_1, C_0=1, E;
float Pdot[4] = {0.1,0.1,
                    0.1,0.1};
float PP[2][2]   = {0.1,0.1,
                    0.1,0.1};


void Kalman_Filter(float Gyro, float Accel)
{
  Angle_X_Final  += (Gyro - Q_bias_x) * dt;       //先验估计
 
  Pdot[0] = Q_angle - PP[0][1] - PP[1][0];     //Pk-先验估计误差协方差的微分
  Pdot[1] = -PP[1][1];
  Pdot[2] = -PP[1][1];
  Pdot[3] = Q_gyro;
 
  PP[0][0] += Pdot[0] * dt;   //Pk-先验估计误差协方差的微分的积分=先验估计误差协方差
  PP[0][1] += Pdot[1] * dt;   
  PP[1][0] += Pdot[2] * dt;
  PP[1][1] += Pdot[3] * dt;
 
  Angle_err_x = Accel - Angle_X_Final;  //zk-先验估计
 
  PCt_0 = C_0 * PP[0][0];
  PCt_1 = C_0 * PP[1][0];
 
  E = R_angle + C_0 * PCt_0;
 
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
 
  t_0 = PCt_0;
  t_1 = C_0 * PP[0][1];
 
  PP[0][0] -= K_0 * t_0;     //后验估计误差协方差
  PP[0][1] -= K_0 * t_1;
  PP[1][0] -= K_1 * t_0;
  PP[1][1] -= K_1 * t_1;
 
  Angle_X_Final += K_0 * Angle_err_x; //后验估计
  Q_bias_x += K_1*Angle_err_x;
  
  Angle_Kalman = Angle_X_Final, Bias_Kalman = Q_bias_x;
}


#endif


void Fist_Older_Complementary_Filter(float Gyro, float Accel_Angle)
{
  Angle_FOCF = K1*Accel_Angle + (1 - K1)*(Angle_FOCF + Gyro * dt);
}


#endif
