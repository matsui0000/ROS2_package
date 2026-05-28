#include "feedbackcontrol.hpp"
#include "utils.hpp"
#include "class.hpp"
#include "2dof_header.hpp"
#include "dof2_arm_angle_control.hpp"
#include <cmath>
#include <stdio.h>


//返り値：視覚フィードバックによるトルク
double VisualFeedbackControl_angle(double target_q, double current_q, int joint_num) {
    static unsigned int FB_cycle[2] = {0, 0}; //フィードバック制御のサイクル数を数えるための変数
    double torque = 0.0;
/*
    if (FB_cycle > 2) {
        //目標値との差が閾値以下ならI項を入れる
        if(abs(orientationTarget - orientationCurrent) < AngleFB_I_threshold){
            orientationIntegral += ((orientationTarget_buf - orientationCurrent_buf)
                                    + (orientationTarget - orientationCurrent))
                                    / (2.0F * SAMPLING_FREQUENCY);
        }
    }
*/

//elementはファイル出力のためのもの。
    if (FB_cycle[joint_num] > 1) {
            //P component
            torque = visual_P * (target_q - current_q);
            element[joint_num] = visual_P * (target_q - current_q);
            //I component
            //torque += visual_I * orientationIntegral;
            //element[1] = visual_I * orientationIntegral;
            //D component
            //torque += visual_D * ((orientationTarget_buf - orientationCurrent_buf) - (orientationTarget - orientationCurrent)) * SAMPLING_FREQUENCY;
            //element[2] = visual_D * ((orientationTarget_buf - orientationCurrent_buf) - (orientationTarget - orientationCurrent)) * SAMPLING_FREQUENCY;

    }
    //次のI制御用
    //orientationTarget_buf = orientationTarget;
    //orientationCurrent_buf = orientationCurrent;

    FB_cycle[joint_num]++;

    return torque;
} //VisualFeedbackControl_angle() 


//まだコピペしただけ
int VisualFB_cycle = 0;
//返り値：視覚フィードバックによるトルク
int VisualFeedbackControl_position(){
    /*
    static unsigned int FB_cycle = 0;
    float Pdf=0.0;

    //P制御
    diviation.P.x[0] = pos.target.x[2] - pos.current.x[2];
    diviation.P.y[0] = pos.target.y[2] - pos.current.y[2];
    diviation.P.z[0] = pos.target.z[2] - pos.current.z[2];

    //I制御
    if (VisualFB_cycle > 2) {
        diviation.I.x[0] += (diviation.P_buf.x[0] + diviation.P.x[0]) / (2.0F * SAMPLING_FREQUENCY);
        diviation.I.y[0] += (diviation.P_buf.y[0] + diviation.P.y[0]) / (2.0F * SAMPLING_FREQUENCY);
        diviation.I.z[0] += (diviation.P_buf.z[0] + diviation.P.z[0]) / (2.0F * SAMPLING_FREQUENCY);
    }
    //次のI制御用に更新
    diviation.P_buf.x[0] = diviation.P.x[0];
    diviation.P_buf.y[0] = diviation.P.y[0];
    diviation.P_buf.z[0] = diviation.P.z[0];


    //Target force
    param.force.x[0] = k.visual_P[0] * diviation.P.x[0] + k.visual_I[0] * diviation.I.x[0];
    param.force.y[0] = k.visual_P[1] * diviation.P.y[0] + k.visual_I[1] * diviation.I.y[0];
    param.force.z[0] = k.visual_P[2] * diviation.P.z[0] + k.visual_I[2] * diviation.I.z[0];

    //ファイル出力用
    element_visualP[0] = k.visual_P[0] * diviation.P.x[0];
    element_visualP[1] = k.visual_P[1] * diviation.P.y[0];
    element_visualP[2] = k.visual_P[2] * diviation.P.z[0];

    element_visualI[0] = k.visual_I[0] * diviation.I.x[0];
    element_visualI[1] = k.visual_I[1] * diviation.I.y[0];
    element_visualI[2] = k.visual_I[2] * diviation.I.z[0];

    //Jacobi matrix
    param.J[0][0] = - sinf(ornt.current.q[0])
                    * (param.link[1]*sinf(ornt.current.q[1]) + param.link[2]*sinf(ornt.current.q[1]+ornt.current.q[2])); 
    
    param.J[0][1] = cosf(ornt.current.q[0]) 
                    * (param.link[1]*cosf(ornt.current.q[1]) + param.link[2]*cosf(ornt.current.q[1]+ornt.current.q[2]));
    
    param.J[0][2] = param.link[2] * cosf(ornt.current.q[0]) * cosf(ornt.current.q[1]+ornt.current.q[2]);

    param.J[1][0] = cosf(ornt.current.q[0]) 
                    * (param.link[1]*sinf(ornt.current.q[1]) + param.link[2]*sinf(ornt.current.q[1]+ornt.current.q[2])); 
    
    param.J[1][1] = sinf(ornt.current.q[0]) 
                    * (param.link[1]*cosf(ornt.current.q[1]) + param.link[2]*cosf(ornt.current.q[1]+ornt.current.q[2]));
    
    param.J[1][2] = param.link[2] * sinf(ornt.current.q[0]) * cosf(ornt.current.q[1]+ornt.current.q[2]);

    param.J[2][0] = 0; 
    
    param.J[2][1] = - param.link[1] * sinf(ornt.current.q[1]) - param.link[2] * sinf(ornt.current.q[1]+ornt.current.q[2]);
    
    param.J[2][2] = - param.link[2] * sinf(ornt.current.q[1]+ornt.current.q[2]);
       
    //Inverse Jacobian matrix
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < DEGREE_OF_FREEDOM; j++) {
            param.J_T[j][i] = param.J[i][j];
            }
        }

    //Force - torque conversion
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        param.torque[i] = param.J_T[i][0] * param.force.x[0]
                        + param.J_T[i][1] * param.force.y[0]
                        + param.J_T[i][2] * param.force.z[0];
    }

    for(int i=0; i < DEGREE_OF_FREEDOM ; i++){
        printf("torque%d=%lf\n",i,param.torque[i]);
    }

    //重力補償トルクを求める。
    CompensateGravity();
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        //param.torque[i] += param.torque_g_rate * param.torque_g[i];
        param.torque[i] += param.torque_g[i];
    }

    float basepressure = 0.0;
    float diffpressure = 0.0;
    for(int i=0 ; i < DEGREE_OF_FREEDOM ; i++){
        //torqueは関節角度が増える方向が正。
        //膨らむと関節角度が増える方がPb+1/2Pdf
        basepressure = JointStiffnessControlBase(param.torque[i] , ornt.current.q[i] , i);
        diffpressure = JointStiffnessControlDiff(param.torque[i] , ornt.current.q[i] , i);

        element_Pdf[i] = diffpressure;
        element_Base[i] = basepressure;

        pressure.target[i*2]   = basepressure + diffpressure / 2.0F;
        pressure.target[i*2+1] = basepressure - diffpressure / 2.0F;
        }

    FB_cycle++;

    VisualFB_cycle++;

    return 0
    */
    ;
} //VisualFeedbackControl_position()



//圧力フィードバック
static int PressureFB_cycle = 0;
void PressureFeedbackControl() {
    static unsigned int FB_cycle = 0;
    //static double pressureTarget_buf[AD_CHANNEL_NUMBER];
    //static double pressureCurrent_buf[AD_CHANNEL_NUMBER];
    //static double pressureIntegral[AD_CHANNEL_NUMBER];
    //static double pressureDeviation_buf[DA_CHANNEL_NUMBER]; //前フレームでの偏差
    //static bool is_first = true;
    //センサ値で得た圧力にローパスフィルタをかける
    // pressureCurrentFiltered[0] = LowPassFilterPressure1(pressure.current[0], cutoffFrequencyPressure);
    // pressureCurrentFiltered[1] = LowPassFilterPressure2(pressure.current[1], cutoffFrequencyPressure);
    // pressureCurrentFiltered[2] = LowPassFilterPressure2(pressure.current[2], cutoffFrequencyPressure);
    // pressureCurrentFiltered[3] = LowPassFilterPressure2(pressure.current[3], cutoffFrequencyPressure);
    pressureCurrentFiltered[0] = pressure.current[0];
    pressureCurrentFiltered[1] = pressure.current[1];
    pressureCurrentFiltered[2] = pressure.current[2];
    pressureCurrentFiltered[3] = pressure.current[3];

    //変数初期化
    //if(is_first){
    //    for(int i = 0; i < AD_CHANNEL_NUMBER ; i++){
    //        pressureTarget_buf[i] = 0.0;
    //        pressureCurrent_buf[i] = 0.0;
    //        pressureIntegral[i] = 0.0;
    //        }
    //    is_first = false;
    //}

    //出力値を目標値へ
    for (int i = 0; i < DEGREE_OF_FREEDOM * 2; i++) {
        pressure.output[i] = pressure.target[i];
        //I項のインテグラル計算
        //if (PressureFB_cycle > 2) {
        //    pressureIntegral[i] += ((pressureTarget_buf[i] - pressureCurrent_buf[i])
        //                            + (pressure.target[i] - pressureCurrentFiltered[i]))
        //                            / (2.0F * SAMPLING_FREQUENCY);
        //}
        //PID制御
        if (PressureFB_cycle > 1) {
            //P component
            //pressureDeviation[i] = pressure.target[i] - pressureCurrentFiltered[i];
            pressureP[i] = pressureKP[i] * (pressure.target[i] - pressureCurrentFiltered[i]);

            //I component
            //pressureI[i] = pressureKI[i] * pressureIntegral[i];

            //D component
            //pressureD[i] = pressureKD[i] * (pressureDeviation[i] - pressureDeviation_buf[i]) * SAMPLING_FREQUENCY;
            
            // pressure.output[i] += pressureKD * (pressureCurrent[i] - pressure.current_buf[i]);
            // pressureD[i] = pressureKD * (pressureCurrent[i] - pressure.current_buf[i]);
            
            //各項を足し合わせる
            //P + I + D
            pressure.output[i] += pressureP[i]; 

            //Update previous value
            //pressureTarget_buf[i] = pressure.target[i];
            //pressureCurrent_buf[i] = pressureCurrentFiltered[i];
            //pressureDeviation_buf[i] = pressureDeviation[i];
        }
    }

    PressureFB_cycle++;
    return;
} //PressureFeedbackControl()


void PressureFeedbackReset(){
    //PressureFB_cycle = 0;
    //for(int i=0 ; i<DEGREE_OF_FREEDOM*2 ; i++){
    //    diviation.pressure_integral[i] = 0;
    //
} //PressureFeedbackReset
