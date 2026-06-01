/****************************************************
Name:       dof2_arm_angle.cpp
Abstract:   ・LDPE2自由度アームの角度制御の実装
           ・キーのそれぞれの対応
            ・r : 全圧力値を0にする。
            ・l : リンクを膨らませる。
            ・s : 初期状態。 
            ・b : 目標角度まで移動。
           ・動かすときは、l,s,b,rの順。
           ・角度は鉛直方向を0度とする．
           ・csvに記録するときはl,s,bの順で押し、一定時間経過後にrを押す。
Author:     B4 Shunsuke Matsui 2026/05/25
update:     2026/05/25 ファイル作成
*****************************************************/


#include "inflatable/program_header/dof2_arm_angle.hpp"

double link_angle_deg = 25; //リンクの膨張角度[deg]
double link_angle = link_angle_deg * M_PI / 180;  //リンクの膨張角度[rad]
double link_length = 0.20;  //リンクの長さ
bool q_dot_flag = true ;
bool joint_stiffness_flag = true;
bool torque_compensate_flag = true;
unsigned int cycle = 0;
double cutoffFrequencyPressure = 100; 
double AngleFB_I_threshold = 30 * M_PI / 180 ;
double c[5] = {-0.00594379, 0.05776778, -0.18364937, 0.19317451, 0}; 

Time timer;
RobotPosition pos;
RobotParameter param;
RobotOrientation orientation;
Pressure pressure;

float positionMarker[THE_NUMBER_OF_MARKERS][3]; 
float positionArm[5][3];  
float positionOrigin[3];   

double voltageInput[AD_CHANNEL_NUMBER];
double voltageOutput[DA_CHANNEL_NUMBER];
double orientationCurrent_raw; 
double orientation_buf;
double orientationCurrent; 
double orientationCurrent2;
double orientationCurrent3;
double q_dot ; 
double orientationTarget; 
double visual_P[DEGREE_OF_FREEDOM];
double visual_I[DEGREE_OF_FREEDOM];
double visual_D[DEGREE_OF_FREEDOM];
double element[3]; 
double P_element[DEGREE_OF_FREEDOM];

double pressureCurrentFiltered[AD_CHANNEL_NUMBER];
double pressureDeviation[DA_CHANNEL_NUMBER];

double pressureKP[DA_CHANNEL_NUMBER];
double pressureKI[DA_CHANNEL_NUMBER];
double pressureKD[DA_CHANNEL_NUMBER];
double pressureP[DA_CHANNEL_NUMBER]; 
double pressureI[DA_CHANNEL_NUMBER]; 
double pressureD[DA_CHANNEL_NUMBER]; 

double basePressure;
double diffPressure;
double linkPressure;
double targetJointStiffness;

bool msgCallback_base_flag;
geometry_msgs::msg::TransformStamped base_pose;
ImpactState impactState[DEGREE_OF_FREEDOM];

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
