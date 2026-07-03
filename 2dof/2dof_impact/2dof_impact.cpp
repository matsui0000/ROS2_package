/****************************************************
Name:       2dof_impact.cpp
Abstract:   ・2dof_impact.cppからコピー
            ・LDPE2自由度アームへのインパクト駆動制御の適用
            ・クラス構造などを4dof_arm2.cppを参考に作成
            ・キーのそれぞれの対応
            ・r : 全圧力値を0にする。
            ・l : リンクを膨らませる。
            ・j : 初期状態。(タイマーなし)
            ・i : 押下後インパクト駆動制御開始
            (以下のキーは基本的に使用しない)
            ・s : 初期状態。(タイマーあり)
            ・y : 圧力値正弦波入力
            ・h : 関節角度正弦波入力
            ・動かすときは、l,j,i,rの順。
            ・関節角度は直線となる姿勢を0度とする．
            ・0~3のアクチュエータを使用．0,2が膨らむと関節角度が小さくなる方のアクチュエータ．
Author:     M2 Takumi Takahara 2026/01/15
update:     2026/01/15 ファイル作成
*****************************************************/


#include "inflatable/program_header/2dof_impact.hpp"

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
double visual_P;
double visual_I;
double visual_D;
double element[3]; 

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
