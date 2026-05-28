#pragma once


#include <fstream>
#include <cmath>
#include <iomanip>
#include <string>
#include <vector>
#include <sstream>  
#include <sys/stat.h> 


#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>


#include "utils.hpp"
#include "class.hpp"
#include "2dof_header.hpp"
#include "inflatable/DataStreamClient.h"
#include "inflatable/msg/voltage_output.hpp"
#include "inflatable/msg/voltage_input.hpp"

#define FILE_OUTPUT //ファイル出力するならコメントアウト外す

//未使用定数
extern double diffPressure; //使ってない
extern double link_length;  //リンクの長さ
extern double link_angle_deg; //リンクの膨張角度[deg]
extern bool q_dot_flag; //q_dotで場合分けをするかのフラグ
extern bool joint_stiffness_flag; //trueなら関節剛性の補正あり
extern bool torque_compensate_flag;
extern double cutoffFrequencyPressure;      //圧力ローパスフィルタのカットオフ周波数[Hz]
extern double orientationTarget_deg;   //目標関節角[deg] 

//convertPdfで使う定数
extern double link_angle;  //リンクの膨張角度[rad]
//extern double link_angle_deg; //リンクの膨張角度[deg]


//msgcallback_handで使う
extern double q_dot ;  //qの微分値
extern double orientationCurrent_raw;  //ローパスフィルタなしでの関節角[rad]

//pressurefeedback controlで使う
extern double pressureCurrentFiltered[AD_CHANNEL_NUMBER]; //ローパスフィルタ後の圧力値
extern double pressureDeviation[DA_CHANNEL_NUMBER];//目標値とセンサ値の圧力の偏差[レギュレータ番号]
extern double pressureP[DA_CHANNEL_NUMBER];        //P項による圧力値[レギュレータ番号] 4,3しか使わない
extern double pressureI[DA_CHANNEL_NUMBER];        //I
extern double pressureD[DA_CHANNEL_NUMBER];        //D

//visualfeedbackcontrolで使う
extern double orientationTarget;       //目標関節角[rad]


extern double orientationCurrent;      //15Hzローパスフィルタでの関節角[rad]
extern double orientationCurrent2;     //10Hzローパスフィルタでの関節角[rad]
extern double orientationCurrent3;     //5Hzローパスフィルタでの関節角[rad]
extern double orientation_buf;         //前ステップの角度
extern double orientationYaw;          //台座部分の角度
extern double AngleFB_I_threshold ; //目標角度と現在の角度が閾値以上ならI項を入れない
extern double element[3];              //視覚フィードバック制御の各項の値
// double  c[5] = {-0.007298062, 0.06181, -0.181226594, 0.194224742, 0};   //圧力-トルク変換の特性係数
extern double c[5];   //20250813更新

extern float positionMarker[THE_NUMBER_OF_MARKERS][3];   //マーカーの座標
extern float positionArm[5][3];                                //アームについているマーカーの座標
extern float positionOrigin[3];                                //原点の座標（ベースの中心）
extern float positionTarget[3];                             //目標位置の座標




extern double pressureKP[DA_CHANNEL_NUMBER];
extern double pressureKI[DA_CHANNEL_NUMBER];
extern double pressureKD[DA_CHANNEL_NUMBER];
extern double visual_P;
extern double visual_I;
extern double visual_D;




class ArmControlNode : public rclcpp::Node {
public:
    ArmControlNode();

private:
    //クラス定義
    Time timer;
    RobotPosition pos;
    RobotParameter param;
    RobotOrientation orientation;
    Pressure pressure;
    ImpactState impactState[DEGREE_OF_FREEDOM];

    rclcpp::Publisher<inflatable::msg::VoltageOutput>::SharedPtr ros_pub_;
    rclcpp::Subscription<inflatable::msg::VoltageInput>::SharedPtr ros_sub2_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr ros_sub3_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr ros_sub4_;
    void msgCallback2(const inflatable::msg::VoltageInput::SharedPtr msg);
    void msgCallback_hand(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
    void msgCallback_base(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

    bool setup_flag = false;
    bool InputReady_flag = false;
    bool SquareInput_flag = false;
    bool CombinationInput_flag = false;

    bool ImpactDrivenControl_flag = false; //インパクト駆動制御
    bool ImpactInput_flag = false;   //インパクト駆動入力 矩形入力後にoffになり，次の矩形入力までonで待機する

    bool PressureSinWave_flag =false;
    bool AngleSinWave_flag = false;

    bool PressureFeedback_flag = false;
    bool VisualFeedback_flag = false;

    double orientationCurrent_buf;  //繰り返し入力で立ち下がりを行った際の角度[rad]
    double orientation_w_buf;
    double orientation_delta;
    double Pdf;
    double now_time = 0.0;
    double push_time = 0.0;         //ボタンを押した時間[s]
    
    double inputstart_time = 5.0;   //立ち上がり入力開始時間[s]
    double inputdown_time = 0.0;    //立ち下がり入力開始時間[s]

    double sinwave_frequency = 0.5; //sin波の周波数[Hz]
    double sinwave_amplitude = 20;  //sin波の振幅[kPa]

    unsigned int cycle_buf = 0; //サンプリング周期のバッファ
    unsigned int start_cycle = 0;
    unsigned int inputdown_cycle = 0;
    unsigned int interval_cycle = 0;
    unsigned int interval_cycle_buf = 0;
    
    bool is_first = true;

    std::ofstream ofs_;
    void open_log_file();
    void control_loop_P();
    void viconUpdateLoop();

    geometry_msgs::msg::TransformStamped base_pose;
    bool msgCallback_base_flag;

    rclcpp::TimerBase::SharedPtr timer_;
    inflatable::msg::VoltageOutput pub_msg_;
    ViconDataStreamSDK::CPP::Client vicon_client_;

    LowPassFilter LowPassFilter_D1;
    LowPassFilter LowPassFilter_D2;
    LowPassFilter LowPassFilter_D3;
    LowPassFilter LowPassFilter_Pressure1;
    LowPassFilter LowPassFilter_Pressure2;
    LowPassFilter LowPassFilter_Angle0;
    LowPassFilter LowPassFilter_Angle1;
    LowPassFilter LowPassFilter_Angle2;

};