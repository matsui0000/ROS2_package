//2自由度関節角度制御のヘッダファイル

#pragma once


#include <stdio.h>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <termios.h>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <sstream>  
#include <sys/stat.h> 

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "inflatable/DataStream_and_contec/DataStreamClient.h"
#include "inflatable/msg/voltage_output.hpp"
#include "inflatable/msg/voltage_input.hpp"


#define DA_CHANNEL_NUMBER 12
#define AD_CHANNEL_NUMBER 8     //=number of pressure sensor
#define LINK_CHANNEL_NUMBER1 8   //膨らませるリンクの番号(8~11が使用可能)
#define LINK_CHANNEL_NUMBER2 9
#define THE_NUMBER_OF_MARKERS 9

#define DA_Conversion_K 0.1     //Voltage = K * Pressure
#define AD_Conversion_K 25.2    //Pressure = K * Voltage + A
#define AD_Conversion_A -25.4
#define SAMPLING_FREQUENCY 200
#define GRAVITY_ACC 9.80665

#define MINIMUM_PRESSURE 0.0F   //アクチュエータの最小圧力[kPa]
#define MAXIMUM_PRESSURE 60.0F  //アクチュエータの最大圧力[kPa]

#define BASE_PRESSURE 30.0F     //基本圧力[kPa]
#define LINK_PRESSURE 4.0F      //リンク膨張用圧力

#define DEGREE_OF_FREEDOM 2     //ロボットアームの自由度

#define FILE_OUTPUT //ファイル出力するならコメントアウト外す
#define GRAPPING_WEIGHT 0.0  //手先に吊り下げる荷重[kg]


//アームの位置制御で追加(20260525)
#define START_ORIENTATION 0.0
#define TARGET_ORIENTATION 20.0

class Time {
public:
    rclcpp::Time now_t;         //Current time
    rclcpp::Duration now_d;     //Current duration time
    rclcpp::Time base;          //Time for current time
    rclcpp::Time buf;           //Time for sampling time
    rclcpp::Duration sampling;  //Sampling time
    rclcpp::Time push_t;        //push 'i' time
    rclcpp::Duration push_d;    //push 'i' duration time from basetime
    Time();
    ~Time();
};

class Direction {
public:
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    Direction();
    ~Direction();   
    //virtual ~Direction(); かも
};

class RobotPosition : public Direction {
public:
    Direction base;
	Direction current;
    Direction buf;
	Direction target;
    Direction deviation;
    RobotPosition();
    ~RobotPosition();
};


class Diviation : public Direction {
public:
    Direction P;             //Diviation
    Direction I;             //Integral(diviation)dt
    Direction D;             //d/dt(diviation)
    Direction P_buf;         //Previous diviation
    std::vector<float> pressure_integral;
    Diviation();
    ~Diviation();
};


class RobotParameter : public Direction {
public:
    Direction force;                            //Force
    Direction force_P;                          //P項によるforce
    Direction force_I;                          //i項によるforce
    Direction force_D;
    float R[3][3];                              //回転行列
    float R_T[3][3];                            //回転行列の転置
    float torque[DEGREE_OF_FREEDOM];            //Torque
    float torque_buf[DEGREE_OF_FREEDOM];        //デバック用
    float torque_g[DEGREE_OF_FREEDOM];          //Compensate gravity torque
    float torque_P[DEGREE_OF_FREEDOM];          //P項によるトルク
    float torque_I[DEGREE_OF_FREEDOM];          //I項によるトルク
    float torque_D[DEGREE_OF_FREEDOM];
    float q3_AngleFB_torque_P;                  //q3の角度制御のP項によるトルク
    float q3_AngleFB_torque_I;                  //q3の角度制御のI項によるトルク
    float torque_g_rate;                        //Compensate gravity torque rate
    float link[DEGREE_OF_FREEDOM + 1];              //Link length
    float link_g[DEGREE_OF_FREEDOM];            //Length between joint and center of gravity
    float link_angle[DEGREE_OF_FREEDOM];        //リンクの膨張角度
    float m_l[DEGREE_OF_FREEDOM];               //Mass of link
    float m_a[DEGREE_OF_FREEDOM];               //Mass of actuator
    float J[2][2];              //Jacobian matrix
    float J_T[2][2];            //Transposed Jacobian matrix
    float target_stiffness[DEGREE_OF_FREEDOM];  //目標関節剛性
    float target_hand_stiff[3];                 //目標手先剛性
    float target_hand_stiff_xz;                 //目標手先剛性（干渉項）
    float target_ellipsoid_axis[3];             //剛性楕円体の軸の大きさ
    float target_ellipsoid_tilt;                //剛性楕円体の傾き（手先座標系のx軸に対する傾き，反時計回りに正）
    RobotParameter();
    ~RobotParameter();
};

class Angle {
public:
    std::vector<float> q;
    Angle();
    ~Angle();
};

class RobotOrientation : public Angle {
public:
    Angle offset;  //Offset angle of pedestal
    Angle hand;    //Hand angle
    Angle current; //Current angle
    Angle buf;     //
    Angle target;  //Target angle
    RobotOrientation();
    ~RobotOrientation();
};

class Pressure {
public:
    float base[DA_CHANNEL_NUMBER];        //Base pressure
    float target[DA_CHANNEL_NUMBER];      //Target pressure
    float target_buf[DA_CHANNEL_NUMBER];  //Previous target pressure
    float current[DA_CHANNEL_NUMBER];     //Current pressure
    float current_buf[DA_CHANNEL_NUMBER]; //Previous current pressure
    float P[DA_CHANNEL_NUMBER];           //Diviation
    float I[DA_CHANNEL_NUMBER];           //int(diviation)dt
    float D[DA_CHANNEL_NUMBER];           //d/dt(diviation)
    float output[DA_CHANNEL_NUMBER];      //Output pressure
    float input[DA_CHANNEL_NUMBER];       //Input pressure
    Pressure();
    ~Pressure();
};




class LowPassFilter {
private:
    // 過去の記憶（内部状態変数）
    double in1_ = 0.0;
    double in2_ = 0.0;
    double out1_ = 0.0;
    double out2_ = 0.0;
    unsigned int cycle_ = 0;

public:
    // コンストラクタ
    LowPassFilter() = default;

    // フィルタ計算関数の「宣言」のみ
    double Filter(double input, double freq);
};





//クラス定義
extern Time timer;
extern RobotPosition pos;
extern RobotParameter param;
extern RobotOrientation orientation;
extern Pressure pressure;

//関節ごとのインパクト駆動制御パラメータ構造体
struct ImpactState {
    bool ImpactInput_flag = false;
    bool PositiveInput_flag = false;
    bool NegativeInput_flag = false;
    int impactDirection = 0; //1:正方向インパクト駆動, -1:負方向インパクト駆動, 0:駆動なし

    unsigned int inputstart_cycle = 0;
    unsigned int inputdown_cycle  = 0;

    unsigned int PositiveImpact_count = 0;
    unsigned int NegativeImpact_count = 0;
    
    double p_h = 0.3;       //pressure height パルスの大きさ[kPa]
    double p_s = 0.05;      //pressure step   変化後のステップの大きさ[kPa]
    double t_w = 0.010;     //time width      パルス幅[s]
    double t_i = 0.2;       //time interval   パルス間隔[s]
    // double position_threshold = 0.002;  //目標位置付近でインパクト駆動を停止する閾値[m]
    double position_threshold = 0.0010;  //インパクト駆動を停止する閾値 [m]
};

extern ImpactState impactState[DEGREE_OF_FREEDOM];


//gain param
extern double pressureKP[DA_CHANNEL_NUMBER];
extern double pressureKI[DA_CHANNEL_NUMBER];
extern double pressureKD[DA_CHANNEL_NUMBER];
extern double visual_P[DEGREE_OF_FREEDOM];
extern double visual_I[DEGREE_OF_FREEDOM];
extern double visual_D[DEGREE_OF_FREEDOM];
extern double basePressure;
extern double diffPressure;
extern double linkPressure;
extern double targetJointStiffness;

extern double link_angle_deg; //リンクの膨張角度[deg]
extern double link_angle;  //リンクの膨張角度[rad]
extern double link_length;  //リンクの長さ


extern double q_dot ;  //qの微分値
extern bool q_dot_flag; //q_dotで場合分けをするかのフラグ

extern bool joint_stiffness_flag; //trueなら関節剛性の補正あり

extern bool torque_compensate_flag;

//ROS time
extern unsigned int cycle;

//sensor
extern double voltageInput[AD_CHANNEL_NUMBER];
extern double voltageOutput[DA_CHANNEL_NUMBER];
extern double pressureCurrentFiltered[AD_CHANNEL_NUMBER]; //ローパスフィルタ後の圧力値

extern double cutoffFrequencyPressure;      //圧力ローパスフィルタのカットオフ周波数[Hz]
extern double pressureDeviation[DA_CHANNEL_NUMBER];//目標値とセンサ値の圧力の偏差[レギュレータ番号]
extern double pressureP[DA_CHANNEL_NUMBER];        //P項による圧力値[レギュレータ番号] 4,3しか使わない
extern double pressureI[DA_CHANNEL_NUMBER];        //I
extern double pressureD[DA_CHANNEL_NUMBER];        //D

extern double orientationTarget;       //目標関節角[rad]
extern double orientationTarget_deg;   //目標関節角[deg]  
extern double orientationCurrent_raw;  //ローパスフィルタなしでの関節角[rad]
extern double orientationCurrent;      //15Hzローパスフィルタでの関節角[rad]
extern double orientationCurrent2;     //10Hzローパスフィルタでの関節角[rad]
extern double orientationCurrent3;     //5Hzローパスフィルタでの関節角[rad]
extern double orientation_buf;         //前ステップの角度
extern double orientationYaw;          //台座部分の角度
extern double AngleFB_I_threshold ; //目標角度と現在の角度が閾値以上ならI項を入れない
extern double P_element[DEGREE_OF_FREEDOM];   //P項で生み出された関節ごとのトルク
extern double I_element[DEGREE_OF_FREEDOM];   //I項で生み出された関節ごとのトルク
extern double D_element[DEGREE_OF_FREEDOM];   //D項で生み出された関節ごとのトルク
// double  c[5] = {-0.007298062, 0.06181, -0.181226594, 0.194224742, 0};   //圧力-トルク変換の特性係数
extern double c[5];   //20250813更新

extern float positionMarker[THE_NUMBER_OF_MARKERS][3];   //マーカーの座標
extern float positionArm[5][3];                                //アームについているマーカーの座標
extern float positionOrigin[3];                                //原点の座標（ベースの中心）
extern float positionTarget[3];                             //目標位置の座標

//関数の宣言
char getch();
int CheckOverPressure_Target(void);
int CheckOverPressure_Output(void);
int DAconversion();
int ADconversion();
void LowPassFilterMarkers();
double FeedbackControl();
void convert_R(const geometry_msgs::msg::TransformStamped& pose, double R[3][3]);
void mul_R(double left[3][3],double right[3][3],double res[3][3]);
void Trans_R(double R[3][3],double res[3][3]);
void print_matrix(double matrix[3][3]);
void print_quartanion(const geometry_msgs::msg::TransformStamped& pose);
void VisualFeedbackControl();
void SetPositionTarget(float x, float y, float z);
void ForwardKinematics(float q1, float q2);
void DetermineImpactDirection();
void DetermineImpactDirection2();
void ImpactDrivenControl(ImpactState& state, int pressure_idx_pos, int pressure_idx_neg);
void PressureFeedbackControl();


void PressureFeedbackReset();
double convertPdf(double q , double torque);
void CompensateGravity();
float GetDistance(float x1, float y1, float z1, float x2, float y2, float z2);






class ArmControlNode : public rclcpp::Node {
public:
    ArmControlNode();

private:
    rclcpp::Publisher<inflatable::msg::VoltageOutput>::SharedPtr ros_pub_;
    rclcpp::Subscription<inflatable::msg::VoltageInput>::SharedPtr ros_sub2_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr ros_sub3_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr ros_sub4_;
    void msgCallback_voltage(const inflatable::msg::VoltageInput::SharedPtr msg);
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

    geometry_msgs::msg::TransformStamped base_pose;
    bool msgCallback_base_flag;

};