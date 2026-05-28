#pragma once
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "utils.hpp"
#include "2dof_header.hpp"


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