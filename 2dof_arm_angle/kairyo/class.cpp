#include "class.hpp"
#include "2dof_header.hpp"
#include "utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <stdio.h>


Time::Time():
  now_d(0, 0), //add
  sampling(0, 0), //add
  push_d(0, 0) //add
{
}
Time::~Time() {
}

Direction::Direction() {
}
Direction::~Direction() {
}

RobotPosition::RobotPosition() {
    this->base.x.emplace_back(0.0F);
    this->base.y.emplace_back(0.0F);
    this->base.z.emplace_back(0.0F);
    for (int i = 0; i < DEGREE_OF_FREEDOM * 2; i++) {
        this->current.x.emplace_back(0.0F);
        this->current.y.emplace_back(0.0F);
        this->current.z.emplace_back(0.0F);
        this->target.x.emplace_back(0.0F);
        this->target.y.emplace_back(0.0F);
        this->target.z.emplace_back(0.0F);
        this->buf.x.emplace_back(0.0F);
        this->buf.y.emplace_back(0.0F);
        this->buf.z.emplace_back(0.0F);
        this->deviation.x.emplace_back(0.0F);
        this->deviation.y.emplace_back(0.0F);
        this->deviation.z.emplace_back(0.0F);
    }
}
RobotPosition::~RobotPosition() {
}

RobotParameter::RobotParameter():
    torque_g_rate(0.5F),  //重力補償トルクのゲイン
    //link{ 0.22F, 0.130F, 0.155F},
    //link{ 0.215F, 0.160F, 0.160F, 0.140F},
    link{0.210F, 0.160F, 0.160F}   //各リンクの長さ[m] 第一リンクは台座ボルトから関節までの長さ
    // link_g{0.0F, 0.0F, 0.0F, 0.0F},
    // link_angle{80.0F * M_PI / 180.0F , 35.0F * M_PI / 180.0F , 35.0F * M_PI / 180.0F , 35.0F * M_PI / 180.0F },
    // m_l{0.015F, 0.055F,0.055F, 0.055F+0.005F*5/*+0.20*/},
    // m_a{0.010F, 0.028F, 0.028, 0.028F+GRAPPING_WEIGHT},
    // //target_stiffness{0.5F ,0.5F , 0.6F , 0.4F},
    // target_stiffness{2.00F , 1.00F , 1.00F , 1.00F},
    // target_hand_stiff{0, 0, 0},
    // target_hand_stiff_xz{0.0F},
    // //target_ellipsoid_axis{7.0F , 15.0F , 79.8F},
    // target_ellipsoid_axis{4.0F , 15.0F , 79.8F},
    // target_ellipsoid_tilt{37 * M_PI / 180.0F}
{
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        this->force.x.emplace_back(0.0F);
        this->force.y.emplace_back(0.0F);
        this->force.z.emplace_back(0.0F);
        this->force_P.x.emplace_back(0.0F);
        this->force_P.y.emplace_back(0.0F);
        this->force_P.z.emplace_back(0.0F);
        this->force_I.x.emplace_back(0.0F);
        this->force_I.y.emplace_back(0.0F);
        this->force_I.z.emplace_back(0.0F);
        this->force_D.x.emplace_back(0.0F);
        this->force_D.y.emplace_back(0.0F);
        this->force_D.z.emplace_back(0.0F);
        this->torque[i] = 0.0F;
        this->torque_g[i] = 0.0F;
        this->torque_P[i] = 0.0F;
        this->torque_I[i] = 0.0F;
        for (int j = 0; j < 2; j++) {
            this->J[j][i] = 0.0F;
            this->J_T[i][j] = 0.0F;
        }
    }
}
RobotParameter::~RobotParameter() {
}

Angle::Angle() {
}
Angle::~Angle() {
}

RobotOrientation::RobotOrientation() {
    this->offset.q.emplace_back(0.0F);
    this->hand.q.emplace_back(0.0F);
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        this->current.q.emplace_back(0.0F);
        this->buf.q.emplace_back(0.0F);
        this->target.q.emplace_back(0.0F);
    }
}
RobotOrientation::~RobotOrientation() {
}

Pressure::Pressure():
base {  BASE_PRESSURE,  //0
        BASE_PRESSURE,  //1
        BASE_PRESSURE - 10.0F,  //2
        BASE_PRESSURE + 10.0F,  //3
        0.0F,           //4
        0.0F,           //5
        0.0F,           //6
        0.0F,           //7
        LINK_PRESSURE,  //8
        LINK_PRESSURE-1,//9
        0.0F,           //10
        0.0F            //11
}
{
    for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        this->target[i] = 0.0F;
        this->target_buf[i] = 0.0F;
        this->current[i] = 0.0F;
        this->current_buf[i] = this->base[i];
        this->P[i] = 0.0F;
        this->I[i] = 0.0F;
        this->D[i] = 0.0F;
        this->output[i] = 0.0F;
        this->input[i] = 0.0F;
    }
}
Pressure::~Pressure() {
}

double LowPassFilter::Filter(double input, double freq) {
    // 1. 無効な値 (NaN) のチェック
    if (std::isnan(input)) {
        return input;
    }

    // 2. フィルタ係数の計算
    double samplerate = (double)SAMPLING_FREQUENCY;
    double q = sqrt(2.0) / 2.0f;
    double omega = 2.0f * M_PI * freq / samplerate;
    double alpha = sin(omega) / (2.0f * q);

    double a0 =  1.0f + alpha;
    double a1 = -2.0f * cos(omega);
    double a2 =  1.0f - alpha;
    double b0 = (1.0f - cos(omega)) / 2.0f;
    double b1 =  1.0f - cos(omega);
    double b2 = (1.0f - cos(omega)) / 2.0f;

    // 3. フィルタリング処理
    double output;
    if (cycle_ < 2) {
        output = input;
    } else {
        output = b0/a0 * input + b1/a0 * in1_  + b2/a0 * in2_
                                - a1/a0 * out1_ - a2/a0 * out2_;
    }

    // 4. 過去の記憶（状態）の更新
    in2_ = in1_;
    in1_ = input;
    out2_ = out1_;
    out1_ = output;
    cycle_++;

    return output;
}