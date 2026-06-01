//関節角度は直線となる姿勢を0度とする．
//0~3のアクチュエータを使用．0,2が膨らむと関節角度が小さくなる方のアクチュエータ．
//q[0]は台座側の関節角度, q[1]は手先側の関節角度．
//q1は台座側の関節角度, q2は手先側の関節角度．

#include "inflatable/program_header/dof2_arm_angle.hpp"
#include "inflatable/DataStream_and_contec/DataStreamClient.h"


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


Diviation::Diviation() {
    this->P.x.emplace_back(9999.0F);
    this->P.y.emplace_back(9999.0F);
    this->P.z.emplace_back(9999.0F);
    this->I.x.emplace_back(0.0F);
    this->I.y.emplace_back(0.0F);
    this->I.z.emplace_back(0.0F);
    this->D.x.emplace_back(0.0F);
    this->D.y.emplace_back(0.0F);
    this->D.z.emplace_back(0.0F);
    this->P_buf.x.emplace_back(0.0F);
    this->P_buf.y.emplace_back(0.0F);
    this->P_buf.z.emplace_back(0.0F);
    for(int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        this->pressure_integral.emplace_back(0.0F);
    }
}
Diviation::~Diviation() {
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
    link{0.210F, 0.160F, 0.160F},   //各リンクの長さ[m] 第一リンクは台座ボルトから関節までの長さ
    //link_g{0.0F, 0.0F, 0.0F, 0.0F},
    link_g{0.0F, 0.0F} //各リンクの重心位置[m] 第一リンクは台座ボルトから重心までの長さ
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

using namespace ViconDataStreamSDK::CPP;
using namespace std::chrono_literals;


void ArmControlNode::viconUpdateLoop(){
    if (vicon_client_.GetFrame().Result != Result::Success) {
            RCLCPP_WARN(this->get_logger(), "Viconフレームの取得に失敗しました。");
            return; 
    }

    for(int i = 0; i < THE_NUMBER_OF_MARKERS; i++) {
        positionMarker[i][0] = 0.0F; positionMarker[i][1] = 0.0F; positionMarker[i][2] = 0.0F;
    }

    unsigned int subject_count = vicon_client_.GetSubjectCount().SubjectCount;

    for(unsigned int i = 0; i < subject_count; i++)
    {
        std::string subject_name = vicon_client_.GetSubjectName(i).SubjectName;
        unsigned int marker_count = vicon_client_.GetMarkerCount(subject_name).MarkerCount;

        for(unsigned int j = 0; j < marker_count; j++)
        {
            std::string marker_name = vicon_client_.GetMarkerName(subject_name, j).MarkerName;
            auto output = vicon_client_.GetMarkerGlobalTranslation(subject_name, marker_name);

            float vx = output.Translation[0] / 1000.0F;
            float vy = output.Translation[1] / 1000.0F;
            float vz = output.Translation[2] / 1000.0F;

            if(marker_name == "base21"){
                positionMarker[0][0] = vx - 0.063F; positionMarker[0][1] = vy; positionMarker[0][2] = vz;
            }
            if(marker_name == "base22"){
                positionMarker[1][0] = vx - 0.063F; positionMarker[1][1] = vy; positionMarker[1][2] = vz;
            }
            if(marker_name == "Hand3"){
                positionMarker[2][0] = vx;         positionMarker[2][1] = vy; positionMarker[2][2] = vz;
            }
            if(marker_name == "Hand4"){
                positionMarker[3][0] = vx;         positionMarker[3][1] = vy; positionMarker[3][2] = vz;
            }
            if(marker_name == "Hand1"){
                positionMarker[4][0] = vx;         positionMarker[4][1] = vy; positionMarker[4][2] = vz;
            }
            if(marker_name == "Hand2"){
                positionMarker[5][0] = vx;         positionMarker[5][1] = vy; positionMarker[5][2] = vz;
            } 
        }
    }

    LowPassFilterMarkers();
    for(int i = 0; i < 3; i++){
        positionOrigin[i] = (positionMarker[0][i] + positionMarker[1][i]) * 0.5F;
    }

     for(int i = 0; i < 4; i++){
        positionArm[i][0] = positionMarker[i+2][0] - positionOrigin[0];
        positionArm[i][1] = positionMarker[i+2][1] - positionOrigin[1];
        positionArm[i][2] = positionMarker[i+2][2] - positionOrigin[2];
    }

    //現在の位置を出していく。
    pos.current.x[3] = (positionArm[2][0] + positionArm[3][0]) / 2.0F;
    pos.current.y[3] = (positionArm[2][1] + positionArm[3][1]) / 2.0F;
    pos.current.z[3] = (positionArm[2][2] + positionArm[3][2]) / 2.0F;

    //positionArm[1](Hand4)が使えないため、pos.current[2]をpositionArm[0]とpositionArm[2]-positionArm[3]のベクトルから求める。
    // 手先側の断面方向（上下方向）
    float nx = positionArm[2][0] - positionArm[3][0];
    float ny = positionArm[2][1] - positionArm[3][1];
    float nz = positionArm[2][2] - positionArm[3][2];
    // positionArm[0] が上側
    pos.current.x[2] = positionArm[0][0] - 0.5F * nx;
    pos.current.y[2] = positionArm[0][1] - 0.5F * ny;
    pos.current.z[2] = positionArm[0][2] - 0.5F * nz;

    //pos.current[3]が手先位置
    float l1 = param.link[0];
    float l2 = param.link[1];
    float l3 = param.link[2];
    float x0 = 0.0F;
    float y0 = 0.0F;
    float z0 = 0.0F;
    float x1 = x0 + l1;
    float y1 = y0;
    float z1 = z0;
    float x2 = pos.current.x[2];
    float y2 = pos.current.y[2];
    float z2 = pos.current.z[2];
    float x3 = pos.current.x[3];
    float y3 = pos.current.y[3];
    float z3 = pos.current.z[3];


    //以下関節角度を出していく。
    // 第1関節角(台座側)
    orientation.buf.q[0] = atan2f(y2 - y1, x2 - x1);
    //q0は(-π/2 , π/2)の範囲, 範囲外の場合は前ステップの値を代入
    if((orientation.buf.q[0] < -M_PI/2.0F) || (orientation.buf.q[0] > M_PI/2.0F)){
        orientation.buf.q[0] = orientation.current.q[0];
    }

    // 第2関節角(手先側)
    orientation.buf.q[1] = atan2f(y3 - y2, x3 - x2) - orientation.buf.q[0];
    //q1は(-π/2 , π/2)の範囲, 範囲外の場合は前ステップの値を代入
    if((orientation.buf.q[1] < -M_PI/2.0F) || (orientation.buf.q[1] > M_PI/2.0F)){
        orientation.buf.q[1] = orientation.current.q[1];
    }

    //計算した関節角度がNANでなければcurrentに格納する。
    for(int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        if(!(std::isnan(orientation.buf.q[i]))) {
            orientation.current.q[i] = orientation.buf.q[i];
        }
        else {
            printf("\e[32m関節角%d=NAN\e[m\n",i);
        }
    }
    
    //目標位置を計算
    // positionTarget[0] = ((positionMarker[6][0] - positionOrigin[0]) + (positionMarker[7][0] - positionOrigin[0])) / 2.0F;
    // positionTarget[1] = ((positionMarker[6][1] - positionOrigin[1]) + (positionMarker[7][1] - positionOrigin[1])) / 2.0F;
    // positionTarget[2] = ((positionMarker[6][2] - positionOrigin[2]) + (positionMarker[7][2] - positionOrigin[2])) / 2.0F;

    // SetPositionTarget(positionTarget[0], positionTarget[1], positionTarget[2]);

    //printf("現在の原点の位置 (%lf, %lf, %lf)\n", positionOrigin[0], positionOrigin[1], positionOrigin[2]);
    //for(int i = 2; i < DEGREE_OF_FREEDOM * 2; i++) {
    //    printf("現在のq%dの位置 (%lf, %lf, %lf)\n", i - 1, pos.current.x[i], pos.current.y[i], pos.current.z[i]);
    //}
    for(int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        printf("現在のq%dの関節角[deg]  %lf\n", i + 1, orientation.current.q[i]*180.0F/M_PI);
    }
    //printf("目標手先位置 (x=%lf, y=%lf, z=%lf)\n", pos.target.x[3], pos.target.y[3], pos.target.z[3]);
    //printf("現在手先位置 (x=%lf, y=%lf, z=%lf)\n", x3, y3, z3);

}

//subscribe voltage
void ArmControlNode::msgCallback_voltage(const inflatable::msg::VoltageInput::SharedPtr sub_msg) {
    for(int i = 0; i < AD_CHANNEL_NUMBER; i++) {
        voltageInput[i] = sub_msg->voltage_input[i];
    }
    ADconversion();
} //msgCallback_voltage()

//アームのサブジェクト
void ArmControlNode::msgCallback_hand(const geometry_msgs::msg::TransformStamped::SharedPtr hand_pose){
    double x,y,z,w;
    double theta; //求めたい手先角度

    double handR[3][3]; //世界座標からhand座標系への回転行列
    double baseR[3][3]; //世界座標からbase座標系への回転行列
    double baseRT[3][3];//baseRの転置行列
    double bhR[3][3];   //baseからhandへの回転行列

    //まだ1回もmsgCallbask_base()が実行されていないなら終了
    if(!(msgCallback_base_flag)) {
        return;
    } 

    orientation_buf = orientationCurrent_raw; //前時間の角度

    //手先のクォータニオンを回転行列に変換
    convert_R(*hand_pose,handR);
    convert_R(base_pose,baseR);

    // Z軸ベクトル同士の内積を計算
    double dot = 0;
    for(int i = 0; i < 3; i++) {
        dot += handR[i][2] * baseR[i][2];
    }

    // Z軸ベクトル同士の外積を計算
    double cross[3];
    cross[0] = baseR[1][2] * handR[2][2] - baseR[2][2] * handR[1][2];
    cross[1] = baseR[2][2] * handR[0][2] - baseR[0][2] * handR[2][2];
    cross[2] = baseR[0][2] * handR[1][2] - baseR[1][2] * handR[0][2];

    // baseのX軸と外積ベクトルの内積を計算
    // ベクトルの正負とスカラー値を取得
    double sign = 0;
    for (int i = 0; i < 3; i++) { 
        sign += cross[i] * baseR[i][0];
    }
    // signが正なら手先角度は正方向、負なら負方向
    // if (sign < 0) {
    //     theta = -theta;
    // }

    // 手先角度 (鉛直方向を0度として左右に±180度ずつ)
    theta = atan2(dot, sign); // atan2(y, x)

    // printf("sign = %lf [rad]\n", sign);
    // printf("dot = %lf [rad]\n", dot);

    orientationCurrent_raw = theta; // -π ~ π [rad]

    //ローパスフィルターに通す
    //関数ごとに前ステップの入力が残っているので関数が分けてある
    orientationCurrent = LowPassFilter_D1.Filter(orientationCurrent_raw, 15.0);
    orientationCurrent2 = LowPassFilter_D2.Filter(orientationCurrent_raw, 10.0);
    orientationCurrent3 = LowPassFilter_D3.Filter(orientationCurrent_raw, 5.0);

    q_dot = (orientationCurrent_raw - orientation_buf) * SAMPLING_FREQUENCY;
} //msgCallback_hand()

//ベースのサブジェクト
void ArmControlNode::msgCallback_base(const geometry_msgs::msg::TransformStamped::SharedPtr pose){
    double x,y,z,w;
    base_pose = *pose;
    msgCallback_base_flag = true;
} //msgCallback_base()

//目標位置、目標関節角度を設定
void SetPositionTarget(float x3, float y3, float z3) {
    //手先目標位置を設定ArmControlNode::S
    pos.target.x[3] = x3;
    pos.target.y[3] = y3;
    pos.target.z[3] = z3;
} //SetPositionTarget()

//順運動学
void ForwardKinematics(float q1, float q2) {
    float l1 = param.link[0];
    float l2 = param.link[1];
    float l3 = param.link[2];

    float position_end_effector[3];
    position_end_effector[0] = l1 + l2 * cosf(q1) + l3 * cosf(q1 + q2);
    position_end_effector[1] = l2 * sinf(q1) + l3 * sinf(q1 + q2);

    pos.buf.x[3] = position_end_effector[0];
    pos.buf.y[3] = position_end_effector[1];
    pos.buf.z[3] = 0.0f;
}

//返り値：視覚フィードバックによるトルク

double VisualFeedbackControl(double target_q, double current_q, int joint_num) {//aaaaa
    static unsigned int FB_cycle[2] = {0, 0}; //フィードバック制御のサイクル数を数えるための変数
    static double orientationIntegral[2] = {0.0F, 0.0F}; //I項の積分値を保存するための変数
    static double orientationTarget_buf[2] = {0.0F, 0.0F}; //前フレームの目標関節角度を保存するための変数
    static double orientationCurrent_buf[2] = {0.0F, 0.0F}; //前フレームの現在関節角度を保存するための変数

    if (FB_cycle[joint_num] > 2) {
        //目標値との差が閾値以下ならI項を入れる
        if(abs(target_q - current_q) < AngleFB_I_threshold){
            orientationIntegral[joint_num] += ((orientationTarget_buf[joint_num] - orientationCurrent_buf[joint_num])
                                    + (orientationTarget[joint_num] - orientationCurrent[joint_num]))
                                    / (2.0F * SAMPLING_FREQUENCY);
        }
    }
        

//elementはファイル出力のためのもの。
    if (FB_cycle[joint_num] > 1) {
            //P component
            param.torque[joint_num] = visual_P[joint_num] * (target_q - current_q);
            P_element[joint_num] = visual_P[joint_num] * (target_q - current_q);

            //I component
            //param.torque[joint_num] += visual_I[joint_num] * orientationIntegral[joint_num];
            I_element[joint_num] = visual_I[joint_num] * orientationIntegral[joint_num];

            //D component
            //param.torque[joint_num] += visual_D[joint_num] * ((orientationTarget_buf[joint_num] - orientationCurrent_buf[joint_num]) - (orientationTarget[joint_num] - orientationCurrent[joint_num])) * SAMPLING_FREQUENCY;
            D_element[joint_num] = visual_D[joint_num] * ((orientationTarget_buf[joint_num] - orientationCurrent_buf[joint_num]) - (orientationTarget[joint_num] - orientationCurrent[joint_num])) * SAMPLING_FREQUENCY;
            joint_torque[joint_num] = P_element[joint_num] + I_element[joint_num] + D_element[joint_num];
    }
    //次のI制御用
    orientationTarget_buf[joint_num] = orientationTarget[joint_num];
    orientationCurrent_buf[joint_num] = orientationCurrent[joint_num];

    FB_cycle[joint_num]++;

    return param.torque[joint_num];
} //VisualFeedbackControl() 


//圧力フィードバック
void PressureFeedbackControl() {
    static unsigned int FB_cycle = 0;
    static double pressureTarget_buf[AD_CHANNEL_NUMBER];
    static double pressureCurrent_buf[AD_CHANNEL_NUMBER];
    static double pressureIntegral[AD_CHANNEL_NUMBER];
    //static double pressureDeviation_buf[DA_CHANNEL_NUMBER]; //前フレームでの偏差
    static bool is_first = true;
    pressureCurrentFiltered[0] = pressure.current[0];
    pressureCurrentFiltered[1] = pressure.current[1];
    pressureCurrentFiltered[2] = pressure.current[2];
    pressureCurrentFiltered[3] = pressure.current[3];

    //変数初期化
    if(is_first){
        for(int i = 0; i < AD_CHANNEL_NUMBER ; i++){
            pressureTarget_buf[i] = 0.0;
            pressureCurrent_buf[i] = 0.0;
            pressureIntegral[i] = 0.0;
            }
        is_first = false;
    }

    //出力値を目標値へ
    for (int i = 0; i < DEGREE_OF_FREEDOM * 2; i++) {
        pressure.output[i] = pressure.target[i];
        //I項のインテグラル計算
        if (FB_cycle > 2) {
            pressureIntegral[i] += ((pressureTarget_buf[i] - pressureCurrent_buf[i])
                                    + (pressure.target[i] - pressureCurrentFiltered[i]))
                                    / (2.0F * SAMPLING_FREQUENCY);
        }
        //PID制御
        if (FB_cycle > 1) {
            //P component
            
            pressureP[i] = pressureKP[i] * (pressure.target[i] - pressureCurrentFiltered[i]);

            //I component
            pressureI[i] = pressureKI[i] * pressureIntegral[i];

            //D component
            //pressureDeviation[i] = pressure.target[i] - pressureCurrentFiltered[i];
            //pressureD[i] = pressureKD[i] * (pressureDeviation[i] - pressureDeviation_buf[i]) * SAMPLING_FREQUENCY;
            
            // pressure.output[i] += pressureKD * (pressureCurrent[i] - pressure.current_buf[i]);
            // pressureD[i] = pressureKD * (pressureCurrent[i] - pressure.current_buf[i]);
            
            //各項を足し合わせる
            //P + I + D
            pressure.output[i] += pressureP[i]; 

            //Update previous value
            pressureTarget_buf[i] = pressure.target[i];
            pressureCurrent_buf[i] = pressureCurrentFiltered[i];
            //pressureDeviation_buf[i] = pressureDeviation[i];
        }
    }
    FB_cycle++;
    return;
} //PressureFeedbackControl()



void PressureFeedbackReset(){
    //PressureFB_cycle = 0;
    //for(int i=0 ; i<DEGREE_OF_FREEDOM*2 ; i++){
    //    diviation.pressure_integral[i] = 0;
    //}
} //PressureFeedbackReset


//クォータニオンを回転行列に変換する。
void convert_R(const geometry_msgs::msg::TransformStamped &pose,double R[3][3]){
    double x,y,z,w;
    x = pose.transform.rotation.x;
    y = pose.transform.rotation.y;
    z = pose.transform.rotation.z;
    w = pose.transform.rotation.w;

    R[0][0] = pow(x,2.0)-pow(y,2.0)-pow(z,2.0)+pow(w,2.0);
    R[0][1] = 2 * (x*y - z*w);
    R[0][2] = 2 * (x*z + y*w);
    
    R[1][0] = 2 * (x*y + z*w);
    R[1][1] = -pow(x,2.0)+pow(y,2.0)-pow(z,2.0)+pow(w,2.0);
    R[1][2] = 2 * (y*z - x*w);

    R[2][0] = 2 * (x*z - y*w);
    R[2][1] = 2 * (y*z + x*w);
    R[2][2] = -pow(x,2.0)-pow(y,2.0)+pow(z,2.0)+pow(w,2.0);
} //convert_R()

//行列の掛け算
void mul_R(double left[3][3],double right[3][3],double res[3][3]){
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            res[i][j] = 0;
            for(int k=0;k<3;k++){
                res[i][j] += left[i][k] * right[k][j];
            }
        }
    }
}//mul_R()

//転置行度列を求める関数。
void Trans_R(double R[3][3],double res[3][3]){
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            res[i][j] = R[j][i];
        }
    }
} //Trans_R()

//行列を表示する関数。(デバック用)
void print_matrix(double matrix[3][3]){
    printf("\n");
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            printf("%8.3lf  ",matrix[i][j]);
        }
        printf("\n");
    }
    printf("\n");
} //print_matrix()

//クォータニオンを表示する関数(デバック用)
void print_quartanion(const geometry_msgs::msg::TransformStamped::SharedPtr pose){
    double x,y,z,w;
    x = pose->transform.rotation.x;
    y = pose->transform.rotation.y;
    z = pose->transform.rotation.z;
    w = pose->transform.rotation.w;
    printf("x=%lf y=%lf x=%lf w=%lf\n\n",x,y,z,w);
} //print_quartanion()

// 圧力目標値のチェック
int CheckOverPressure_Target(void) {
    auto logger = rclcpp::get_logger("inflatable_node");

    for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        if (pressure.target[i] > MAXIMUM_PRESSURE) {
            pressure.target[i] = MAXIMUM_PRESSURE;
            RCLCPP_WARN(logger, "Over maximum pressure %d", i);
        }
        if (pressure.target[i] < MINIMUM_PRESSURE) {
            pressure.target[i] = MINIMUM_PRESSURE;
            RCLCPP_WARN(logger, "Under minimum pressure %d", i);
        }
    }
    return 0;
} //CheckOverPressure_Target()




//電空レギュレータによる制限
int CheckOverPressure_Output(void) {
	for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
		if (pressure.output[i] > MAXIMUM_PRESSURE) {
			pressure.output[i] = MAXIMUM_PRESSURE;
			RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Over maximum pressure %d", i);
		}
        if (pressure.output[i] < MINIMUM_PRESSURE) {
			pressure.output[i] = MINIMUM_PRESSURE;
			RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Under minimum pressure %d", i);
		}
	}
	return 0;
} //CheckOverPressure_Output()

int DAconversion() {
    double k = (double)DA_Conversion_K;
	for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
		voltageOutput[i] = k * pressure.output[i];
    }
	return 0;
}

//Voltage - pressure conversion
int ADconversion() {
    double k = (double)AD_Conversion_K;
    double a = (double)AD_Conversion_A;

	for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {
		pressure.current[i] = k * voltageInput[i] + a;
        if(pressure.current[i] < 0){
            pressure.current[i] = 0;
        }
	}
	return 0;
}

void LowPassFilterMarkers() {
    double samplerate = (double)SAMPLING_FREQUENCY;  //サンプリング周波数
    double freq = 5.0;  //カットオフ周波数
    double q = sqrtf(2.0) / 2.0;    //フィルタのQ値
    double omega = 2.0f * 3.14159265f * freq/samplerate;
    double alpha = sin(omega) / (2.0f * q);
 
    double a0 =  1.0f + alpha;
    double a1 = -2.0f * cos(omega);
    double a2 =  1.0f - alpha;
    double b0 = (1.0f - cos(omega)) / 2.0f;
    double b1 =  1.0f - cos(omega);
    double b2 = (1.0f - cos(omega)) / 2.0f;

    double input[THE_NUMBER_OF_MARKERS][3];
    double output[THE_NUMBER_OF_MARKERS][3];
    static double in1[THE_NUMBER_OF_MARKERS][3], in2[THE_NUMBER_OF_MARKERS][3], out1[THE_NUMBER_OF_MARKERS][3], out2[THE_NUMBER_OF_MARKERS][3]; //0:x, 1:y, 2:z

    for(int i = 0; i < THE_NUMBER_OF_MARKERS; i++){
        for(int j = 0; j < 3; j++){
            input[i][j] = positionMarker[i][j];

            output[i][j] = b0/a0 * input[i][j] + b1/a0 * in1[i][j]  + b2/a0 * in2[i][j]
	                                - a1/a0 * out1[i][j] - a2/a0 * out2[i][j];
            in2[i][j]  = in1[i][j];       // 2つ前の入力信号を更新
	        in1[i][j]  = input[i][j];  // 1つ前の入力信号を更新

	        out2[i][j] = out1[i][j];      // 2つ前の出力信号を更新
	        out1[i][j] = output[i][j]; // 1つ前の出力信号を更新

            positionMarker[i][j] = output[i][j];
        }
    }
}//LowPassFilterMarkers()



char getch() {
    // ROS 2用のロガーを定義
    static auto logger = rclcpp::get_logger("inflatable_node");

    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0) {
        // ROS_ERROR -> RCLCPP_ERROR
        RCLCPP_ERROR(logger, "tcgetattr()"); 
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0) {
        RCLCPP_ERROR(logger, "tcsetattr ICANON");
    }
    
    if(rv == -1) {
        RCLCPP_ERROR(logger, "select");
    }
    else if(rv == 0) {
        // 必要であれば RCLCPP_INFO(logger, "no_key_pressed");
    }
    else {
        read(filedesc, &buff, len );
    }

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0) {
        RCLCPP_ERROR(logger, "tcsetattr ~ICANON");
    }
    return (buff);
} //getch()



//差圧をPdfを返す関数。
double convertPdf(double q , double torque){
    //cは2020年9月に測定したもの。
    //angleは[rad]
    //double torque = 0.0;
    //double torque_g = CompensateGravity();
    double L = M_PI - link_angle;
    double Pdf;
    double A = 3*c[0]*q*pow(L,2.0) + c[0]*pow(q,3.0) + 2*c[1]*q*L + c[2]*q;
    double B = c[0]*pow(L,3.0) + 3*c[0]*L*pow(q,2.0) + c[1]*pow(L,2.0) + c[1]*pow(q,2.0) + c[2]*L + c[3];

    //torqueは[2]に傾く方向が正
    Pdf = (torque - 2 * basePressure * A) / B ;

    return Pdf;
} //convertPdf()


ArmControlNode::ArmControlNode() : Node("dof2_arm_LDPE"){
    // --- Viconサーバーへの接続設定をコンストラクタ内に追記 ---
    std::string vicon_ip = "192.168.4.151:801";
    RCLCPP_INFO(this->get_logger(), "Viconサーバー (%s) に接続中...", vicon_ip.c_str());
    
    vicon_client_.Connect(vicon_ip);

    while (!vicon_client_.IsConnected().Connected && rclcpp::ok())
    {
        RCLCPP_INFO(this->get_logger(), "接続を待機しています...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    if (!rclcpp::ok()) return;
    
    RCLCPP_INFO(this->get_logger(), "Viconに正常に接続されました。");
    printf("Viconサーバーに接続されました。\n"); // デバッグ用
    vicon_client_.EnableMarkerData();
    // -----------------------------------------------------
// 圧力PIDパラメータの取得 (Channel 0)
    pressureKP[0] = this->declare_parameter<double>("pressure_Kp0", 0.0);
    pressureKI[0] = this->declare_parameter<double>("pressure_Ki0", 0.0);
    pressureKD[0] = this->declare_parameter<double>("pressure_Kd0", 0.0);

    // Channel 1
    pressureKP[1] = this->declare_parameter<double>("pressure_Kp1", 0.0);
    pressureKI[1] = this->declare_parameter<double>("pressure_Ki1", 0.0);
    pressureKD[1] = this->declare_parameter<double>("pressure_Kd1", 0.0);

    // Channel 2
    pressureKP[2] = this->declare_parameter<double>("pressure_Kp2", 0.0);
    pressureKI[2] = this->declare_parameter<double>("pressure_Ki2", 0.0);
    pressureKD[2] = this->declare_parameter<double>("pressure_Kd2", 0.0);

    // Channel 3
    pressureKP[3] = this->declare_parameter<double>("pressure_Kp3", 0.0);
    pressureKI[3] = this->declare_parameter<double>("pressure_Ki3", 0.0);
    pressureKD[3] = this->declare_parameter<double>("pressure_Kd3", 0.0);

    // ビジュアルPIDパラメータ
    visual_P[0] = this->declare_parameter<double>("visual_P0", 0.03); //aaaaa
    visual_P[1] = this->declare_parameter<double>("visual_P1", 0.03); //aaaaa
    visual_I[0] = this->declare_parameter<double>("visual_I0", 0.0);
    visual_I[1] = this->declare_parameter<double>("visual_I1", 0.0);
    visual_D[0] = this->declare_parameter<double>("visual_D0", 0.0);
    visual_D[1] = this->declare_parameter<double>("visual_D1", 0.0);

    basePressure = this->declare_parameter<double>("base_pressure", 30.0);
    linkPressure = this->declare_parameter<double>("link_pressure", 4.0);
    targetJointStiffness = this->declare_parameter<double>("target_JointStiffness", 1.20);
    
    // すべて先頭に 「/inflatable/」 を付加し、絶対パス（スラッシュ始まり）にする
    ros_pub_ = this->create_publisher<inflatable::msg::VoltageOutput>("/inflatable/voltage_output", 10);
    ros_sub2_ = this->create_subscription<inflatable::msg::VoltageInput>("/inflatable/voltage_input", 10, std::bind(&ArmControlNode::msgCallback2, this, std::placeholders::_1));
    ros_sub3_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/inflatable/vicon/LDPE_hand/Hand", 10, std::bind(&ArmControlNode::msgCallback_hand, this, std::placeholders::_1));
    ros_sub4_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/inflatable/vicon/LDPE_base/base2", 10, std::bind(&ArmControlNode::msgCallback_base, this, std::placeholders::_1));



    auto period = std::chrono::milliseconds(1000 / SAMPLING_FREQUENCY); 
    // タイマーを作成し、実行する関数を紐付ける
    timer_ = this->create_wall_timer(
        period, 
        std::bind(&ArmControlNode::control_loop_P, this)
    );
    timer.base = this->now();
    timer.buf= this->now();

    //初期化
    for ( int i=0; i<DA_CHANNEL_NUMBER; i++) {
        pressure.output[i] = 0;
    }
    for (int i=0; i<DA_CHANNEL_NUMBER; i++) {
        pressure.target[i] = 0;
    }
#ifdef FILE_OUTPUT
    open_log_file();
#endif
}

void ArmControlNode::open_log_file(){
    char filename[100];
    time_t date_info = time(NULL);
    struct tm *pnow = localtime(&date_info);
    sprintf(filename, "/home/takahara/ros2_ws/data/dof2_arm_angle/%02d%02d_%02d%02d_%02d.csv", 
        pnow->tm_mon + 1, 
        pnow->tm_mday, 
        pnow->tm_hour, 
        pnow->tm_min ,
        pnow->tm_sec);
    ofs_.open(filename);


    ofs_ << "cycle" << ',' << "Time[s]" << ',';

    ofs_ << "q1 target[deg]" << ',' << "q1 current[deg]" << ',' 
        << "q2 target[deg]" << ',' << "q2 current[deg]" << ',';

    ofs_ << "P1 gain" << ',' << "P2 gain" << ',' << "I1 gain" << ','
         << "I2 gain" << ',' << "D1 gain" << ',' << "D2 gain" << ',';

    ofs_ << "q0 torque" << ',' << "q1 torque" << ',';

    ofs_ << "q0 P_torque" << ',' << "q0 I_torque" << ',' << "q0 D_torque" << ','
         << "q1 P_torque" << ',' << "q1 I_torque" << ',' << "q1 D_torque" << ',';


    for (int i = 0; i < DEGREE_OF_FREEDOM * 2; i++) { 
        ofs_ << "Target Pressure" << i << "[kPa]" << ',' 
             << "Output Pressure" << i << "[kPa]" << ',' 
             << "Current Pressure" << i << "[kPa]" << "," 
             << "Current Pressure" << i << "_" << cutoffFrequencyPressure << "Hz" << "[kPa]" << ","
             << "PressureP"<< i << "," 
             << "PressureI" << i << ','
             << "PressureD"<< i << ","; 
    }
    ofs_ << "Link Output1" << ',' << "Link Current1" << ','
         << "Link Output2" << ',' << "Link Current2" << ',';
    for (int i = 0; i < DEGREE_OF_FREEDOM * 2; i++) {
        ofs_ << "Base Pressure" << i << "[kPa]" << ','; 
    }
    

    ofs_ << "x3 target[m]" << ',' << "y3 target[m]" << ',' << "z3 target[m]" << ','
         << "x3[m]" << ',' << "y3[m]" << ',' << "z3[m]" << ','
         << "deviation_x3[m]" << ',' << "deviation_y3[m]" << ',' << "deviation_z3[m]" << ',';
    ofs_ << "x2[m]" << ',' << "y2[m]" << ',' << "z2[m]" << ',';
        
    ofs_ << std::endl;
}

void ArmControlNode::control_loop_P(){
    viconUpdateLoop();
    printf("cycle = %d\n", cycle);
    //圧力を表示
    for (int i = 0; i < 4; i++) {
        printf(" pressure.target[%i] = %lf\n", i, pressure.target[i]);
        printf("pressure.current[%i] = %lf\n", i, pressure.current[i]);
    }
    printf(" linkPressure1.output = %lf\n", pressure.output[LINK_CHANNEL_NUMBER1]);
    printf(" linkPressure2.output = %lf\n", pressure.output[LINK_CHANNEL_NUMBER2]);
    printf("linkPressure1.current = %lf\n", pressure.current[6]);
    printf("linkPressure2.current = %lf\n", pressure.current[7]);

    int key = getch();

    // 各種キー入力
    //全圧力値0
    if (key == 'r') {
        setup_flag  = false;
        PressureFeedback_flag = false;
        VisualFeedback_flag = false;
        is_first = true;
        for (int i=0; i<DA_CHANNEL_NUMBER; i++) {
            pressure.target[i] = 0;
        }
        for (int i=0; i<DA_CHANNEL_NUMBER; i++) {
            pressure.output[i] = 0;
        }
    }
    //リンクを膨らませる
    if (key == 'l') {
        pressure.output[LINK_CHANNEL_NUMBER1] = pressure.base[LINK_CHANNEL_NUMBER1];
        pressure.output[LINK_CHANNEL_NUMBER2] = pressure.base[LINK_CHANNEL_NUMBER2];
    }

    //初期状態
    if(key=='s'){
        setup_flag = false;
        VisualFeedback_flag = false;
        PressureFeedback_flag = true;
        pressure.target[0] = pressure.base[0];
        pressure.target[1] = pressure.base[1];
        pressure.target[2] = pressure.base[2];
        pressure.target[3] = pressure.base[3];
        pressure.output[4] = 0;
        pressure.output[5] = 0;
        pressure.output[6] = 0;
        pressure.output[7] = 0;
    }

    //目標角度まで移動
    if(key=='b'){
        PressureFeedbackReset();
        setup_flag = true;
        VisualFeedback_flag = true;
        PressureFeedback_flag = true;
        // 第1関節角(台座側)
        orientation.target.q[0] = -30.0 / 180 * M_PI;//aaaaa
        // 第2関節角(手先側)
        orientation.target.q[1] = 30.0 / 180 * M_PI;
        //SetPositionTarget(target_x,target_y,target_z);
    }



    // ------------------------------- 以下各種flag --------------------------------------//
    //一回のみ実行される。タイマースタート(rでリセット)
    if(setup_flag && is_first){
        //ROS timeでの現時刻を取得する関数。
        //ros::Time同士の差は時刻ではなく時間になり、ros::Durationという型で表される。
        timer.base = this->now();
        timer.buf = this->now();
        //タイマースタート時のサイクルを保存
        start_cycle = cycle;
        
        is_first = false;
    }

    //タイマースタート(rでリセット)
    if(setup_flag){
        timer.now_t = this->now();
        timer.now_d = timer.now_t - timer.base;
        timer.sampling = timer.now_t - timer.buf;
        timer.buf = timer.now_t;
        now_time = timer.now_d.seconds();
        //時間などを表示
        printf("now_time = %lf\n", now_time);
    }


    if(VisualFeedback_flag){
        orientation.current.q[0] = LowPassFilter_Angle0.Filter(orientation.current.q[0], 5.0); //aaaaa
        orientation.current.q[1] = LowPassFilter_Angle1.Filter(orientation.current.q[1], 5.0);

        for (int i = 0; i < DEGREE_OF_FREEDOM; i++){
            double required_torque = VisualFeedbackControl(orientation.target.q[i], orientation.current.q[i], i);
            double pdf = convertPdf(orientation.target.q[i], required_torque);
            if (i == 0) {
                // 台座側の関節を正の方向（角度が大きくなる方向）に動かすには、ch1を膨らませ、ch0を縮める
                // 正の方向（角度が大きくなる方向）に動かすには ch1 を膨らませ、ch0 を縮める
                pressure.target[0] = basePressure - pdf / 2.0F; // ch0: 膨らむと小さくなる
                pressure.target[1] = basePressure + pdf / 2.0F; // ch1: 膨らむと大きくなる
            } 
            else if (i == 1) {
                // 手先側の関節を正の方向（角度が大きくなる方向）に動かすには、ch3を膨らませ、ch2を縮める
                // 同様に、正の方向に動かすには ch3 を膨らませ、ch2 を縮める
                pressure.target[2] = basePressure - pdf / 2.0F; // ch2: 膨らむと小さくなる
                pressure.target[3] = basePressure + pdf / 2.0F; // ch3: 膨らむと大きくなる
            }
        }

    }

    //s(初期状態)を押し、rを押すまでの間
    if (PressureFeedback_flag) {
        CheckOverPressure_Target();
        PressureFeedbackControl();
        CheckOverPressure_Output();
    }


    
    //圧力電圧変換
    DAconversion();

    //Publish VoltageOutput.msg
    pub_msg_.voltage_output.clear();
    //pub_msg_.voltage_output.shrink_to_fit();    //いらんかも
    for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        pub_msg_.voltage_output.emplace_back(voltageOutput[i]);
    }
    ros_pub_->publish(pub_msg_);

    //File output
#ifdef FILE_OUTPUT
    if(setup_flag){
    ofs_ << cycle << ","<< timer.now_d.seconds() << ',';
 
    ofs_ << orientation.target.q[0] * 180 / M_PI << ',' << orientation.current.q[0] * 180 / M_PI << ','
         << orientation.target.q[1] * 180 / M_PI << ',' << orientation.current.q[1] * 180 / M_PI << ',';    

    ofs_ << visual_P[0] << ',' << visual_P[1] << ',' << visual_I[0] << ',' << visual_I[1] << ',' << visual_D[0] << ',' << visual_D[1] << ',';

    ofs_ << joint_torque[0] << ',' << joint_torque[1] << ',';

    ofs_ << P_element[0] << ',' << I_element[0] << ',' << D_element[0] << ','
         << P_element[1] << ',' << I_element[1] << ',' << D_element[1] << ',';

    
    for (int i = 0; i < DEGREE_OF_FREEDOM * 2; i++) {
        ofs_ << pressure.target[i] << ',' 
             << pressure.output[i] << ',' 
             << pressure.current[i] << ',' 
             << pressureCurrentFiltered[i] << ',' 
             << pressureP[i] << ',' 
             << pressureI[i] << ',' 
             << pressureD[i] << ',';
    }
    ofs_ << pressure.output[LINK_CHANNEL_NUMBER1] << ',' << pressure.current[LINK_CHANNEL_NUMBER1] << ','
         << pressure.output[LINK_CHANNEL_NUMBER2] << ',' << pressure.current[LINK_CHANNEL_NUMBER2] << ',';
    
    for (int i = 0; i < DEGREE_OF_FREEDOM * 2; i++) {
        ofs_ << pressure.base[i] << ','; 
    }
    

    ofs_ << pos.target.x[3] << ',' << pos.target.y[3] << ',' << pos.target.z[3] << ','
         << pos.current.x[3] << ',' << pos.current.y[3] << ',' << pos.current.z[3] << ','
         << pos.deviation.x[3] << ',' << pos.deviation.y[3] << ',' << pos.deviation.z[3] << ',';
    ofs_ << pos.current.x[2] << ',' << pos.current.y[2] << ',' << pos.current.z[2] << ',';
    



    ofs_ << std::endl;
    }
#endif
    cycle++;
}


//返り値：重力によるトルク
void CompensateGravity()
{
    // ===== Joint angles =====
    const float q1 = orientation.current.q[1];
    const float q2 = orientation.current.q[2];

    const float yaw = orientation.current.q[0];

    // ===== Trigonometric values =====
    const float s1 = sinf(q1);
    const float c1 = cosf(q1);

    const float s12 = sinf(q1 + q2);
    const float c12 = cosf(q1 + q2);

    const float cy = cosf(yaw);
    const float sy = sinf(yaw);

    // ===== Total masses =====
    float m1 = param.m_l[1] + param.m_a[1];
    float m2 = param.m_l[2] + param.m_a[2];

    // ===== Link center of gravity =====
    float x_l1 = 0.5F * param.link[1] * s1 * cy;
    float y_l1 = 0.5F * param.link[1] * s1 * sy;
    float z_l1 = 0.5F * param.link[1] * c1;

    float x_l2 = pos.current.x[1]
               + 0.5F * param.link[2] * s12 * cy;

    float y_l2 = pos.current.y[1]
               + 0.5F * param.link[2] * s12 * sy;

    float z_l2 = pos.current.z[1]
               + 0.5F * param.link[2] * c12;

    // ===== Combined center of gravity =====
    float x_g1 = (param.m_l[1] * x_l1 + param.m_a[1] * pos.current.x[1]) / m1;
    float y_g1 = (param.m_l[1] * y_l1 + param.m_a[1] * pos.current.y[1]) / m1;
    float z_g1 = (param.m_l[1] * z_l1 + param.m_a[1] * pos.current.z[1]) / m1;

    float x_g2 = (param.m_l[2] * x_l2 + param.m_a[2] * pos.current.x[2]) / m2;
    float y_g2 = (param.m_l[2] * y_l2 + param.m_a[2] * pos.current.y[2]) / m2;
    float z_g2 = (param.m_l[2] * z_l2 + param.m_a[2] * pos.current.z[2]) / m2;

    // ===== Distance from joint to center of gravity =====
    param.link_g[1] = GetDistance(
        x_g1, y_g1, z_g1,
        pos.current.x[1],
        pos.current.y[1],
        pos.current.z[1]);

    param.link_g[2] = GetDistance(
        x_g2, y_g2, z_g2,
        pos.current.x[2],
        pos.current.y[2],
        pos.current.z[2]);

    // ===== Gravity compensation torque =====

    // Base joint
    param.torque_g[0] = 0.0F;

    // Joint 1
    param.torque_g[1] =
        -(m1 * GRAVITY_ACC * param.link_g[1]
        + m2 * GRAVITY_ACC * param.link[1]) * s1
        -(m2 * GRAVITY_ACC * param.link_g[2]) * s12;

    // Joint 2
    param.torque_g[2] =
        -(m2 * GRAVITY_ACC * param.link_g[2]) * s12;
}

float GetDistance(float x1, float y1, float z1, float x2, float y2, float z2) {
	float distance;
	distance = pow((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1), 0.5F);

	return distance;
}

