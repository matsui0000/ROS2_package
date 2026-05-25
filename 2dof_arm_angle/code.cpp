#include "inflatable/header.hpp"
#include "inflatable/DataStreamClient.h"


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
    // 第1関節角
    orientation.buf.q[0] = atan2f(y2 - y1, x2 - x1);
    //q0は(-π/2 , π/2)の範囲, 範囲外の場合は前ステップの値を代入
    if((orientation.buf.q[0] < -M_PI/2.0F) || (orientation.buf.q[0] > M_PI/2.0F)){
        orientation.buf.q[0] = orientation.current.q[0];
    }

    // 第2関節角
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

    printf("現在の原点の位置 (%lf, %lf, %lf)\n", positionOrigin[0], positionOrigin[1], positionOrigin[2]);
    for(int i = 2; i < DEGREE_OF_FREEDOM * 2; i++) {
        printf("現在のq%dの位置 (%lf, %lf, %lf)\n", i - 1, pos.current.x[i], pos.current.y[i], pos.current.z[i]);
    }
    for(int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        printf("現在のq%dの関節角[deg]  %lf\n", i + 1, orientation.current.q[i]*180.0F/M_PI);
    }
    printf("目標手先位置 (x=%lf, y=%lf, z=%lf)\n", pos.target.x[3], pos.target.y[3], pos.target.z[3]);
    printf("現在手先位置 (x=%lf, y=%lf, z=%lf)\n", x3, y3, z3);

}

//subscribe voltage
void ArmControlNode::msgCallback2(const inflatable::msg::VoltageInput::SharedPtr sub_msg) {
    for(int i = 0; i < AD_CHANNEL_NUMBER; i++) {
        voltageInput[i] = sub_msg->voltage_input[i];
    }
    ADconversion();
} //msgCallback2()

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
    orientationCurrent = LowPassFilterD1(orientationCurrent_raw, 15.0);
    orientationCurrent2 = LowPassFilterD2(orientationCurrent_raw, 10.0);
    orientationCurrent3 = LowPassFilterD3(orientationCurrent_raw, 5.0);

    q_dot = (orientationCurrent_raw - orientation_buf) * SAMPLING_FREQUENCY;
} //msgCallback_hand()

//ベースのサブジェクト
void ArmControlNode::msgCallback_base(const geometry_msgs::msg::TransformStamped::SharedPtr pose){
    double x,y,z,w;
    base_pose = *pose;
    msgCallback_base_flag = true;
} //msgCallback_base()

//返り値：視覚フィードバックによるトルク
void VisualFeedbackControl(){
    static unsigned int FB_cycle;
    double torque;
    //elementはファイル出力のためのもの。
    torque = visual_P * (orientationTarget - orientationCurrent);
    element[0] = visual_P * (orientationTarget - orientationCurrent);
    FB_cycle++;
    return;
} //VisualFeedbackControl() 


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


//圧力フィードバック
void PressureFeedbackControl() {
    static unsigned int FB_cycle = 0;
    static double pressureDeviation_buf[DA_CHANNEL_NUMBER]; //前フレームでの偏差
    static bool is_first = true;
    pressureCurrentFiltered[0] = pressure.current[0];
    pressureCurrentFiltered[1] = pressure.current[1];
    pressureCurrentFiltered[2] = pressure.current[2];
    pressureCurrentFiltered[3] = pressure.current[3];

    //変数初期化
    if(is_first){
        for(int i = 0; i < AD_CHANNEL_NUMBER ; i++){
            //pressureTarget_buf[i] = 0.0;
            pressureCurrent_buf[i] = 0.0;
            pressureIntegral[i] = 0.0;
            }
        is_first = false;
    }

    //出力値を目標値へ
    for (int i = 0; i < DEGREE_OF_FREEDOM * 2; i++) {
        pressure.output[i] = pressure.target[i];
        //P制御
        pressureDeviation[i] = pressure.target[i] - pressureCurrentFiltered[i];
        pressureP[i] = pressureKP[i] * (pressure.target[i] - pressureCurrentFiltered[i]);
        pressure.output[i] += pressureP[i];
    }
    FB_cycle++;
    return;
} //PressureFeedbackControl()



void PressureFeedbackReset(){
    PressureFB_cycle = 0;
    for(int i=0 ; i<DEGREE_OF_FREEDOM*2 ; i++){
        //diviation.pressure_integral[i] = 0;
    }
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
    printf("\n===== DAconversion =====\n"); //デバッグ用
    printf("DA_Conversion_K = %lf\n", k); //デバッグ用

	for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        printf("[DA BEFORE] ch=%d pressure.output=%lf\n", i, pressure.output[i]); //デバッグ用
		voltageOutput[i] = k * pressure.output[i];
        printf("[DA AFTER ] ch=%d voltageOutput=%lf\n", i, voltageOutput[i]); //デバッグ用
    }
	return 0;
}

//Voltage - pressure conversion
int ADconversion() {
    double k = (double)AD_Conversion_K;
    double a = (double)AD_Conversion_A;

    printf("\n===== ADconversion =====\n");
    printf("AD_Conversion_K = %lf\n", k);
    printf("AD_Conversion_A = %lf\n", a);

	for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {

        printf("[AD INPUT ] ch=%d voltageInput=%lf\n", i, voltageInput[i]); //デバッグ用

		pressure.current[i] = k * voltageInput[i] + a;
        if(pressure.current[i] < 0){

            printf("[WARN] ch=%d pressure.current < 0 : %lf -> 0\n", i, pressure.current[i]); //デバッグ用

            pressure.current[i] = 0;
        }

         printf("[AD OUTPUT] ch=%d pressure.current=%lf\n", i, pressure.current[i]); //デバッグ用

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

double LowPassFilterPressure1(double input, double freq){
    double samplerate = (double)SAMPLING_FREQUENCY;  //サンプリング周波数
    //double freq = 5;  //カットオフ周波数
    double q = sqrtf(2.0) / 2.0;    //フィルタのQ値
    double omega = 2.0f * 3.14159265f * freq/samplerate;
    double alpha = sin(omega) / (2.0f * q);
 
    double a0 =  1.0f + alpha;
    double a1 = -2.0f * cos(omega);
    double a2 =  1.0f - alpha;
    double b0 = (1.0f - cos(omega)) / 2.0f;
    double b1 =  1.0f - cos(omega);
    double b2 = (1.0f - cos(omega)) / 2.0f;

    double output;
    static double in1, in2, out1, out2; //0:x, 1:y, 2:z
    static unsigned int cycle;

    if(std::isnan(input)){
        return input;
    }
    if(cycle < 2){
        output = input;
    }else{
        output = b0/a0 * input + b1/a0 * in1  + b2/a0 * in2
                                - a1/a0 * out1 - a2/a0 * out2;
    }
    in2 = in1;       // 2つ前の入力信号を更新
    in1 = input;  // 1つ前の入力信号を更新

    out2 = out1;      // 2つ前の出力信号を更新
    out1 = output; // 1つ前の出力信号を更新

    cycle++;
    return output;
}//LowPassFilterPressure1()

double LowPassFilterPressure2(double input, double freq){
    double samplerate = (double)SAMPLING_FREQUENCY;  //サンプリング周波数
    //double freq = 5;  //カットオフ周波数
    double q = sqrtf(2.0) / 2.0;    //フィルタのQ値
    double omega = 2.0f * 3.14159265f * freq/samplerate;
    double alpha = sin(omega) / (2.0f * q);
 
    double a0 =  1.0f + alpha;
    double a1 = -2.0f * cos(omega);
    double a2 =  1.0f - alpha;
    double b0 = (1.0f - cos(omega)) / 2.0f;
    double b1 =  1.0f - cos(omega);
    double b2 = (1.0f - cos(omega)) / 2.0f;

    double output;
    static double in1, in2, out1, out2; //0:x, 1:y, 2:z
    static unsigned int cycle;

    if(std::isnan(input)){
        return input;
    }
    if(cycle < 2){
        output = input;
    }else{
        output = b0/a0 * input + b1/a0 * in1  + b2/a0 * in2
                                - a1/a0 * out1 - a2/a0 * out2;
    }
    in2 = in1;       // 2つ前の入力信号を更新
    in1 = input;  // 1つ前の入力信号を更新

    out2 = out1;      // 2つ前の出力信号を更新
    out1 = output; // 1つ前の出力信号を更新

    cycle++;
    return output;
}//LowPassFilterPressure2()

double LowPassFilterD1(double input, double freq){
    double samplerate = (double)SAMPLING_FREQUENCY;  //サンプリング周波数
    //double freq = 5;  //カットオフ周波数
    double q = sqrtf(2.0) / 2.0;    //フィルタのQ値
    double omega = 2.0f * 3.14159265f * freq/samplerate;
    double alpha = sin(omega) / (2.0f * q);
 
    double a0 =  1.0f + alpha;
    double a1 = -2.0f * cos(omega);
    double a2 =  1.0f - alpha;
    double b0 = (1.0f - cos(omega)) / 2.0f;
    double b1 =  1.0f - cos(omega);
    double b2 = (1.0f - cos(omega)) / 2.0f;

    double output;
    static double in1, in2, out1, out2; //0:x, 1:y, 2:z
    static unsigned int cycle;
    if(std::isnan(input)){
        return input;
    }
    if(cycle < 2){
        output = input;
    }else{
        output = b0/a0 * input + b1/a0 * in1  + b2/a0 * in2
                                - a1/a0 * out1 - a2/a0 * out2;
    }
    in2  = in1;       // 2つ前の入力信号を更新
    in1 = input;  // 1つ前の入力信号を更新

    out2 = out1;      // 2つ前の出力信号を更新
    out1 = output; // 1つ前の出力信号を更新

    cycle++;
    return output;
}//LowPassFilterD1()

//input->output freq=cutoff frequency
double LowPassFilterD2(double input, double freq){
    double samplerate = (double)SAMPLING_FREQUENCY;  //サンプリング周波数
    //double freq = 5;  //カットオフ周波数
    double q = sqrtf(2.0) / 2.0;    //フィルタのQ値
    double omega = 2.0f * 3.14159265f * freq/samplerate;
    double alpha = sin(omega) / (2.0f * q);
 
    double a0 =  1.0f + alpha;
    double a1 = -2.0f * cos(omega);
    double a2 =  1.0f - alpha;
    double b0 = (1.0f - cos(omega)) / 2.0f;
    double b1 =  1.0f - cos(omega);
    double b2 = (1.0f - cos(omega)) / 2.0f;

    double output;
    static double in1, in2, out1, out2; //0:x, 1:y, 2:z
    static unsigned int cycle;
    if(std::isnan(input)){
        return input;
    }
    if(cycle < 2){
        output = input;
    }else{
        output = b0/a0 * input + b1/a0 * in1  + b2/a0 * in2
                                - a1/a0 * out1 - a2/a0 * out2;
    }
    in2  = in1;       // 2つ前の入力信号を更新
    in1 = input;  // 1つ前の入力信号を更新

    out2 = out1;      // 2つ前の出力信号を更新
    out1 = output; // 1つ前の出力信号を更新

    cycle++;
    return output;
}//LowPassFilterD2()

//input->output freq=cutoff frequency
double LowPassFilterD3(double input, double freq){
    double samplerate = (double)SAMPLING_FREQUENCY;  //サンプリング周波数
    //double freq = 5;  //カットオフ周波数
    double q = sqrtf(2.0) / 2.0;    //フィルタのQ値
    double omega = 2.0f * 3.14159265f * freq/samplerate;
    double alpha = sin(omega) / (2.0f * q);
 
    double a0 =  1.0f + alpha;
    double a1 = -2.0f * cos(omega);
    double a2 =  1.0f - alpha;
    double b0 = (1.0f - cos(omega)) / 2.0f;
    double b1 =  1.0f - cos(omega);
    double b2 = (1.0f - cos(omega)) / 2.0f;

    double output;
    static double in1, in2, out1, out2; //0:x, 1:y, 2:z
    static unsigned int cycle;
    if(std::isnan(input)){
        return input;
    }
    if(cycle < 2){
        output = input;
    }else{
        output = b0/a0 * input + b1/a0 * in1  + b2/a0 * in2
                                - a1/a0 * out1 - a2/a0 * out2;
    }
    in2  = in1;       // 2つ前の入力信号を更新
    in1 = input;  // 1つ前の入力信号を更新

    out2 = out1;      // 2つ前の出力信号を更新
    out1 = output; // 1つ前の出力信号を更新

    cycle++;
    return output;
}//LowPassFilterD3()



double LowPassFilterAngle0(double input, double freq) {
    double samplerate = (double)SAMPLING_FREQUENCY;  //サンプリング周波数
    //double freq = 5;  //カットオフ周波数
    double q = sqrtf(2.0) / 2.0;    //フィルタのQ値
    double omega = 2.0f * 3.14159265f * freq/samplerate;
    double alpha = sin(omega) / (2.0f * q);
 
    double a0 =  1.0f + alpha;
    double a1 = -2.0f * cos(omega);
    double a2 =  1.0f - alpha;
    double b0 = (1.0f - cos(omega)) / 2.0f;
    double b1 =  1.0f - cos(omega);
    double b2 = (1.0f - cos(omega)) / 2.0f;

    double output;
    static double in1, in2, out1, out2; //0:x, 1:y, 2:z
    static unsigned int cycle;
    if(std::isnan(input)){
        return input;
    }
    if(cycle < 2){
        output = input;
    }else{
        output = b0/a0 * input + b1/a0 * in1  + b2/a0 * in2
                                - a1/a0 * out1 - a2/a0 * out2;
    }
    in2  = in1;       // 2つ前の入力信号を更新
    in1 = input;  // 1つ前の入力信号を更新

    out2 = out1;      // 2つ前の出力信号を更新
    out1 = output; // 1つ前の出力信号を更新
    cycle++;
    return output;
}//LowPassFilterAngle0()



double LowPassFilterAngle1(double input, double freq) {
    double samplerate = (double)SAMPLING_FREQUENCY;  //サンプリング周波数
    //double freq = 5;  //カットオフ周波数
    double q = sqrtf(2.0) / 2.0;    //フィルタのQ値
    double omega = 2.0f * 3.14159265f * freq/samplerate;
    double alpha = sin(omega) / (2.0f * q);
 
    double a0 =  1.0f + alpha;
    double a1 = -2.0f * cos(omega);
    double a2 =  1.0f - alpha;
    double b0 = (1.0f - cos(omega)) / 2.0f;
    double b1 =  1.0f - cos(omega);
    double b2 = (1.0f - cos(omega)) / 2.0f;

    double output;
    static double in1, in2, out1, out2; //0:x, 1:y, 2:z
    static unsigned int cycle;
    if(std::isnan(input)){
        return input;
    }
    if(cycle < 2){
        output = input;
    }else{
        output = b0/a0 * input + b1/a0 * in1  + b2/a0 * in2
                                - a1/a0 * out1 - a2/a0 * out2;
    }
    in2  = in1;       // 2つ前の入力信号を更新
    in1 = input;  // 1つ前の入力信号を更新

    out2 = out1;      // 2つ前の出力信号を更新
    out1 = output; // 1つ前の出力信号を更新
    cycle++;
    return output;
}//LowPassFilterAngle1()

double LowPassFilterAngle2(double input, double freq) {
    double samplerate = (double)SAMPLING_FREQUENCY;  //サンプリング周波数
    //double freq = 5;  //カットオフ周波数
    double q = sqrtf(2.0) / 2.0;    //フィルタのQ値
    double omega = 2.0f * 3.14159265f * freq/samplerate;
    double alpha = sin(omega) / (2.0f * q);
 
    double a0 =  1.0f + alpha;
    double a1 = -2.0f * cos(omega);
    double a2 =  1.0f - alpha;
    double b0 = (1.0f - cos(omega)) / 2.0f;
    double b1 =  1.0f - cos(omega);
    double b2 = (1.0f - cos(omega)) / 2.0f;

    double output;
    static double in1, in2, out1, out2; //0:x, 1:y, 2:z
    static unsigned int cycle;
    if(std::isnan(input)){
        return input;
    }
    if(cycle < 2){
        output = input;
    }else{
        output = b0/a0 * input + b1/a0 * in1  + b2/a0 * in2
                                - a1/a0 * out1 - a2/a0 * out2;
    }
    in2  = in1;       // 2つ前の入力信号を更新
    in1 = input;  // 1つ前の入力信号を更新

    out2 = out1;      // 2つ前の出力信号を更新
    out1 = output; // 1つ前の出力信号を更新

    cycle++;
    return output;
}//LowPassFilterAngle2()

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
    visual_P = this->declare_parameter<double>("visual_P", 1.0);
    visual_I = this->declare_parameter<double>("visual_I", 1.0);
    visual_D = this->declare_parameter<double>("visual_D", 0.0);

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
    sprintf(filename, "/home/takahara/ros2_ws/src/inflatable/data/dof2_impact/%02d%02d_%02d%02d_%02d.csv", 
        pnow->tm_mon + 1, 
        pnow->tm_mday, 
        pnow->tm_hour, 
        pnow->tm_min ,
        pnow->tm_sec);
    ofs_.open(filename);


    ofs_ << "cycle" << ',' << "Time[s]" << ',';
    ofs_ << "x3 target[m]" << ',' << "y3 target[m]" << ',' << "z3 target[m]" << ','
        << "x3[m]" << ',' << "y3[m]" << ',' << "z3[m]" << ','
        << "deviation_x3[m]" << ',' << "deviation_y3[m]" << ',' << "deviation_z3[m]" << ',';
    ofs_ << "x2[m]" << ',' << "y2[m]" << ',' << "z2[m]" << ',';
    ofs_ << "q1 target[deg]" << ',' << "q1 current[deg]" << ',' 
        << "q2 target[deg]" << ',' << "q2 current[deg]" << ',';
    for (int i = 0; i < DEGREE_OF_FREEDOM * 2; i++) { 
        ofs_ << "Target Pressure" << i << "[kPa]" << ',' << "Output Pressure" << i << "[kPa]" << ',' << "Current Pressure" << i << "[kPa]" << "," << "Current Pressure" << i << "_" << cutoffFrequencyPressure << "Hz" << "[kPa]" << ","
            << "PressureP"<< i << "," << "PressureI" << i << ',' << "PressureD"<< i << ","; 
    }
    ofs_ << "Link Output1" << ',' << "Link Current1" << ','
        << "Link Output2" << ',' << "Link Current2" << ',';
    for (int i = 0; i < DEGREE_OF_FREEDOM * 2; i++) {
        ofs_ << "Base Pressure" << i << "[deg]" << ','; 
    }
    ofs_ << "visualP " << visual_P << ',' << "visualI " << visual_I << ',' << "visualD " << visual_D << ',';
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        ofs_ << "PositiveImpact_count_q" << i + 1 << ',' << "NegativeImpact_count_q" << i + 1 << ',';
        ofs_ << "inputstart_cycle_q" << i + 1 << ',' << "inputdown_cycle_q" << i + 1 << ',';
    }
    // ここから中身なし列
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        ofs_ << "q" << i + 1 << "p_h[kPa]" << impactState[i].p_h << ','
            << "q" << i + 1 << "p_s[kPa]" << impactState[i].p_s << ','
            << "q" << i + 1 << "t_w[s]" << impactState[i].t_w << ','
            << "q" << i + 1 << "t_i[s]" << impactState[i].t_i << ','
            << "q" << i + 1 << "position_threshold[m]" << impactState[i].position_threshold << ',';
    }
    for (int i = 0; i < DEGREE_OF_FREEDOM * 2; i++) {
        ofs_ << "PressureKP" << i << " " << pressureKP[i] << ',' 
            << "PressureKI" << i << " " << pressureKI[i] << ',' 
            << "PressureKD" << i << " " << pressureKD[i] << ',';
    }
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
        setup_flag  = false; //いる
        PressureFeedback_flag = false; //いる
        VisualFeedback_flag = false; //いる
        is_first = true; //いる
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
        setup_flag = true;
        VisualFeedback_flag = true;
        PressureFeedbackReset();
        PressureFeedback_flag = true;
        orientation.target.q[0] = -30.0 / 180 * M_PI;
        orientation.target.q[1] = -30.0 / 180 * M_PI;
        //SetPositionTarget(target_x,target_y,target_z);
    }



    ////////////////////////////////////////////////以下各種flag//////////////////////////////////////////////////////
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
        orientation.current.q[0] = LowPassFilterAngle0(orientation.current.q[0], 5.0);
        orientation.current.q[1] = LowPassFilterAngle1(orientation.current.q[1], 5.0);
        VisualFeedbackControl();
        torque -= CompensateGravity();
        torque += VisualFeedbackControl();
        Pdf = convertPdf(orientationTarget,torque);
        printf("Pdf=%lf\n",Pdf);
        pressureTarget[2] = basePressure - Pdf / 2.0F ;
        pressureTarget[3] = basePressure + Pdf / 2.0F ;
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
    pub_msg_.voltage_output.shrink_to_fit();    //いらんかも
    for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        pub_msg_.voltage_output.emplace_back(voltageOutput[i]);
    }
    ros_pub_->publish(pub_msg_);

    //File output
#ifdef FILE_OUTPUT
    if(setup_flag){
    ofs_ << cycle << ","<< timer.now_d.seconds() << ',';
    ofs_ << pos.target.x[3] << ',' << pos.target.y[3] << ',' << pos.target.z[3] << ','
        << pos.current.x[3] << ',' << pos.current.y[3] << ',' << pos.current.z[3] << ','
        << pos.deviation.x[3] << ',' << pos.deviation.y[3] << ',' << pos.deviation.z[3] << ',';
    ofs_ << pos.current.x[2] << ',' << pos.current.y[2] << ',' << pos.current.z[2] << ',';
    ofs_ << orientation.target.q[0] * 180 / M_PI << ',' << orientation.current.q[0] * 180 / M_PI << ','
        << orientation.target.q[1] * 180 / M_PI << ',' << orientation.current.q[1] * 180 / M_PI << ',';
    for (int i = 0; i < DEGREE_OF_FREEDOM * 2; i++) {
        ofs_ << pressure.target[i] << ',' << pressure.output[i] << ',' << pressure.current[i] << ',' << pressureCurrentFiltered[i] << ',' << pressureP[i] << ',' << pressureI[i] << ',' << pressureD[i] << ',';
    }
    ofs_ << pressure.output[LINK_CHANNEL_NUMBER1] << ',' << pressure.current[LINK_CHANNEL_NUMBER1] << ','
        << pressure.output[LINK_CHANNEL_NUMBER2] << ',' << pressure.current[LINK_CHANNEL_NUMBER2] << ',';
    for (int i = 0; i < DEGREE_OF_FREEDOM * 2; i++) {
        ofs_ << pressure.base[i] << ','; 
    }
    ofs_ << element[0] << ',' << element[1] << ',' << element[2] << ',';
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        ofs_ << impactState[i].PositiveImpact_count << ',' << impactState[i].NegativeImpact_count << ',';
        ofs_ << impactState[i].inputstart_cycle << ',' << impactState[i].inputdown_cycle << ',';
    }
    ofs_ << std::endl;
    }
#endif
    cycle++;
}


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