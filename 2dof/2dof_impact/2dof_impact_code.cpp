#include "inflatable/program_header/2dof_impact.hpp"
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

            if(marker_name == "base1"){
                positionMarker[0][0] = vx - 0.063F; positionMarker[0][1] = vy; positionMarker[0][2] = vz;
            }
            if(marker_name == "base2"){
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

/*
//マーカー座標から現在の関節角度を求める。
void ArmControlNode::msgCallback( const mocap4r2_msgs::msg::Markers_<std::allocator<void>>::SharedPtr markers_msg){//need-shusei
    unsigned int count = 0;
    double orientationBuf;

    for(int i=0 ; i < THE_NUMBER_OF_MARKERS ; i++){
        if(markers_msg->markers[i].marker_name == "Base1"){
            positionMarker[0][0] = markers_msg->markers[i].translation.x / 1000.0F - 0.063F; //+0.01Fはマーカーの位置によって適宜修正
            positionMarker[0][1] = markers_msg->markers[i].translation.y / 1000.0F;
            positionMarker[0][2] = markers_msg->markers[i].translation.z / 1000.0F;
        }
        if(markers_msg->markers[i].marker_name == "Base2"){
            positionMarker[1][0] = markers_msg->markers[i].translation.x / 1000.0F - 0.063F; //+0.01Fはマーカーの位置によって適宜修正
            positionMarker[1][1] = markers_msg->markers[i].translation.y / 1000.0F;
            positionMarker[1][2] = markers_msg->markers[i].translation.z / 1000.0F;
        }
        if(markers_msg->markers[i].marker_name == "Hand3"){
            positionMarker[2][0] = markers_msg->markers[i].translation.x / 1000.0F;
            positionMarker[2][1] = markers_msg->markers[i].translation.y / 1000.0F;
            positionMarker[2][2] = markers_msg->markers[i].translation.z / 1000.0F;
        }
        if(markers_msg->markers[i].marker_name == "Hand4"){
            positionMarker[3][0] = markers_msg->markers[i].translation.x / 1000.0F;
            positionMarker[3][1] = markers_msg->markers[i].translation.y / 1000.0F;
            positionMarker[3][2] = markers_msg->markers[i].translation.z / 1000.0F;
        }
        if(markers_msg->markers[i].marker_name == "Hand1"){
            positionMarker[4][0] = markers_msg->markers[i].translation.x / 1000.0F;
            positionMarker[4][1] = markers_msg->markers[i].translation.y / 1000.0F;
            positionMarker[4][2] = markers_msg->markers[i].translation.z / 1000.0F;
        }
        if(markers_msg->markers[i].marker_name == "Hand2"){
            positionMarker[5][0] = markers_msg->markers[i].translation.x / 1000.0F;
            positionMarker[5][1] = markers_msg->markers[i].translation.y / 1000.0F;
            positionMarker[5][2] = markers_msg->markers[i].translation.z / 1000.0F;
        } 
        // if(markers_msg.markers[i].marker_name == "Target1"){
        //     positionMarker[6][0] = markers_msg.markers[i].translation.x / 1000.0F;
        //     positionMarker[6][1] = markers_msg.markers[i].translation.y / 1000.0F;
        //     positionMarker[6][2] = markers_msg.markers[i].translation.z / 1000.0F;
        // }
        // if(markers_msg.markers[i].marker_name == "Target2"){
        //     positionMarker[7][0] = markers_msg.markers[i].translation.x / 1000.0F;
        //     positionMarker[7][1] = markers_msg.markers[i].translation.y / 1000.0F;
        //     positionMarker[7][2] = markers_msg.markers[i].translation.z / 1000.0F;
        // }
    }

    LowPassFilterMarkers();

    //原点の絶対座標を出す。原点はロボットの台座中心。
    for(int i = 0; i < 3; i++){
        positionOrigin[i] = (positionMarker[0][i] + positionMarker[1][i]) * 0.5F;
    }
    //printf("base0x=%lf,base1x=%lf\n",positionMarker[0][1],positionMarker[1][1]);
    
    //アームのマーカー座標の原点からの相対座標を出す。
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
} //msgCallback()

*/

//subscribe voltage
void ArmControlNode::msgCallback2(const inflatable::msg::VoltageInput::SharedPtr sub_msg) {
    for(int i = 0; i < AD_CHANNEL_NUMBER; i++) {
        voltageInput[i] = sub_msg->voltage_input[i];
    }
    printf("adconversionbefore "); //デバッグ用
    //Conversion from voltage to preassure
    ADconversion();
    printf("adconversionafter "); //デバッグ用
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

    static double orientationCurrent_buf;
    static double orientationTarget_buf;
    static double orientationIntegral;
    double torque;
    double interval;

    if (FB_cycle > 2) {
        //目標値との差が閾値以下ならI項を入れる
        if(abs(orientationTarget - orientationCurrent) < AngleFB_I_threshold){
            orientationIntegral += ((orientationTarget_buf - orientationCurrent_buf)
                                    + (orientationTarget - orientationCurrent))
                                    / (2.0F * SAMPLING_FREQUENCY);
        }
    }
    //elementはファイル出力のためのもの。
    if (FB_cycle > 1) {
            //P component
            torque = visual_P * (orientationTarget - orientationCurrent);
            element[0] = visual_P * (orientationTarget - orientationCurrent);
            //I component
            torque += visual_I * orientationIntegral;
            element[1] = visual_I * orientationIntegral;
            //D component
            torque += visual_D * ((orientationTarget_buf - orientationCurrent_buf) - (orientationTarget - orientationCurrent)) * SAMPLING_FREQUENCY;
            element[2] = visual_D * ((orientationTarget_buf - orientationCurrent_buf) - (orientationTarget - orientationCurrent)) * SAMPLING_FREQUENCY;

    }
    //次のI制御用
    orientationTarget_buf = orientationTarget;
    orientationCurrent_buf = orientationCurrent;

    FB_cycle++;

    return;
} //VisualFeedbackControl() 


//目標位置、目標関節角度を設定
void SetPositionTarget(float x3, float y3, float z3) {
    //手先目標位置を設定ArmControlNode::S
    pos.target.x[3] = x3;
    pos.target.y[3] = y3;
    pos.target.z[3] = z3;
    // //逆運動学で目標関節角度を計算
    // float l1 = param.link[0];
    // float l2 = param.link[1];
    // float l3 = param.link[2];
    // float buf_target_q[DEGREE_OF_FREEDOM];

    // float x0 = 0.0F;
    // float y0 = 0.0F;
    // float z0 = 0.0F;
    // float x1 = x0 + l1;
    // float y1 = y0;
    // float z1 = z0;

    // buf_target_q[0] = acosf((powf(x3, 2.0F) + powf(y3, 2.0F) - powf(l2, 2.0F) - powf(l3, 2.0F)) / (2.0F * l2 * sqrtf(powf(x3, 2.0F) + powf(y3, 2.0F)))) 
    //                     + atan2f(y3, x3);
    // buf_target_q[1] = atan2f(y3 - l2 * sinf(buf_target_q[0]), x3 - l2 * cosf(buf_target_q[0])) - buf_target_q[0];

    // //計算した関節角度がNANでないか判定
    // for(int i = 0 ; i < DEGREE_OF_FREEDOM; i++) {
    //     if(!(std::isnan(buf_target_q[i]))) {
    //         orientation.target.q[i] = buf_target_q[i];
    //     }
    // }
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


//各関節のインパクト駆動方向を決定する（旧版、ヤコビアンを使用）
void DetermineImpactDirection() {
    //目標位置誤差
    pos.deviation.x[3] = pos.target.x[3] - pos.current.x[3];
    pos.deviation.y[3] = pos.target.y[3] - pos.current.y[3];
    pos.deviation.z[3] = pos.target.z[3] - pos.current.z[3];
    float ex = pos.deviation.x[3];
    float ey = pos.deviation.y[3];
    float ez = pos.deviation.z[3];
    //偏差距離
    float error_distance = sqrtf(powf(ex, 2) + powf(ey, 2));

    printf("目標位置との偏差 (x=%lf, y=%lf)\n", ex, ey);
    printf("偏差距離 = %lf\n", error_distance);

    //ヤコビ行列
    param.J[0][0] = -param.link[1] * sin(orientation.current.q[0]) - param.link[2] * sin(orientation.current.q[0] + orientation.current.q[1]);
    param.J[0][1] = -param.link[2] * sin(orientation.current.q[0] + orientation.current.q[1]);
    param.J[1][0] = param.link[1] * cos(orientation.current.q[0]) + param.link[2] * cos(orientation.current.q[0] + orientation.current.q[1]);
    param.J[1][1] = param.link[2] * cos(orientation.current.q[0] + orientation.current.q[1]);

    //ヤコビ行列の転置
    param.J_T[0][0] = param.J[0][0];
    param.J_T[0][1] = param.J[1][0];
    param.J_T[1][0] = param.J[0][1];
    param.J_T[1][1] = param.J[1][1];

    float delta_q[DEGREE_OF_FREEDOM];   //[m^2/rad] 目標位置誤差を関節角度の変化量に変換したもの

    delta_q[0] = param.J_T[0][0] * ex + param.J_T[0][1] * ey;
    delta_q[1] = param.J_T[1][0] * ex + param.J_T[1][1] * ey;

    // const float delta_q_step = 0.1f * M_PI / 180.0f;    //1回のインパクト駆動で得られるはずの関節角度変化量 [rad] 例：0.1[deg]
    // const float min_error_reduction = powf(0.001f, 2); // [m^2] 例：1mm

    //インパクト駆動方向判定
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        printf("delta_q%d = %lf \n", i + 1, delta_q[i]);
        if (delta_q[i] > impactState[i].position_threshold) {
            impactState[i].impactDirection = 1;
        }
        else if (delta_q[i] < -impactState[i].position_threshold) {
            impactState[i].impactDirection = -1;
        }
        else {
            impactState[i].impactDirection = 0;
        }
    }
}//DetermineImpactDirection()


//各関節のインパクト駆動方向を決定する（順運動学、総当り）
void DetermineImpactDirection2() {
    //目標位置偏差
    pos.deviation.x[3] = pos.target.x[3] - pos.current.x[3];
    pos.deviation.y[3] = pos.target.y[3] - pos.current.y[3];
    pos.deviation.z[3] = pos.target.z[3] - pos.current.z[3];
    float ex = pos.deviation.x[3];
    float ey = pos.deviation.y[3];
    float ez = pos.deviation.z[3];
    
    //偏差距離
    float error_distance = sqrtf(powf(ex, 2) + powf(ey, 2));

    printf("目標位置との偏差[m] (x=%lf, y=%lf)\n", ex, ey);
    printf("偏差距離[m] = %lf\n", error_distance);

    const float delta_q_step = 0.1f * M_PI / 180.0f;    //1回のインパクト駆動で得られるはずの関節角度変化量 [rad] 例：0.1[deg]
    
    float original_q[2];    //現在の関節角度を取得
    original_q[0] = orientation.current.q[0];
    original_q[1] = orientation.current.q[1];

    float buf_q[2];

    //順運動学計算での手先位置を設定
    ForwardKinematics(original_q[0], original_q[1]);
    float FK_original_x = pos.buf.x[3];
    float FK_original_y = pos.buf.y[3];

    printf("順運動学で計算した手先位置 (x=%lf, y=%lf, z=%lf)\n", pos.buf.x[3], pos.buf.y[3], pos.buf.z[3]);
    printf("　　　　　　実際の手先位置 (x=%lf, y=%lf, z=%lf)\n", pos.current.x[3], pos.current.y[3], pos.current.z[3]);  
    printf("計算結果と実際の手先位置の距離 [m] = %lf\n", sqrtf(powf(pos.buf.x[3] - pos.current.x[3], 2) + powf(pos.buf.y[3] - pos.current.y[3], 2)));

    float ex_tmp = pos.buf.x[3] - FK_original_x; //順運動学でインパクト駆動前後の偏差ベクトルを計算
    float ey_tmp = pos.buf.y[3] - FK_original_y;
    float dot = ex*ex_tmp + ey*ey_tmp;  //内積を計算
    float best_dot = dot;  //最も誤差が減るときの内積の値を保存する変数
    int best_dir[2] = {0, 0};   //最も誤差が減るときの関節1と関節2のインパクト駆動方向を保存する変数
    int direction_set[3] = {-1, 0, 1};  //関節のインパクト駆動方向の候補セット

    //偏差が閾値以内ならインパクト駆動しない
    if(error_distance < impactState[0].position_threshold) {
        impactState[0].impactDirection = 0;
        impactState[1].impactDirection = 0;
        return;
    }

    //関節1と関節2のインパクト駆動方向の組み合わせを総当りで試す
    for (int d1 = 0; d1 < 3; d1++) {
        for (int d2 = 0; d2 < 3; d2++) {
            int dir1 = direction_set[d1];
            int dir2 = direction_set[d2];

            // 仮角度
            buf_q[0] = original_q[0] + dir1 * delta_q_step;
            buf_q[1] = original_q[1] + dir2 * delta_q_step;
            
            // 順運動学で手先位置を計算 pos.bufに格納される
            ForwardKinematics(buf_q[0], buf_q[1]);

            // 順運動学で計算した位置と実際の位置の偏差
            ex_tmp = pos.buf.x[3] - FK_original_x;
            ey_tmp = pos.buf.y[3] - FK_original_y;

            // 内積を計算。内積が大きいほど、インパクト駆動による変化ベクトルが偏差ベクトルに近い
            dot = ex*ex_tmp + ey*ey_tmp;

            //誤差が減るなら方向を保存
            if (best_dot < dot) {
                best_dot = dot;
                best_dir[0] = dir1;
                best_dir[1] = dir2;
            }
        }
    }
    impactState[0].impactDirection = best_dir[0];
    impactState[1].impactDirection = best_dir[1];
}//DetermineImpactDirection2()


//インパクト駆動制御
void ImpactDrivenControl(ImpactState& state, int pressure_idx_pos, int pressure_idx_neg) {
    //矩形入力部分
    if (state.ImpactInput_flag && cycle >= state.inputstart_cycle) {
        //目標角度に近い場合は入力を行わずそのままループを継続
        if (state.impactDirection == 0) {
            return;
        }
        //目標圧力をセット 変化前の目標値はbufに保存
        pressure.target_buf[pressure_idx_pos] = pressure.target[pressure_idx_pos];
        pressure.target_buf[pressure_idx_neg] = pressure.target[pressure_idx_neg];
        if (state.impactDirection == 1) {
            pressure.target[pressure_idx_pos] += state.p_h;
            pressure.target[pressure_idx_neg] -= state.p_h;
            state.PositiveInput_flag = true;
            state.NegativeInput_flag = false;
        } 
        else if (state.impactDirection == -1) {
            pressure.target[pressure_idx_pos] -= state.p_h;
            pressure.target[pressure_idx_neg] += state.p_h;
            state.NegativeInput_flag = true;
            state.PositiveInput_flag = false;
        }
        //矩形入力の終わりをinputdown_cycleとする
        state.inputdown_cycle = cycle + floor(state.t_w * SAMPLING_FREQUENCY);
        state.ImpactInput_flag = false;
    }
    //矩形入力後のステップ入力への移行部分
    else if (!state.ImpactInput_flag && cycle >= state.inputdown_cycle) {
        if (state.impactDirection == 1) {
            pressure.target[pressure_idx_pos] = pressure.target_buf[pressure_idx_pos] + state.p_s;
            pressure.target[pressure_idx_neg] = pressure.target_buf[pressure_idx_neg] - state.p_s;
            state.PositiveImpact_count++;
        } 
        else if (state.impactDirection == -1) {
            pressure.target[pressure_idx_pos] = pressure.target_buf[pressure_idx_pos] - state.p_s;
            pressure.target[pressure_idx_neg] = pressure.target_buf[pressure_idx_neg] + state.p_s;
            state.NegativeImpact_count++;
        }
        state.inputstart_cycle = cycle + floor(state.t_i * SAMPLING_FREQUENCY);
        state.ImpactInput_flag = true;
    }
}//ImpactDrivenControl()


//圧力フィードバック
void PressureFeedbackControl() {
    static unsigned int FB_cycle = 0;
    static double pressureTarget_buf[AD_CHANNEL_NUMBER];
    static double pressureCurrent_buf[AD_CHANNEL_NUMBER];
    static double pressureIntegral[AD_CHANNEL_NUMBER];
    static double pressureDeviation_buf[DA_CHANNEL_NUMBER]; //前フレームでの偏差
    static bool is_first = true;
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
            pressureDeviation[i] = pressure.target[i] - pressureCurrentFiltered[i];
            pressureP[i] = pressureKP[i] * (pressure.target[i] - pressureCurrentFiltered[i]);

            //I component
            pressureI[i] = pressureKI[i] * pressureIntegral[i];

            //D component
            pressureD[i] = pressureKD[i] * (pressureDeviation[i] - pressureDeviation_buf[i]) * SAMPLING_FREQUENCY;
            
            // pressure.output[i] += pressureKD * (pressureCurrent[i] - pressure.current_buf[i]);
            // pressureD[i] = pressureKD * (pressureCurrent[i] - pressure.current_buf[i]);
            
            //各項を足し合わせる
            //P + I + D
            pressure.output[i] += pressureP[i] + pressureI[i] + pressureD[i]; 

            //Update previous value
            pressureTarget_buf[i] = pressure.target[i];
            pressureCurrent_buf[i] = pressureCurrentFiltered[i];
            pressureDeviation_buf[i] = pressureDeviation[i];
        }
    }
    FB_cycle++;
    return;
} //PressureFeedbackControl()

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

//圧力目標値のチェック
/*
int CheckOverPressure_Target(void) {
	for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
		if (pressure.target[i] > MAXIMUM_PRESSURE) {
			pressure.target[i] = MAXIMUM_PRESSURE;
			ROS_WARN("Over maximum pressure %d\n", i);
		}
        if (pressure.target[i] < MINIMUM_PRESSURE) {
			pressure.target[i] = MINIMUM_PRESSURE;
			ROS_WARN("Under minimum pressure %d\n", i);
		}
	}
	return 0;
} //CheckOverPressure_Target()
*/
// 圧力目標値のチェック
int CheckOverPressure_Target(void) {
    // ロガーを取得（"inflatable_node" はお好みの名前に変更してください）
    auto logger = rclcpp::get_logger("inflatable_node");

    for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        if (pressure.target[i] > MAXIMUM_PRESSURE) {
            pressure.target[i] = MAXIMUM_PRESSURE;
            // ROS 2の書き方: 第1引数にロガーを追加、末尾の \n は不要
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

/*
char getch() {
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
        ROS_ERROR("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0) {
        ROS_ERROR("tcsetattr ICANON");
    }
    if(rv == -1) {
        ROS_ERROR("select");
    }
    else if(rv == 0) {
        //ROS_INFO("no_key_pressed");
    }
    else {
        read(filedesc, &buff, len );
    }

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0) {
        ROS_ERROR ("tcsetattr ~ICANON");
    }
    return (buff);
} //getch()
*/

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
    
    // ROS 1: nh_param.param<double>("base_pressure", basePressure, 30);
    basePressure = this->declare_parameter<double>("base_pressure", 30.0);
    // ROS 1: nh_param.param<double>("link_pressure", linkPressure, 4.0);
    linkPressure = this->declare_parameter<double>("link_pressure", 4.0);
    // ROS 1: nh_param.param<double>("target_JointStiffness", targetJointStiffness, 1.20);
    targetJointStiffness = this->declare_parameter<double>("target_JointStiffness", 1.20);
    
    //ros_pub_ = this->create_publisher<inflatable::msg::VoltageOutput>("voltage_output", 10);
    //ros_sub_ = this->create_subscription<mocap4r2_msgs::msg::Markers>("/mocap4r2_msgs/msg/markers", 10, std::bind(&ArmControlNode::msgCallback, this, std::placeholders::_1));
    //ros_sub2_ = this->create_subscription<inflatable::msg::VoltageInput>("voltage_input", 10, std::bind(&ArmControlNode::msgCallback2, this, std::placeholders::_1));
    //ros_sub3_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/vicon/LDPE_hand/Hand", 10, std::bind(&ArmControlNode::msgCallback_hand, this, std::placeholders::_1));
    //ros_sub4_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/vicon/LDPE_base/Base", 10, std::bind(&ArmControlNode::msgCallback_base, this, std::placeholders::_1));

    // すべて先頭に 「/inflatable/」 を付加し、絶対パス（スラッシュ始まり）にする
    ros_pub_ = this->create_publisher<inflatable::msg::VoltageOutput>("/inflatable/voltage_output", 10);
    ros_sub2_ = this->create_subscription<inflatable::msg::VoltageInput>("/inflatable/voltage_input", 10, std::bind(&ArmControlNode::msgCallback2, this, std::placeholders::_1));
    ros_sub3_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/inflatable/vicon/LDPE_hand/Hand", 10, std::bind(&ArmControlNode::msgCallback_hand, this, std::placeholders::_1));
    ros_sub4_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/inflatable/vicon/LDPE_base_v2/Base", 10, std::bind(&ArmControlNode::msgCallback_base, this, std::placeholders::_1));



    auto period = std::chrono::milliseconds(1000 / SAMPLING_FREQUENCY); 
    // タイマーを作成し、実行する関数を紐付ける
    timer_ = this->create_wall_timer(
        period, 
        std::bind(&ArmControlNode::control_loop, this)
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
    sprintf(filename, "/home/takahara/ros2_ws/data/2dof_impact/%02d%02d_%02d%02d_%02d.csv", 
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

void ArmControlNode::control_loop(){
    viconUpdateLoop();
    printf("cycle = %d\n", cycle);
    //圧力を表示
    for (int i = 0; i < 4; i++) {
        printf(" pressure.target[%i] = %lf\n", i, pressure.target[i]);
        // printf(" pressure.output[%i] = %lf\n", i, pressure.output[i]);
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
        SquareInput_flag = false;
        CombinationInput_flag = false;
        ImpactDrivenControl_flag = false;
        InputReady_flag = false;
        PressureFeedback_flag = false;
        AngleSinWave_flag = false;
        VisualFeedback_flag = false;
        is_first = true;
        for (int i=0; i<DEGREE_OF_FREEDOM; i++) {
            impactState[i].ImpactInput_flag = false;
        }
        for (int i=0; i<DA_CHANNEL_NUMBER; i++) {
            pressure.target[i] = 0;
        }
        for (int i=0; i<DA_CHANNEL_NUMBER; i++) {
            pressure.output[i] = 0;
        }
            //printf("pushued r\n");
    }
    //リンクを膨らませる
    if (key == 'l') {
        
        pressure.output[LINK_CHANNEL_NUMBER1] = pressure.base[LINK_CHANNEL_NUMBER1];
        pressure.output[LINK_CHANNEL_NUMBER2] = pressure.base[LINK_CHANNEL_NUMBER2];
    
       //printf("pushued l\n");
    }

    //初期状態(基準圧力, 圧力FBなし, タイマースタートなし)
    if (key == 'j') {
        setup_flag = false;
        PressureFeedback_flag = false;
        pressure.output[0] = pressure.base[0];
        pressure.output[1] = pressure.base[1];
        //printf("pushued j\n");
    }

    //初期状態(基準圧力, 圧力FBあり, タイマースタートあり)
    if (key == 's') {
        setup_flag = true;
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

    //インパクト駆動制御
    if (key == 'i') {
        
        setup_flag = true;
        PressureFeedback_flag = true;
        ImpactDrivenControl_flag = true;
        impactState[0].ImpactInput_flag = true;
        impactState[1].ImpactInput_flag = true;
        
        pressure.target[0] = pressure.base[0];
        pressure.target[1] = pressure.base[1];
        pressure.target[2] = pressure.base[2];
        pressure.target[3] = pressure.base[3];
        pressure.output[4] = 0;
        pressure.output[5] = 0;
        pressure.output[6] = 0;
        pressure.output[7] = 0;

        //目標位置をセット
        // orientation.target.q[0] = -30.0 / 180 * M_PI;
        // orientation.target.q[1] = -30.0 / 180 * M_PI;
        // (x=0.476813, y=0.111512, z=-0.072841)
        // (x=0.497752, y=0.116852)

        // SetPositionTarget(pos.current.x[3] + 0.005f, pos.current.y[3] - 0.01f, pos.current.z[3]);
        // SetPositionTarget(0.497752, 0.116852, -0.072841);
        
        // SetPositionTarget(0.493, 0.120, -0.05);
        
        //SetPositionTarget(0.492795, 0.119429, -0.0490081);  //0216
        SetPositionTarget(-0.086335, 0.523879, 0.023271);     //0522

        printf("pushued i\n");
    }
    
    //圧力値正弦波入力（応答確認用）
    if (key == 'y') {
        setup_flag = true;
        PressureFeedback_flag = true;
        PressureSinWave_flag = true;
        pressure.target[0] = pressure.base[0];
        pressure.target[1] = pressure.base[1];
        // pressure.target[2] = pressure.base[2];
        // pressure.target[3] = pressure.base[3];
    }

    //角度FB正弦波入力（応答確認用）
    if (key == 'h') {
        setup_flag = true;
        PressureFeedback_flag = true;
        AngleSinWave_flag = true;
        VisualFeedback_flag = true;
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

        //now_time = (float)timer.now_d.sec + (float)timer.now_d.nsec/1e9; //整数部+小数点以下[s]
        now_time = timer.now_d.seconds();
        //now_time = (double)timer.now_d.nanoseconds() / 1e9;


        //時間などを表示
        // printf("start_cycle = %d\n", start_cycle);
        printf("now_time = %lf\n", now_time);
    }

    //圧力正弦波入力（応答確認用）
    if (PressureSinWave_flag) {
        pressure.target[0] = pressure.base[0] + sinwave_amplitude * sin(2 * M_PI * sinwave_frequency * now_time);
        pressure.target[1] = pressure.base[1] - sinwave_amplitude * sin(2 * M_PI * sinwave_frequency * now_time);
        // pressure.target[2] = pressure.base[2] + sinwave_amplitude * sin(2 * M_PI * sinwave_frequency * now_time);
        // pressure.target[3] = pressure.base[3] - sinwave_amplitude * sin(2 * M_PI * sinwave_frequency * now_time);
    }

    //角度正弦波入力（応答確認用）
    if (AngleSinWave_flag) {
        //目標角度をsin波で変化させる
        orientationTarget = sinwave_amplitude * sin(2 * M_PI * sinwave_frequency * now_time) / 180 * M_PI ;
    }

    //i押下後 インパクト駆動制御
    if (ImpactDrivenControl_flag) {
        orientation.current.q[0] = LowPassFilterAngle1(orientation.current.q[0], 15);
        orientation.current.q[1] = LowPassFilterAngle2(orientation.current.q[1], 15);
        // DetermineImpactDirection();
        DetermineImpactDirection2();
        for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
            ImpactDrivenControl(impactState[i], i * 2 + 1, i * 2);
            // インパクト駆動カウントを表示
            printf("q%d PositiveImpact_count = %d\n", i + 1, impactState[i].PositiveImpact_count);
            printf("q%d NegativeImpact_count = %d\n", i + 1, impactState[i].NegativeImpact_count);
            // printf("q%d orientationTarget = %lf\n", i + 1, orientation.target.q[i] / M_PI * 180);
            // printf("q%d orientationCurrent = %lf\n", i + 1, orientation.current.q[i] / M_PI * 180);
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
    pub_msg_.voltage_output.shrink_to_fit();    //いらんかも
    for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        pub_msg_.voltage_output.emplace_back(voltageOutput[i]);
    }
    ros_pub_->publish(pub_msg_);
    //loop_rate.sleep();

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

/*
void control_loop(){
    printf("cycle = %d\n", cycle);
    //圧力を表示
    for (int i = 0; i < 4; i++) {
        printf(" pressure.target[%i] = %lf\n", i, pressure.target[i]);
        // printf(" pressure.output[%i] = %lf\n", i, pressure.output[i]);
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
        SquareInput_flag = false;
        CombinationInput_flag = false;
        ImpactDrivenControl_flag = false;
        InputReady_flag = false;
        PressureFeedback_flag = false;
        AngleSinWave_flag = false;
        VisualFeedback_flag = false;
        is_first = true;
        for (int i=0; i<DEGREE_OF_FREEDOM; i++) {
            impactState[i].ImpactInput_flag = false;
        }
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

    //初期状態(基準圧力, 圧力FBなし, タイマースタートなし)
    if (key == 'j') {
        setup_flag = false;
        PressureFeedback_flag = false;
        pressure.output[0] = pressure.base[0];
        pressure.output[1] = pressure.base[1];
        // pressure.output[2] = pressure.base[2];
        // pressure.output[3] = pressure.base[3];
    }

    //初期状態(基準圧力, 圧力FBあり, タイマースタートあり)
    if (key == 's') {
        setup_flag = true;
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

    //インパクト駆動制御
    if (key == 'i') {
        setup_flag = true;
        PressureFeedback_flag = true;
        ImpactDrivenControl_flag = true;
        impactState[0].ImpactInput_flag = true;
        impactState[1].ImpactInput_flag = true;

        pressure.target[0] = pressure.base[0];
        pressure.target[1] = pressure.base[1];
        pressure.target[2] = pressure.base[2];
        pressure.target[3] = pressure.base[3];
        pressure.output[4] = 0;
        pressure.output[5] = 0;
        pressure.output[6] = 0;
        pressure.output[7] = 0;

        //目標位置をセット
        // orientation.target.q[0] = -30.0 / 180 * M_PI;
        // orientation.target.q[1] = -30.0 / 180 * M_PI;
        // (x=0.476813, y=0.111512, z=-0.072841)
        // (x=0.497752, y=0.116852)

        // SetPositionTarget(pos.current.x[3] + 0.005f, pos.current.y[3] - 0.01f, pos.current.z[3]);
        // SetPositionTarget(0.497752, 0.116852, -0.072841);
        SetPositionTarget(0.492795, 0.119429, -0.0490081);  //0216
        // SetPositionTarget(0.493, 0.120, -0.05);
    }
    
    //圧力値正弦波入力（応答確認用）
    if (key == 'y') {
        setup_flag = true;
        PressureFeedback_flag = true;
        PressureSinWave_flag = true;
        pressure.target[0] = pressure.base[0];
        pressure.target[1] = pressure.base[1];
        // pressure.target[2] = pressure.base[2];
        // pressure.target[3] = pressure.base[3];
    }

    //角度FB正弦波入力（応答確認用）
    if (key == 'h') {
        setup_flag = true;
        PressureFeedback_flag = true;
        AngleSinWave_flag = true;
        VisualFeedback_flag = true;
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

        now_time = (float)timer.now_d.sec + (float)timer.now_d.nsec/1e9; //整数部+小数点以下[s]

        //時間などを表示
        // printf("start_cycle = %d\n", start_cycle);
        printf("now_time = %lf\n", now_time);
    }

    //圧力正弦波入力（応答確認用）
    if (PressureSinWave_flag) {
        pressure.target[0] = pressure.base[0] + sinwave_amplitude * sin(2 * M_PI * sinwave_frequency * now_time);
        pressure.target[1] = pressure.base[1] - sinwave_amplitude * sin(2 * M_PI * sinwave_frequency * now_time);
        // pressure.target[2] = pressure.base[2] + sinwave_amplitude * sin(2 * M_PI * sinwave_frequency * now_time);
        // pressure.target[3] = pressure.base[3] - sinwave_amplitude * sin(2 * M_PI * sinwave_frequency * now_time);
    }

    //角度正弦波入力（応答確認用）
    if (AngleSinWave_flag) {
        //目標角度をsin波で変化させる
        orientationTarget = sinwave_amplitude * sin(2 * M_PI * sinwave_frequency * now_time) / 180 * M_PI ;
    }

    //i押下後 インパクト駆動制御
    if (ImpactDrivenControl_flag) {
        orientation.current.q[0] = LowPassFilterAngle1(orientation.current.q[0], 15);
        orientation.current.q[1] = LowPassFilterAngle2(orientation.current.q[1], 15);
        // DetermineImpactDirection();
        DetermineImpactDirection2();
        for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
            ImpactDrivenControl(impactState[i], i * 2 + 1, i * 2);
            // インパクト駆動カウントを表示
            printf("q%d PositiveImpact_count = %d\n", i + 1, impactState[i].PositiveImpact_count);
            printf("q%d NegativeImpact_count = %d\n", i + 1, impactState[i].NegativeImpact_count);
            // printf("q%d orientationTarget = %lf\n", i + 1, orientation.target.q[i] / M_PI * 180);
            // printf("q%d orientationCurrent = %lf\n", i + 1, orientation.current.q[i] / M_PI * 180);
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
    pub_msg.voltage_output.clear();
    pub_msg.voltage_output.shrink_to_fit();
    for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        pub_msg.voltage_output.emplace_back(voltageOutput[i]);
    }
    ros_pub.publish(pub_msg);
    loop_rate.sleep();

    //File output
#ifdef FILE_OUTPUT
    if(setup_flag){
    ofs_ << cycle << ","<< (float)timer.now_d.sec + (float)timer.now_d.nsec / 1e9 << ',';
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
        ofs << impactState[i].PositiveImpact_count << ',' << impactState[i].NegativeImpact_count << ',';
        ofs << impactState[i].inputstart_cycle << ',' << impactState[i].inputdown_cycle << ',';
    }
    ofs_ << std::endl;
    }
#endif
    cycle++;
}*/


