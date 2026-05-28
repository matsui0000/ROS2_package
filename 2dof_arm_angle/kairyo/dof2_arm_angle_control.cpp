#include "dof2_arm_angle.hpp"  // 1. 【最優先】先ほど整理したこのノードのヘッダファイル
#include "class.hpp"           // 2. Time, RobotPosition などのメンバ変数アクセスに必須
#include "utils.hpp"           // 3. AD_CHANNEL_NUMBER, LINK_CHANNEL_NUMBER1 などの定数・マクロに必須
#include "2dof_header.hpp"     // 4. DEGREE_OF_FREEDOM などの定数に必須

// 4. C++ 標準ライブラリ
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <ctime>

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
    sprintf(filename, "/home/takahara/ros2_ws/src/inflatable/data/dof2_arm_angle/%02d%02d_%02d%02d_%02d.csv", 
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
        //PressureFeedback_flag = true;
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
        //orientation.target.q[0] = -30.0 / 180 * M_PI;
        orientation.target.q[0] = 0 / 180 * M_PI;
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
        orientation.current.q[0] = LowPassFilter_angle0.Filter(orientation.current.q[0], 5.0);
        orientation.current.q[1] = LowPassFilter_angle1.Filter(orientation.current.q[1], 5.0);
        for (int i = 0; i < DEGREE_OF_FREEDOM; i++){
            double required_torque = VisualFeedbackControl(orientation.target.q[i], orientation.current.q[i], i);
            double pdf = convertPdf(orientation.target.q[i], required_torque);
            if (i == 0) {
                // 手先側の関節を負の方向（角度が小さくなる方向）に動かすには ch0 を膨らませ、ch1 を縮める
                // 正の方向（角度が大きくなる方向）に動かすには ch1 を膨らませ、ch0 を縮める
                pressure.target[0] = basePressure - pdf / 2.0F; // ch0: 膨らむと小さくなる
                pressure.target[1] = basePressure + pdf / 2.0F; // ch1: 膨らむと大きくなる
            } 
            else if (i == 1) {
                // 台座側の関節も同様に、負の方向（角度が小さくなる方向）に動かすには ch2 を膨らませ、ch3 を縮める
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
