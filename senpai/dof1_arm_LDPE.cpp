/****************************************************
Name     : dof1_arm_LDPE.cpp
Abstract : ・LDPE1自由度アームの位置制御。
           ・dof1_arm_control.cppを基に作成。
           ・キーのそれぞれの対応
            ・r : 全圧力値を0にする。
            ・l : リンクを膨らませる。
            ・s : 初期状態。 
            ・b : 開始位置まで移動。
            ・a : 目標位置まで移動。
           ・動かすときは、r,l,s,b,a,rの順。
           ・角度は鉛直方向を0度として、右側に傾く向きを正とする。
           ・2,3のアクチュエータ、8のリンクが膨らむ。
           ・bを押して、change_timeだけ経過したら目標位置まで移動する。
           ・csvに記録するときはl,s,bの順で押し、一定時間経過後ににrを押す。
Author   : B4 Kota Takenaka  2020/10/06
update   : 2020/10/08 重力トルクなどを修正。
           2020/10/09
           2020/10/12
           2020/10/13
           2020/10/14,15,17,18,20,29
           2020/10/30 ファイル出力関係を修正。
           2020/10/31,11/07,11/08
           2020/11/10 
           2020/11/17,18
*****************************************************/

#include <stdio.h>
#include "ros/ros.h"

#include <fstream>
#include <cmath>
#include <iomanip>
#include <termios.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
//Publishing
#include "inflatable/VoltageOutput.h"
//Subscribing
//#include "inflatable/InflatablePose.h"
#include "inflatable/VoltageInput.h"
//viconGrabPose.srvファイルをビルドして自動生成されたヘッダファイル
#include "vicon_bridge/viconGrabPose.h"

#define DA_CHANNEL_NUMBER 12
#define AD_CHANNEL_NUMBER 8     //=number of pressure sensor
#define LINK_CHANNEL_NUMBER 8
#define THE_NUMBER_OF_MARKERS 11
#define THE_NUMBER_OF_MARKERS_USING 4

#define DA_Conversion_K 0.1     //Voltage = K * Pressure
#define AD_Conversion_K 25.2    //Pressure = K * Voltage + A
#define AD_Conversion_A -25.4
#define SAMPLING_FREQUENCY 100
#define GRAVITY_ACC 9.80665

#define MINIMUM_PRESSURE 0.0

#define START_ORIENTATION 0.0
#define TARGET_ORIENTATION 20.0

#define FILE_OUTPUT //ファイル出力するならコメントアウト外す

class Time {
public:
    ros::Time now_t;         //Current time
    ros::Duration now_d;     //Current duration time
    ros::Time base;          //Time for current time
    ros::Time buf;           //Time for sampling time
    ros::Duration sampling;  //Sampling time
    Time();
    ~Time();
};

Time::Time() {
}

Time::~Time() {
}

//gain param
double pressure_P;//1.0;
double pressure_I;//1.0;
double pressure_D;//6.2;
double visual_P;
double visual_I;
double visual_D;
double basePressure;
double diffPressure;
double linkPressure;
double maxPressure;
double targetJointStiffness;

double link_angle_deg = 35;  //[deg]
double link_angle = link_angle_deg*M_PI/180;  //[rad]

double link_length = 0.20; //リンクの長さ
double body_hight  = 0.195; //白の色の本体の長さ+本体内のリンクの出っ張り 

int link_num = 8; //膨らませるリンクの番号

float change_time = 0.5; //初期位置から目標位置まで移動する時間[s]

//ROS time
unsigned int cycle = 0;
Time timer;

//sensor
double voltageInput[AD_CHANNEL_NUMBER];
double voltageOutput[DA_CHANNEL_NUMBER];

double pressureCurrent[AD_CHANNEL_NUMBER];
double pressureTarget[DA_CHANNEL_NUMBER];
double pressureOutput[DA_CHANNEL_NUMBER];

double positionMarker[THE_NUMBER_OF_MARKERS_USING][3]; //position of base and arm marker
double positionBase[3];   //positionBase[number][x:0, y:1, z:2]
double positionArm[3];   //positionArm[number][x:0, y:1, z:2]
double orientationTarget;
double orientationCurrent;
double element[3];
double element_pre[3];

bool flag_overpressure[DA_CHANNEL_NUMBER];

//prototype
char getch();
int CheckOverPressure_Target(void);
int CheckOverPressure_Output(void);
int DAconversion();
int ADconversion();
void LowPassFilter();
double LowPassFilter(double input);
double LowPassFilterD(double input);
double LowPassFilterD2(double input);
double CompensateGravity();
double FeedbackControl();

//subscribe voltage
void msgCallback2(const inflatable::VoltageInput::ConstPtr& sub_msg) {
    for(int i = 0; i < AD_CHANNEL_NUMBER; i++) {
        voltageInput[i] = sub_msg->voltage_input[i];
    }

    //Conversion from voltage to preassure
    ADconversion();
} //msgCallback2()

//クォータニオンを回転行列に変換する。
void convert_R(geometry_msgs::TransformStamped pose,double R[3][3]){
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
void print_quartanion(geometry_msgs::TransformStamped pose){
    double x,y,z,w;
    x = pose.transform.rotation.x;
    y = pose.transform.rotation.y;
    z = pose.transform.rotation.z;
    w = pose.transform.rotation.w;
    printf("x=%lf y=%lf x=%lf w=%lf\n\n",x,y,z,w);
} //print_quartanion()

geometry_msgs::TransformStamped base_pose;
bool msgCallback_base_flag = false;

void msgCallback_hand(const geometry_msgs::TransformStamped hand_pose){
    double x,y,z,w;
    double theta; //求めたい手先角度

    double handR[3][3]; //世界座標からhand座標系への回転行列
    double baseR[3][3]; //世界座標からbase座標系への回転行列
    double baseRT[3][3];//baseRの転置行列
    double bhR[3][3];   //baseからhandへの回転行列

    //まだ1回もmsgCallbask_base()が実行されていないなら終了
    if(!(msgCallback_base_flag)) return; 

    /*printf("hand_pose\n");
    print_quartanion(hand_pose);
    printf("base_pose\n");
    print_quartanion(base_pose);

    convert_R(hand_pose,handR);
    printf("hand座標系への回転行列");
    print_matrix(handR);

    convert_R(base_pose,baseR);
    printf("base座標系への回転行列");
    print_matrix(baseR);

    Trans_R(baseR,baseRT);
    printf("base座標系への回転行列の転置行列");
    print_matrix(baseRT);

    mul_R(baseRT,handR,bhR);
    printf("baseからhandへの回転行列");
    print_matrix(bhR);*/

    convert_R(hand_pose,handR);
    convert_R(base_pose,baseR);
    double inner = 0;
    for(int i=0;i<3;i++){
        //inner += bhR[0][i] * baseR[0][i];
        inner += handR[0][i] * baseR[0][i];
    }
    printf("inner=%lf\n",inner);

    printf("acos(inner)=%lf\n",acos(inner)/M_PI *180);
    //手先角度は鉛直方向を0度として左右に±90度ずつ。
    theta = acos(inner) - M_PI/2;
    printf("手先角度：%lf度\n",theta/M_PI * 180);
    orientationCurrent = theta;
    //orientationCurrent = LowPassFilterD2(orientationCurrent);
} //msgCallback_hand()

void msgCallback_base(const geometry_msgs::TransformStamped pose){
    double x,y,z,w;
    base_pose = pose;
    msgCallback_base_flag = true;
} //msgCallback_base()


//返り値：視覚フィードバックによるトルク
double VisualFeedbackControl(){
    static unsigned int FB_cycle;
    //Visual Feedback
    static double orientationCurrent_buf;
    static double orientationTarget_buf;
    static double orientationIntegral;
    double torque;
    double interval;

    static double interval2;
    static double D_buf;

    if (FB_cycle > 2) {
            orientationIntegral += ((orientationTarget_buf - orientationCurrent_buf)
                                    + (orientationTarget - orientationCurrent))
                                    / (2.0F * SAMPLING_FREQUENCY);
    }
    //elementはファイル出力のためのもの。
    if (FB_cycle > 1) {
            //P component
            torque = visual_P * (orientationTarget - orientationCurrent);
            element[0] = visual_P * (orientationTarget - orientationCurrent);
            //I component
            torque += visual_I * orientationIntegral;
            element[1] = visual_I * orientationIntegral;
    }

    orientationTarget_buf = orientationTarget;
    orientationCurrent_buf = orientationCurrent;
    FB_cycle++;
    return torque;
} //VisualFeedbackControl()

void PressureFeedbackControl(){
    static unsigned int FB_cycle = 0;
    //Pressure feedback control
    //今回は2と3のみ
    static double pressureTarget_buf[AD_CHANNEL_NUMBER];
    static double pressureCurrent_buf[AD_CHANNEL_NUMBER];
    static double pressureIntegral[AD_CHANNEL_NUMBER];
    static bool is_first = true;


    //変数初期化
    if(is_first){
        for(int i=0; i < AD_CHANNEL_NUMBER ; i++){
            pressureTarget_buf[i] = 0.0;
            pressureCurrent_buf[i] = 0.0;
            pressureIntegral[i] = 0.0;
            }
        is_first = false;
    }

    for (int i = 2; i < 4; i++) {
        pressureOutput[i] = pressureTarget[i];
        if (FB_cycle > 2) {
            pressureIntegral[i] += ((pressureTarget_buf[i] - pressureCurrent_buf[i])
                                    + (pressureTarget[i] - pressureCurrent[i]))
                                    / (2.0F * SAMPLING_FREQUENCY);
        }

        if (FB_cycle > 1) {
            //P component
            pressureOutput[i] += pressure_P * (pressureTarget[i] - pressureCurrent[i]);
            
            //I component
            pressureOutput[i] += pressure_I * pressureIntegral[i];
            element_pre[i] = pressureIntegral[i];
            //D component
            pressureOutput[i] += -1.0F * pressure_D * (pressureCurrent[i] - pressureCurrent_buf[i]);

            //Update previous value
            pressureTarget_buf[i] = pressureTarget[i];
            pressureCurrent_buf[i] = pressureCurrent[i];
        }
        
    }
    FB_cycle++;
    return;
} //PressureFeedbackControl()


//差圧をPdfを返す関数。
double convertPdf(double q , double torque){
    //cは2020年9月に測定したもの。
    //angleは[rad]
    double c[5] = {-0.0070497, 0.06649 , -0.21356, 0.2529, -0.2088};
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


int main(int argc, char **argv) {
    ros::init(argc, argv, "dof1_arm_LDPE");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");
    //VoltageOutputメッセージファイルを使用
    //トピック名はvoltage_output
    ros::Publisher ros_pub = nh.advertise<inflatable::VoltageOutput>("voltage_output", 10);
    //ros::Subscriber ros_sub = nh.subscribe("inflatable_pose", 10, msgCallback);
    //トピック名は/vicon/markers ?
    //ros::Subscriber ros_sub = nh.subscribe("/vicon/markers", 10, msgCallback);
    //トピック名はvoltage_input
    ros::Subscriber ros_sub2 = nh.subscribe("voltage_input", 10, msgCallback2);
    ros::Subscriber ros_sub3 = nh.subscribe("/vicon/LDPE_hand/hand",10,msgCallback_hand);
    ros::Subscriber ros_sub4 = nh.subscribe("/vicon/LDPE_base/base",10,msgCallback_base);

    ros::Rate loop_rate(SAMPLING_FREQUENCY);
    //VoltageOutput.msgに記載した形式のmsgを宣言
    inflatable::VoltageOutput pub_msg;
    timer.base = ros::Time::now();
    timer.buf = ros::Time::now();

    //launchから取得するための設定。
    nh_param.param<double>("pressure_Kp", pressure_P, 0);
    nh_param.param<double>("pressure_Ki", pressure_I, 0);
    nh_param.param<double>("pressure_Kd", pressure_D, 0);

    nh_param.param<double>("visual_Kp", visual_P, 1.0);
    nh_param.param<double>("visual_Ki", visual_I, 0);
    nh_param.param<double>("visual_Kd", visual_D, 0);

    nh_param.param<double>("base_pressure", basePressure, 0);
    nh_param.param<double>("link_pressure", linkPressure, 0);

    nh_param.param<double>("max_pressure", maxPressure, 60);

    nh_param.param<double>("target_JointStiffness", targetJointStiffness, 0.05);

    for(int i=0;i<DA_CHANNEL_NUMBER;i++){
        pressureOutput[i] = 0;
    }

    static bool action_flag = false;
    static bool setup_flag = false;
    static bool PressureFeedback_flag = false;
    static double start_time;
    static double action_start_time;
    static double start_orientation;
    double torque;
    double Pdf;
    double now_time = 0.0;
    static bool is_first = true;

    //File output preparation
#ifdef FILE_OUTPUT

    char filename[100];
    time_t date_info = time(NULL);
    struct tm *pnow = localtime(&date_info);
    sprintf(filename, "/home/takenaka/catkin_ws/src/inflatable/Result/%04d%02d%02d%02d%02d.csv", 
        pnow->tm_year + 1900, 
        pnow->tm_mon + 1, 
        pnow->tm_mday, 
        pnow->tm_hour, 
        pnow->tm_min);
    std::ofstream ofs(filename);
    ofs.clear();
    ofs.seekp(0);

    ofs << "cycle" << ',' << "Time[s]" << ','
        << "torque[Nm]" << ',' 
        << "orientationCurrent[deg]" << ','
        << "orientationTarget[deg]" << ',' ;
    ofs << "Link Output[kPa]"     << ',';
    for (int i = 2; i < 4; i++) { 
        ofs << "Target pressure" << i << "[kPa]" << ',' << "Current pressure" << i << "[kPa]" << ","
            << "Pressure Integral" << i << ','; 
        }
    ofs << "Base Pressure[kPa]" << ',' << "Pdf[kPa]" << ',';
    ofs << "visualP " << visual_P << ',' << "visualI " << visual_I << ',' 
        << "visualD " << visual_D << ',' << "pressureP " << pressure_P << ',' 
        << "pressureI " << pressure_I << ',' << "pressureD " << pressure_D << ','
        << "Link_Angle[deg]" << link_angle_deg << ',';
    ofs << std::endl;
#endif

    while (ros::ok()) {
        //Time processing
        /*timer.now_t = ros::Time::now();
        timer.now_d = timer.now_t - timer.base;
        timer.sampling = timer.now_t - timer.buf;
		timer.buf = timer.now_t;*/
        ros::spinOnce();

        torque = 0.0;
        int key = getch();
        //orientationCurrent:現在の関節角？
        orientationCurrent = LowPassFilterD2(orientationCurrent);

        //全圧力値0
        if(key=='r'){
            action_flag = false;
            setup_flag  = false;
            PressureFeedback_flag = false;
            is_first = true;
            for(int i=0;i<DA_CHANNEL_NUMBER;i++){
                pressureOutput[i] = 0;
            }
        }
        //リンクを膨らませる
        if(key=='l'){
            pressureOutput[link_num]  = linkPressure;
        }
        //初期状態
        if(key=='s'){
            action_flag = false;
            setup_flag = false;
            PressureFeedback_flag = true;
            pressureOutput[0] = 0;
            pressureOutput[1] = 0;
            pressureTarget[2] = basePressure;
            pressureTarget[3] = basePressure;
            pressureOutput[4] = 0;
            pressureOutput[5] = 0;
            pressureOutput[6] = 0;
            pressureOutput[7] = 0;
        }
        //目標位置まで移動
        if(key=='a'){
            action_flag = true;
            setup_flag = false;
            start_orientation = (double)START_ORIENTATION/180*M_PI;//orientationCurrent;
        }
        //開始位置まで移動
        if(key=='b'){
            setup_flag = true;
            action_flag = false;
            start_orientation = (double)START_ORIENTATION/180*M_PI;
        }
        //torqueは[2]に傾く方向が正

        //bを押した後に一回のみ実行される。タイマースタート。
        if(setup_flag && is_first){
            timer.base = ros::Time::now();
            timer.buf = ros::Time::now();
            is_first = false;
        }

        //aを押したとき(目標位置まで移動)
        //または、bを押して一定時間経過したとき。
        if(action_flag){
            //目標角度
            orientationTarget = start_orientation + ((double)TARGET_ORIENTATION/180*M_PI);
            //torque += VisualFeedbackControl((double)timer.now_d.sec + (double)timer.now_d.nsec / 1e9);
        }

        //bを押したとき(開始位置まで移動)
        if(setup_flag){
            orientationTarget = start_orientation;
            timer.now_t = ros::Time::now();
            timer.now_d = timer.now_t - timer.base;
            timer.sampling = timer.now_t - timer.buf;
		    timer.buf = timer.now_t;

            now_time = (float)timer.now_d.sec + (float)timer.now_d.nsec/1e9;
            printf("%lf\n",now_time);
            //時間がたてば目標位置までの移動を開始。
            if(now_time >= change_time){
                orientationTarget = start_orientation + ((double)TARGET_ORIENTATION/180*M_PI);
                //action_flag = true;
            }
            //torque += VisualFeedbackControl();
        }

        //s(初期状態)を押し、rを押すまでの間
        if(PressureFeedback_flag){
            //重力補償トルク
            torque -= CompensateGravity();
            torque += VisualFeedbackControl();
            
            Pdf = convertPdf(orientationTarget,torque);
            printf("Pdf=%lf\n",Pdf);
            pressureTarget[2] = basePressure - Pdf / 2.0F ;
            pressureTarget[3] = basePressure + Pdf / 2.0F ;
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
        ofs << cycle << ","<< (float)timer.now_d.sec + (float)timer.now_d.nsec / 1e9 << ','
            << torque << ',';
        ofs << orientationCurrent / M_PI * 180.0F << ','
            << orientationTarget / M_PI * 180.0F  << ',' 
            << pressureOutput[link_num] << ',';
        for (int i = 2; i < 4; i++) { 
            ofs << pressureTarget[i] << ',' << pressureCurrent[i] << "," << element_pre[i] << ','; 
            }
        //ofs << pressure_P << ',' << pressure_I << ',' << pressure_D << ',' ;
        //ofs << visual_P << ',' << visual_I << ',' << visual_D << ',' ;
        ofs /*<< link_angle_deg << ',' */<< basePressure << ',' << Pdf << ',';
        ofs << element[0] << ',' << element[1] << ',' << element[2] << ',';
        ofs << std::endl;
        }
    #endif
        cycle++;
    }

    return 0;
} //main()


int CheckOverPressure_Target(void) {
	for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        flag_overpressure[i] = false;
		if (flag_overpressure[i] = pressureTarget[i] > maxPressure) {
			pressureTarget[i] = maxPressure;
			ROS_WARN("Over maximum pressure %d\n", i);
		}
        if (flag_overpressure[i] = pressureTarget[i] < MINIMUM_PRESSURE) {
			pressureTarget[i] = MINIMUM_PRESSURE;
			ROS_WARN("Under minimum pressure %d\n", i);
		}
	}

	return 0;
} //CheckOverPressure_Target()

//電空レギュレータによる制限
int CheckOverPressure_Output(void) {
	for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        flag_overpressure[i] = false;
		if (flag_overpressure[i] = pressureOutput[i] > maxPressure) {
			pressureOutput[i] = maxPressure;
			ROS_WARN("Over maximum pressure %d\n", i);
		}
        if (flag_overpressure[i] = pressureOutput[i] < 0) {
			pressureOutput[i] = 0.0;
			ROS_WARN("Under minimum pressure %d\n", i);
		}
	}

	return 0;
} //CheckOverPressure_Output()

//返り値：重力によるトルク
double CompensateGravity() {
    //printf("Tg %f\n",-1.0*0.015*(double)GRAVITY_ACC*0.14*sin(orientationCurrent+45.0F * M_PI / 180.0F));
    double tau_g;
    //リンク本体の重力によるトルク
    tau_g = 0.015 * (double)GRAVITY_ACC * 0.105 * sin(orientationTarget);
    //マーカーの重力によるトルク
    tau_g += 0.005 /*kg*/ * 2 /*個*/ * (double)GRAVITY_ACC * 0.18 /*m*/ * sin(orientationTarget);
    tau_g += 0.005 /*kg*/ * 2 /*個*/ * (double)GRAVITY_ACC * 0.025 /*m*/ * sin(orientationTarget);
    tau_g += 0.005 /*kg*/ * 1 /*個*/ * (double)GRAVITY_ACC * 0.1025 /*m*/ * sin(orientationTarget);
    return tau_g;
} //CompensateGravity()

int DAconversion() {
    double k = (double)DA_Conversion_K;
	for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
		voltageOutput[i] = k * pressureOutput[i];
    }
	return 0;
}

//Voltage - pressure conversion
int ADconversion() {
    double k = (double)AD_Conversion_K;
    double a = (double)AD_Conversion_A;
	for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {
		pressureCurrent[i] = k * voltageInput[i] + a;
	}
	return 0;
}

void LowPassFilter(){
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

    double input[THE_NUMBER_OF_MARKERS_USING][3];
    double output[THE_NUMBER_OF_MARKERS_USING][3];
    static double in1[THE_NUMBER_OF_MARKERS_USING][3], in2[THE_NUMBER_OF_MARKERS_USING][3], out1[THE_NUMBER_OF_MARKERS_USING][3], out2[THE_NUMBER_OF_MARKERS_USING][3]; //0:x, 1:y, 2:z

    for(int i = 0; i < THE_NUMBER_OF_MARKERS_USING; i++){
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
}//LowPassFilter()

double LowPassFilterD(double input){
    double samplerate = (double)SAMPLING_FREQUENCY;  //サンプリング周波数
    double freq = 20;  //カットオフ周波数
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

    output = b0/a0 * input + b1/a0 * in1  + b2/a0 * in2
                            - a1/a0 * out1 - a2/a0 * out2;
    in2  = in1;       // 2つ前の入力信号を更新
    in1 = input;  // 1つ前の入力信号を更新

    out2 = out1;      // 2つ前の出力信号を更新
    out1 = output; // 1つ前の出力信号を更新

    return output;
}//LowPassFilterD()

double LowPassFilterD2(double input){
    double samplerate = (double)SAMPLING_FREQUENCY;  //サンプリング周波数
    double freq = 5;  //カットオフ周波数
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
}//LowPassFilterD2

void World2Local(double *position, double orientation){
    double local[3];
    local[0] = cos(orientation)*position[0] - sin(orientation)*position[1];
    local[1] = sin(orientation)*position[0] + cos(orientation)*position[1];
    for(int i= 0;i<3;i++){
        position[i] = local[i];
    }
}

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
