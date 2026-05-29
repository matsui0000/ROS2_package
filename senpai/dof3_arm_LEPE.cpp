/****************************************************
Name     : dof3_arm_LDPE.cpp
Abstract : ・LDPE3自由度アームの位置制御。
           ・dof1_arm_LDPE.cpp,inflatable_control.cppを基に作成。
           ・キーのそれぞれの対応
            ・r : 全圧力値を0にする。
            ・l : リンクを膨らませる。
            ・s : 初期状態。 
            ．a : 初期状態で関節剛性制御と重力補償トルクを適用．
            ・b : 目標位置まで移動。
           ・動かすときは、l,s,a,b,rの順。
           ・角度は鉛直方向を0度とする．
           ・csvに記録するときはl,s,bの順で押し、一定時間経過後ににrを押す。
Author   : B4 Kota Takenaka  2020/11/17
update   : 2020/11/25
           2020/11/27
           2020/12/02 , 12/04 ,12/05 , 12/07 , 12/07_2 , 12/07_3
           2020/12/08
           2020/12/09  CheckOverPressure_Targetを修正。
           2020/12/09_2
           12/09_3     圧力センサ0,4の出力がおかしい。
                       0は7、4は6を使うように修正。
           2020/12/10 , 12/11 , 12/12 , 12/16
           2020/12/17  1自由度づつ角度制御
           2020/12/18
           2020/12/21  3自由度角度制御(デバック用)
           2020/12/22  , 12/22_2
           2020/12/30  q2を修正．正しいか試す．
           2021/01/04  視覚フィードバックのP項, I項をファイル出力
           2021/01/06 , 01/07 目標圧力が最大値を超えたときのみ警告
           2021/01/12  圧力のP,I,D項それぞれをファイル出力する．
           2021/01/14  
           2021/01/15  aを押すと初期状態に関節剛性と重力補償トルクを適用
           2021/01/19 , 1/20
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

#include <vector> //2020/11/17追加

#define DA_CHANNEL_NUMBER 12
#define AD_CHANNEL_NUMBER 8     //=number of pressure sensor
#define LINK_CHANNEL_NUMBER 8
#define THE_NUMBER_OF_MARKERS 12
#define THE_NUMBER_OF_MARKERS_USING 6

#define DA_Conversion_K 0.1     //Voltage = K * Pressure
#define AD_Conversion_K 25.2    //Pressure = K * Voltage + A
#define AD_Conversion_A -25.4
#define SAMPLING_FREQUENCY 100
#define GRAVITY_ACC 9.80665

#define MINIMUM_PRESSURE 0.0F
#define MAXIMUM_PRESSURE 40.0F
#define MAXIMUM_LINK_PRESSURE 4.0F

#define DEGREE_OF_FREEDOM 3 //自由度

#define BASE_PRESSURE 15.0F
#define LINK_PRESSURE 4.0F

#define HAND_ANGLE 100.0F  //第三リンクの鉛直方向に対する角度[deg]

#define TARGET_STIFFNESS 0.20F  //関節剛性値

#define FILE_OUTPUT //ファイル出力するならコメントアウト外す

//角度制御デバック用
double visual_P_angle = 1.0;
double visual_I_angle = 1.0;
int angle_test_num = 2;            //確認する関節番号
double orientationTarget_deg = 50;  //目標角度[deg]
double orientationTarget = orientationTarget_deg * M_PI /180.0F;

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

class Direction {
public:
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    Direction();
    ~Direction();
};
Direction::Direction() {
}
Direction::~Direction() {
}

class RobotPosition : public Direction {
public:
    Direction base;
	Direction current;
	Direction target;
    RobotPosition();
    ~RobotPosition();
};
RobotPosition::RobotPosition() {
    this->base.x.emplace_back(0.0F);
    this->base.y.emplace_back(0.0F);
    this->base.z.emplace_back(0.0F);
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        this->current.x.emplace_back(0.0F);
        this->current.y.emplace_back(0.0F);
        this->current.z.emplace_back(0.0F);
        this->target.x.emplace_back(0.0F);
        this->target.y.emplace_back(0.0F);
        this->target.z.emplace_back(0.0F);
    }
}
RobotPosition::~RobotPosition() {
}

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

class RobotParameter : public Direction {
public:
    Direction force;                   //Force
    float torque[DEGREE_OF_FREEDOM];   //Torque
    float torque_buf[DEGREE_OF_FREEDOM]; //デバック用
    float torque_g[DEGREE_OF_FREEDOM]; //Compensate gravity torque
    float torque_g_rate;               //Compensate gravity torque rate
    float link[DEGREE_OF_FREEDOM];     //Link length
    float link_g[DEGREE_OF_FREEDOM];   //Length between joint and center of gravity
    float link_l;                      //Lower first link length
    float m_l[DEGREE_OF_FREEDOM];      //Mass of link
    float m_a[DEGREE_OF_FREEDOM];      //Mass of actuator
    float J[3][DEGREE_OF_FREEDOM];     //Jacobian matrix
    float J_T[DEGREE_OF_FREEDOM][3];   //Transposed Jacobian matrix
    float current_yaw;                 //Yaw angle with respect to base coordinate
    float target_yaw;
    float target_stiffness[DEGREE_OF_FREEDOM];  //目標関節剛性
    RobotParameter();
    ~RobotParameter();
};
RobotParameter::RobotParameter():
    torque_g_rate(0.5F),
    link{ 0.10F, 0.153F, 0.155F},
    link_g{0.0F, 0.0F, 0.0F},
    link_l{ 0.210F},
    m_l{0.015F, 0.055F, 0.055F+0.005F*5},
    m_a{0.010F, 0.028F, 0.028F},
    current_yaw(0.0F),
    target_yaw(0.0F),
    target_stiffness{0.25F , 0.25F , 0.25F}
{
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        this->force.x.emplace_back(0.0F);
        this->force.y.emplace_back(0.0F);
        this->force.z.emplace_back(0.0F);
        this->torque[i] = 0.0F;
        this->torque_g[i] = 0.0F;
        for (int j = 0; i < 3; i++) {
           this->J[j][i] = 0.0F;
           this->J_T[i][j] = 0.0F;
        }
    }
}
RobotParameter::~RobotParameter() {
}

class Coefficient {
public:
    float torque[DA_CHANNEL_NUMBER];
   /* const float visual_P;                      //Visual feedback P gain
    const float visual_I;     */                 //Visual feedback I gain 
    const float visual_P[3];  //x,y,zそれぞれのPゲイン
    const float visual_I[3];  //x,y,zそれぞれのIゲイン
    const float pressure_P; //Pressure feedback P gain
    const float pressure_I; //Pressure feedback I gain
    const float pressure_D; //Pressure feedback D gain
    const float AD_a[AD_CHANNEL_NUMBER];       //Voltage - pressure conversion coefficient
    const float AD_b[AD_CHANNEL_NUMBER];
    const float AD_c[AD_CHANNEL_NUMBER];
    const float DA_a[DA_CHANNEL_NUMBER];       //Pressure - voltage conversion coefficient
    const float DA_b[DA_CHANNEL_NUMBER];
    const float DA_c[DA_CHANNEL_NUMBER];
    Coefficient();
    ~Coefficient();
};
Coefficient::Coefficient():
    torque{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },
    /*visual_P(20.0F),
    visual_I(0.0F),*/
    visual_P{15.0F , 15.0F , 15.0F},
    visual_I{10.0F , 10.0F , 10.0F},
    pressure_P(0.03F),
    pressure_I(0.01F),
    pressure_D(0.03F),
    AD_a{   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0 },
	AD_b{   25.2,   25.2,   25.2,   25.2,   25.2,   25.2,   25.2,   25.2 },
	AD_c{  -25.4,  -25.4,  -25.4,  -25.4,  -25.4,  -25.4,  -25.4,  -25.4 },
    DA_a{       0.0,       0.0,       0.0,       0.0,       0.0,       0.0,       0.0,       0.0,        0.0,        0.0,        0.0,        0.0 },
	DA_b{       0.1,       0.1,       0.1,       0.1,       0.1,       0.1,       0.1,       0.1,  0.1,  0.1,  0.1,  0.1 },
	DA_c{       0.0,       0.0,       0.0,       0.0,       0.0,       0.0,       0.0,       0.0,  0.0,  0.0, 0.0,  0.0 }
{
}
Coefficient::~Coefficient() {
}

class Angle {
public:
    std::vector<float> q;
    Angle();
    ~Angle();
};
Angle::Angle() {
}
Angle::~Angle() {
}

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
Pressure::Pressure():
base{   BASE_PRESSURE ,
        BASE_PRESSURE ,
        BASE_PRESSURE ,
        BASE_PRESSURE ,
        BASE_PRESSURE ,
        BASE_PRESSURE ,
        0.0F ,
        0.0F ,
        LINK_PRESSURE,
        LINK_PRESSURE,
        LINK_PRESSURE,
        0.0F }
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

class Flag {
public:
    std::vector<bool> overpressure; //true : safe, false : unsafe
    bool current_reach;             //true : safe, false : unsafe
    bool target_reach;              //true : safe, false : unsafe
    Flag();
    ~Flag();
};
Flag::Flag():
    current_reach(false),
    target_reach(false)
{
    for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {
        this->overpressure.emplace_back(false);
    }
}
Flag::~Flag() {
}

class Voltage {
public:
    std::vector<float> voltage;
    std::vector<float> reliability; //0 : Reliable, -1 : Not reliable
    Voltage();
    ~Voltage();
};
Voltage::Voltage() {
}
Voltage::~Voltage() {
}

class IO : public Voltage {
public:
    Voltage input;
    Voltage output;
    IO();
    ~IO();
};
IO::IO() {
    for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {
        this->input.voltage.emplace_back(0.0F);
        this->input.reliability.emplace_back(0.0F);
    }
    for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        this->output.voltage.emplace_back(0.0F);
        this->output.reliability.emplace_back(0.0F);
    }
}
IO::~IO() {
}


Diviation diviation;
RobotPosition pos;
RobotParameter param;
Coefficient k;
RobotOrientation ornt;
Pressure pressure;
Flag flag;
IO io;
Time timer;

float link_angle = 35.0F * M_PI / 180.0F;      //リンクの膨張角度
float base_link_angle = 35.0F * M_PI / 180.0F; //台座部分

float change_time = 0.5; //初期位置から目標位置まで移動する時間[s]

//ファイル出力用
float element_Pdf[DEGREE_OF_FREEDOM];  
float element_Base[DEGREE_OF_FREEDOM];
float element_visualP[3];             //視覚FBのP項
float element_visualI[3];             //視覚FBのI項
float element_pre[6][3];              //圧力のP,I,D項

unsigned int cycle = 0;


//初期位置[m]
//台座一番下中心を原点としている。
float q0_f  = -10.0F * M_PI / 180.0F;
float q1_f  = 40.0F * M_PI / 180.0F;
float q2_f  = 40.0F * M_PI / 180.0F;
//目標位置[m]
float first_x = (param.link[1]*sinf(q1_f) + param.link[2]*sinf(q1_f+q2_f)) * cosf(q0_f);
float first_y = (param.link[1]*sinf(q1_f) + param.link[2]*sinf(q1_f+q2_f)) * sinf(q0_f);
float first_z = param.link_l + param.link[1] * cosf(q1_f) + param.link[2] * cosf(q1_f+q2_f);

float q0_t  = 10.0F * M_PI / 180.0F;
float q1_t  = 60.0F * M_PI / 180.0F;
float q2_t  = 40.0F * M_PI / 180.0F;

//目標位置[m]
float target_x = (param.link[1]*sinf(q1_t) + param.link[2]*sinf(q1_t+q2_t)) * cosf(q0_t);
float target_y = (param.link[1]*sinf(q1_t) + param.link[2]*sinf(q1_t+q2_t)) * sinf(q0_t);
float target_z = param.link_l + param.link[1] * cosf(q1_t) + param.link[2] * cosf(q1_t+q2_t);


//sensor
double voltageInput[AD_CHANNEL_NUMBER];
double voltageOutput[DA_CHANNEL_NUMBER];

double pressureCurrent[AD_CHANNEL_NUMBER];
double pressureTarget[DA_CHANNEL_NUMBER];
double pressureOutput[DA_CHANNEL_NUMBER];

float positionMarker[THE_NUMBER_OF_MARKERS_USING][3]; //position of base and arm marker
float positionBase[3];   //positionBase[number][x:0, y:1, z:2]
float positionArm[5][3];   //positionArm[number][x:0, y:1, z:2]
float positionOrigin[3];   //原点

float orientationCurrent;

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
float LowPassFilter_angle0(float input);
float LowPassFilter_angle1(float input);
float LowPassFilter_angle2(float input);
void CompensateGravity();
double FeedbackControl();
void convert_R(geometry_msgs::TransformStamped pose,double R[3][3]);
void mul_R(double left[3][3],double right[3][3],double res[3][3]);
void Trans_R(double R[3][3],double res[3][3]);
void print_matrix(double matrix[3][3]);
void print_quartanion(geometry_msgs::TransformStamped pose);
void set_position_target(float x,float y,float z);
void PressureFeedbackControl();
void print_position(float x,float y,float z);
float GetDistance(float x1, float y1, float z1, float x2, float y2, float z2);
void target_inverse();
float JointStiffnessControlBase(float torque , float q , int i);
float JointStiffnessControlDiff(float torque , float q , int i);

//返り値：目標関節剛性にするための基準圧力
float JointStiffnessControlBase(float torque , float q , int i){
    float Pb = 0; //基準圧力
    float L = (float)M_PI - link_angle;        //PI-(リンクの膨らみによる角度)
    if(i==0) L=(float)M_PI - base_link_angle;  //台座部分のアクチュエータ
    float c[5] = {-0.0070497, 0.06649 , -0.21356, 0.2529, -0.2088};
    float Kd = param.target_stiffness[i]; //目標関節剛性

    float alpha = (6.0*c[0]*L + 2.0*c[1])*q;
    float beta = 3.0*c[0]*pow(q, 2.0) + 3.0*c[0]*pow(L, 2.0) + 2.0*c[1]*L + c[2];
    float gamma = (3.0*c[0]*L + c[1])*pow(q, 2.0) + c[0]*pow(L, 3.0) + c[1] * pow(L, 2.0) + c[2]*L + c[3];
    float delta = c[0]*pow(q, 3.0) + (3.0*c[0]*pow(L, 2.0) + 2.0*c[1]*L + c[2])*q;

    Pb = (gamma * Kd + alpha * torque) / (2.0*(alpha * delta - beta * gamma));
    //printf("Pb %f, K %f, t %f\n",Pb,Kd,torque);
    return Pb;
}

//返り値：目標関節剛性にするための差圧
float JointStiffnessControlDiff(float torque , float q , int i){
    float Pdf = 0; //差圧
    float L = (float)M_PI - link_angle;//PI-(リンクの膨らみによる角度)
    if(i==0) L=(float)M_PI - base_link_angle;  //台座部分のアクチュエータ
    float c[5] = {-0.0070497, 0.06649 , -0.21356, 0.2529, -0.2088};
    float Kd = param.target_stiffness[i]; //目標関節剛性

    float alpha = (6.0*c[0]*L + 2.0*c[1])*q;
    float beta = 3.0*c[0]*pow(q, 2.0) + 3.0*c[0]*pow(L, 2.0) + 2.0*c[1]*L + c[2];
    float gamma = (3.0*c[0]*L + c[1])*pow(q, 2.0) + c[0]*pow(L, 3.0) + c[1] * pow(L, 2.0) + c[2]*L + c[3];
    float delta = c[0]*pow(q, 3.0) + (3.0*c[0]*pow(L, 2.0) + 2.0*c[1]*L + c[2])*q;

    Pdf = (delta * Kd + beta * torque) / (beta * gamma - alpha * delta);
    //printf("Pdf %f, q %f, t %f, a %f, b %f, c %f, d %f\n",Pdf,q,torque,alpha,beta,gamma,delta);
    return Pdf;
}

//座標表示(デバック用)
void print_position(float x,float y,float z){
    printf("x=%lf , y=%lf , z=%lf\n",x,y,z);
}

//マーカー座標から現在の関節角度を求める。
void msgCallback(const vicon_bridge::Markers markers_msg){
    unsigned int count = 0;
    double orientationBuf;

    vicon_bridge::Marker Arr[THE_NUMBER_OF_MARKERS];

    for(std::vector<vicon_bridge::Marker>::const_iterator it = markers_msg.markers.begin(); it != markers_msg.markers.end(); ++it){
        Arr[count] = *it;
        if(Arr[count].marker_name == "Base1"){
            positionMarker[0][0] = Arr[count].translation.x / 1000.0F;
            positionMarker[0][1] = Arr[count].translation.y / 1000.0F;
            positionMarker[0][2] = Arr[count].translation.z / 1000.0F;
        }
        if(Arr[count].marker_name == "Base2"){
            positionMarker[1][0] = Arr[count].translation.x / 1000.0F;
            positionMarker[1][1] = Arr[count].translation.y / 1000.0F;
            positionMarker[1][2] = Arr[count].translation.z / 1000.0F;
        }
        if(Arr[count].marker_name == "Hand1"){
            positionMarker[2][0] = Arr[count].translation.x / 1000.0F;
            positionMarker[2][1] = Arr[count].translation.y / 1000.0F;
            positionMarker[2][2] = Arr[count].translation.z / 1000.0F;
        }
        if(Arr[count].marker_name == "Hand2"){
            positionMarker[3][0] = Arr[count].translation.x / 1000.0F;
            positionMarker[3][1] = Arr[count].translation.y / 1000.0F;
            positionMarker[3][2] = Arr[count].translation.z / 1000.0F;
        }
        if(Arr[count].marker_name == "Hand3"){
            positionMarker[4][0] = Arr[count].translation.x / 1000.0F;
            positionMarker[4][1] = Arr[count].translation.y / 1000.0F;
            positionMarker[4][2] = Arr[count].translation.z / 1000.0F;
        }
        if(Arr[count].marker_name == "Hand4"){
            positionMarker[5][0] = Arr[count].translation.x / 1000.0F;
            positionMarker[5][1] = Arr[count].translation.y / 1000.0F;
            positionMarker[5][2] = Arr[count].translation.z / 1000.0F;
        }
        count++;
        if(count > THE_NUMBER_OF_MARKERS)break;
    }

    LowPassFilter();

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
    
    //現在の位置を出す。
    for(int i = 1; i < DEGREE_OF_FREEDOM ; i++){
        pos.current.x[i] = (positionArm[i*2-2][0] + positionArm[i*2-1][0])*0.5F;
        pos.current.y[i] = (positionArm[i*2-2][1] + positionArm[i*2-1][1])*0.5F;
        pos.current.z[i] = (positionArm[i*2-2][2] + positionArm[i*2-1][2])*0.5F;
    }  

    float beta  = 0.0;
    float gamma = 0.0;
    float alpha = 0.0;
    float x2 = pos.current.x[2];
    float y2 = pos.current.y[2];
    float z2 = pos.current.z[2];
    float l1 = param.link_l;
    float l2 = param.link[1];
    float l3 = param.link[2];
    //以下関節角度を出していく。
    
    /*//デバック用
    float theta0  = 20.0F * M_PI / 180.0F;
    float theta1  = 50.0F * M_PI / 180.0F;
    float theta2  = 50.0F * M_PI / 180.0F;
    //目標位置[m]
    x2 = (param.link[1]*sinf(theta1) + param.link[2]*sinf(theta1+theta2)) * cosf(theta0);
    y2 = (param.link[1]*sinf(theta1) + param.link[2]*sinf(theta1+theta2)) * sinf(theta0);
    z2 = param.link_l + param.link[1] * cosf(theta1) + param.link[2] * cosf(theta1+theta2);*/


    ornt.buf.q[0] = atan2f(y2,x2);

    ornt.buf.q[2] = acosf((powf(x2,2.0F)+powf(y2,2.0F)+powf(z2-l1,2.0F) - powf(l2,2.0F) - powf(l3,2.0F))
                          /(2*l2*l3));
    
    float n1 = sqrtf(powf(x2,2.0F)+powf(y2,2.0F));

    ornt.buf.q[1] = atan2f((l2+l3*cosf(ornt.buf.q[2]))*n1
                            -l3*sinf(ornt.buf.q[2])*(z2-l1)
                            , l3*sinf(ornt.buf.q[2])*n1
                           +(l2+l3*cosf(ornt.buf.q[2]))*(z2-l1));

    //yawは(-π/2 , π/2)の範囲
    if((ornt.buf.q[0] < -M_PI/2.0F) || (ornt.buf.q[0] > M_PI/2.0F)){
        ornt.buf.q[0] = ornt.current.q[0];
    }

    //計算した関節角度がNANでなければcurrentに格納する。
    for(int i=0 ; i < DEGREE_OF_FREEDOM ; i++){
        if(!(std::isnan(ornt.buf.q[i]))){
            ornt.current.q[i] = ornt.buf.q[i];
        }
        else{
            printf("\e[32m関節角%d=NAN\e[m\n",i);
            }
    }

    //以下デバック用
    //print_position(positionOrigin[0],positionOrigin[1],positionOrigin[2]);
    /*for(int i = 0; i < 4; i++){
        printf("%d番目のアームマーカー  ",i);
        print_position(positionArm[i][0],positionArm[i][1],positionArm[i][2]);
    }*/
   
    for(int i = 1; i < 2; i++){
        printf("現在の%dの位置  ",i);
        print_position(pos.current.x[i],pos.current.y[i],pos.current.z[i]);
    }
    for(int i = 0; i < DEGREE_OF_FREEDOM; i++){
        printf("現在の%dの関節角[deg]  %lf\n",i,ornt.current.q[i]*180.0F/M_PI);
    }
    printf("現在の手先位置x=%lf,y=%lf,z=%lf\n",x2,y2,z2);
    printf("手先までの長さ実際：%lf\n",sqrtf(powf(x2,2)+powf(y2,2)+powf(z2-l1,2)));
    printf("手先までの長さの最大値：%lf\n",l2+l3);
   // printf("acosの中身%lf\n",(pos.current.z[1] - param.link_l) / (param.link[2]));
    printf("\n");  
}


//subscribe voltage
void msgCallback2(const inflatable::VoltageInput::ConstPtr& sub_msg) {
    for(int i = 0; i < AD_CHANNEL_NUMBER; i++) {
        io.input.voltage[i] = sub_msg->voltage_input[i];
    }

    //Conversion from voltage to preassure
    ADconversion();
} //msgCallback2()

//デバック用目標位置から目標関節角を出す．
void target_inverse(){
    float x2 = pos.target.x[2];
    float y2 = pos.target.y[2];
    float z2 = pos.target.z[2];
    float l1 = param.link_l;
    float l2 = param.link[1];
    float l3 = param.link[2];

    //以下関節角度を出していく。

    ornt.target.q[0] = atan2f(y2,x2);

    ornt.target.q[2] = acosf((powf(x2,2.0F)+powf(y2,2.0F)+powf(z2-l1,2.0F) - powf(l2,2.0F) - powf(l3,2.0F))
                          /(2*l2*l3));
    
    float n1 = sqrtf(powf(x2,2.0F)+powf(y2,2.0F));

    ornt.target.q[1] = atan2f((l2+l3*cosf(ornt.target.q[2]))*n1
                            -l3*sinf(ornt.target.q[2])*(z2-l1)
                            , l3*sinf(ornt.target.q[2])*n1
                           +(l2+l3*cosf(ornt.target.q[2]))*(z2-l1));
} //target_inverse()

geometry_msgs::TransformStamped base_pose;
bool msgCallback_base_flag = false;

void set_position_target(float x,float y,float z){
    pos.target.x[DEGREE_OF_FREEDOM-1] = x;
    pos.target.y[DEGREE_OF_FREEDOM-1] = y;
    pos.target.z[DEGREE_OF_FREEDOM-1] = z;
} //set_position_target()

//デバック用の角度制御
void VisualFeedbackControl_angle(){
    static unsigned int FB_cycle = 0;
    //Visual Feedback
    static float orientationCurrent_buf[3];
    static float orientationTarget_buf[3];
    static float orientationIntegral[3];
    double torque;
    float Pdf;

    diviation.P.x[0] = pos.target.x[DEGREE_OF_FREEDOM-1] - pos.current.x[DEGREE_OF_FREEDOM-1];
    diviation.P.y[0] = pos.target.y[DEGREE_OF_FREEDOM-1] - pos.current.y[DEGREE_OF_FREEDOM-1];
    diviation.P.z[0] = pos.target.z[DEGREE_OF_FREEDOM-1] - pos.current.z[DEGREE_OF_FREEDOM-1];


    for(int i=0 ; i<3 ; i++){
    if (FB_cycle > 2) {
            orientationIntegral[i] += ((orientationTarget_buf[i] - orientationCurrent_buf[i])
                                    + (ornt.target.q[i] - ornt.current.q[i]))
                                    / (2.0F * SAMPLING_FREQUENCY);
    }
    //elementはファイル出力のためのもの。
    if (FB_cycle > 1) {
            //P component
            param.torque[i] = visual_P_angle * (ornt.target.q[i] - ornt.current.q[i]);
            param.torque_buf[i] = visual_P_angle * (ornt.target.q[i] - ornt.current.q[i]);
            //I component
            param.torque[i] += visual_I_angle * orientationIntegral[i];
            param.torque_buf[i] += visual_I_angle * orientationIntegral[i];
            //element[1] = visual_I_angle * orientationIntegral;
    }

    orientationTarget_buf[i] = ornt.target.q[i];
    orientationCurrent_buf[i] = ornt.current.q[i];
    }

    //重力補償トルクを求める。
   CompensateGravity();
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        param.torque_buf[i] += param.torque_g[i];
        param.torque[i] += param.torque_g[i];
    }

    float basepressure = 0.0;
    float diffpressure = 0.0;
    
    for(int i=0 ; i < DEGREE_OF_FREEDOM ; i++){
        //torqueは関節角度が増える方向が正。
        //膨らむと関節角度が増える方がPb+1/2Pdf
        basepressure = JointStiffnessControlBase(param.torque[i] , ornt.current.q[i] , i);
        diffpressure = JointStiffnessControlDiff(param.torque[i] , ornt.current.q[i] , i);

        element_Pdf[i] = diffpressure;
        element_Base[i] = basepressure;

        pressure.target[i*2]   = basepressure + diffpressure / 2.0F;
        pressure.target[i*2+1] = basepressure - diffpressure / 2.0F;
        }
    

    FB_cycle++;
} //VisualFeedbackControl_angle()

int VisualFB_cycle = 0;
//返り値：視覚フィードバックによるトルク
int VisualFeedbackControl(){
    static unsigned int FB_cycle = 0;
    float Pdf=0.0;

    //P制御
    diviation.P.x[0] = pos.target.x[2] - pos.current.x[2];
    diviation.P.y[0] = pos.target.y[2] - pos.current.y[2];
    diviation.P.z[0] = pos.target.z[2] - pos.current.z[2];

    //I制御
    if (VisualFB_cycle > 2) {
        diviation.I.x[0] += (diviation.P_buf.x[0] + diviation.P.x[0]) / (2.0F * SAMPLING_FREQUENCY);
        diviation.I.y[0] += (diviation.P_buf.y[0] + diviation.P.y[0]) / (2.0F * SAMPLING_FREQUENCY);
        diviation.I.z[0] += (diviation.P_buf.z[0] + diviation.P.z[0]) / (2.0F * SAMPLING_FREQUENCY);
    }
    //次のI制御用に更新
    diviation.P_buf.x[0] = diviation.P.x[0];
    diviation.P_buf.y[0] = diviation.P.y[0];
    diviation.P_buf.z[0] = diviation.P.z[0];

    //Target force
    /*param.force.x[0] = k.visual_P * diviation.P.x[0] + k.visual_I * diviation.I.x[0];
    param.force.y[0] = k.visual_P * diviation.P.y[0] + k.visual_I * diviation.I.y[0];
    param.force.z[0] = k.visual_P * diviation.P.z[0] + k.visual_I * diviation.I.z[0];

    //ファイル出力用
    element_visualP[0] = k.visual_P * diviation.P.x[0];
    element_visualP[1] = k.visual_P * diviation.P.y[0];
    element_visualP[2] = k.visual_P * diviation.P.z[0];

    element_visualI[0] = k.visual_I * diviation.I.x[0];
    element_visualI[1] = k.visual_I * diviation.I.y[0];
    element_visualI[2] = k.visual_I * diviation.I.z[0];*/

    //Target force
    param.force.x[0] = k.visual_P[0] * diviation.P.x[0] + k.visual_I[0] * diviation.I.x[0];
    param.force.y[0] = k.visual_P[1] * diviation.P.y[0] + k.visual_I[1] * diviation.I.y[0];
    param.force.z[0] = k.visual_P[2] * diviation.P.z[0] + k.visual_I[2] * diviation.I.z[0];

    //ファイル出力用
    element_visualP[0] = k.visual_P[0] * diviation.P.x[0];
    element_visualP[1] = k.visual_P[1] * diviation.P.y[0];
    element_visualP[2] = k.visual_P[2] * diviation.P.z[0];

    element_visualI[0] = k.visual_I[0] * diviation.I.x[0];
    element_visualI[1] = k.visual_I[1] * diviation.I.y[0];
    element_visualI[2] = k.visual_I[2] * diviation.I.z[0];

    //Jacobi matrix
    param.J[0][0] = - sinf(ornt.current.q[0])
                    * (param.link[1]*sinf(ornt.current.q[1]) + param.link[2]*sinf(ornt.current.q[1]+ornt.current.q[2])); 
    
    param.J[0][1] = cosf(ornt.current.q[0]) 
                    * (param.link[1]*cosf(ornt.current.q[1]) + param.link[2]*cosf(ornt.current.q[1]+ornt.current.q[2]));
    
    param.J[0][2] = param.link[2] * cosf(ornt.current.q[0]) * cosf(ornt.current.q[1]+ornt.current.q[2]);

    param.J[1][0] = cosf(ornt.current.q[0]) 
                    * (param.link[1]*sinf(ornt.current.q[1]) + param.link[2]*sinf(ornt.current.q[1]+ornt.current.q[2])); 
    
    param.J[1][1] = sinf(ornt.current.q[0]) 
                    * (param.link[1]*cosf(ornt.current.q[1]) + param.link[2]*cosf(ornt.current.q[1]+ornt.current.q[2]));
    
    param.J[1][2] = param.link[2] * sinf(ornt.current.q[0]) * cosf(ornt.current.q[1]+ornt.current.q[2]);

    param.J[2][0] = 0; 
    
    param.J[2][1] = - param.link[1] * sinf(ornt.current.q[1]) - param.link[2] * sinf(ornt.current.q[1]+ornt.current.q[2]);
    
    param.J[2][2] = - param.link[2] * sinf(ornt.current.q[1]+ornt.current.q[2]);
       
    //Inverse Jacobian matrix
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < DEGREE_OF_FREEDOM; j++) {
            param.J_T[j][i] = param.J[i][j];
            }
        }

    //Force - torque conversion
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        param.torque[i] = param.J_T[i][0] * param.force.x[0]
                        + param.J_T[i][1] * param.force.y[0]
                        + param.J_T[i][2] * param.force.z[0];
    }

    for(int i=0; i < DEGREE_OF_FREEDOM ; i++){
        printf("torque%d=%lf\n",i,param.torque[i]);
    }

    //重力補償トルクを求める。
    CompensateGravity();
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        //param.torque[i] += param.torque_g_rate * param.torque_g[i];
        param.torque[i] += param.torque_g[i];
    }

    float basepressure = 0.0;
    float diffpressure = 0.0;
    for(int i=0 ; i < DEGREE_OF_FREEDOM ; i++){
        //torqueは関節角度が増える方向が正。
        //膨らむと関節角度が増える方がPb+1/2Pdf
        basepressure = JointStiffnessControlBase(param.torque[i] , ornt.current.q[i] , i);
        diffpressure = JointStiffnessControlDiff(param.torque[i] , ornt.current.q[i] , i);

        element_Pdf[i] = diffpressure;
        element_Base[i] = basepressure;

        pressure.target[i*2]   = basepressure + diffpressure / 2.0F;
        pressure.target[i*2+1] = basepressure - diffpressure / 2.0F;
        }

    FB_cycle++;

    VisualFB_cycle++;

    return 0;
} //VisualFeedbackControl()

int first_position(){

    //重力補償トルクを求める。
    CompensateGravity();
    for (int i = 0; i < DEGREE_OF_FREEDOM; i++) {
        param.torque[i] = param.torque_g[i];
    }

    float basepressure = 0.0;
    float diffpressure = 0.0;
    for(int i=0 ; i < DEGREE_OF_FREEDOM ; i++){
        //torqueは関節角度が増える方向が正。
        //膨らむと関節角度が増える方がPb+1/2Pdf
        basepressure = JointStiffnessControlBase(param.torque[i] , ornt.current.q[i] , i);
        diffpressure = JointStiffnessControlDiff(param.torque[i] , ornt.current.q[i] , i);

        element_Pdf[i] = diffpressure;
        element_Base[i] = basepressure;

        pressure.target[i*2]   = basepressure + diffpressure / 2.0F;
        pressure.target[i*2+1] = basepressure - diffpressure / 2.0F;
        }

    return 0;
} //first_position()

int PressureFB_cycle = 0;
void PressureFeedbackControl(){
    static unsigned int FB_cycle = 0;

    for (int i = 0; i < DEGREE_OF_FREEDOM*2; i++) {
        pressure.output[i] = pressure.target[i] ;
        if (PressureFB_cycle > 2) {
            diviation.pressure_integral[i] += ((pressure.target_buf[i] - pressure.current_buf[i])
                                       + (pressure.target[i] - pressure.current[i]))
                                       / (2.0F * SAMPLING_FREQUENCY);
        }

        if (PressureFB_cycle > 1) {
                //P component
                pressure.P[i] = k.pressure_P * (pressure.target[i] - pressure.current[i]);
                pressure.output[i] += pressure.P[i];
                element_pre[i][0] = k.pressure_P * (pressure.target[i] - pressure.current[i]);

                //I component
                pressure.I[i] = k.pressure_I * diviation.pressure_integral[i];
                pressure.output[i] += pressure.I[i];
                element_pre[i][1] = k.pressure_I * diviation.pressure_integral[i];

                //D component
                pressure.D[i] = -1.0F * k.pressure_D * (pressure.current[i] - pressure.current_buf[i]);
                pressure.output[i] += pressure.D[i];
                element_pre[i][2] = -1.0F * k.pressure_D * (pressure.current[i] - pressure.current_buf[i]);

                /*//Update previous value
                pressure.target_buf[i] = pressure.target[i];
                pressure.current_buf[i] = pressure.current[i];*/
        }
                //Update previous value
                pressure.target_buf[i] = pressure.target[i];
                pressure.current_buf[i] = pressure.current[i];
    }
    PressureFB_cycle++;
    return;
} //PressureFeedbackControl()

void PressureFeedback_reset(){
    PressureFB_cycle = 0;
    for(int i=0 ; i<DEGREE_OF_FREEDOM*2 ; i++){
        diviation.pressure_integral[i] = 0;
    }
} //PressureFeedback_reset

int main(int argc, char **argv) {
    printf("aaaa\n");
    ros::init(argc, argv, "dof3_arm_LDPE");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");
    //VoltageOutputメッセージファイルを使用
    //トピック名はvoltage_output
    ros::Publisher ros_pub = nh.advertise<inflatable::VoltageOutput>("voltage_output", 10);

    //トピック名は/vicon/markers ?
    ros::Subscriber ros_sub = nh.subscribe("/vicon/markers", 10, msgCallback);
    //トピック名はvoltage_input
    ros::Subscriber ros_sub2 = nh.subscribe("voltage_input", 10, msgCallback2);

    ros::Rate loop_rate(SAMPLING_FREQUENCY);
    //VoltageOutput.msgに記載した形式のmsgを宣言
    inflatable::VoltageOutput pub_msg;
    timer.base = ros::Time::now();
    timer.buf = ros::Time::now();

    for(int i=0;i<DA_CHANNEL_NUMBER;i++){
        pressure.output[i] = 0;
    }
    for(int i=0;i<DA_CHANNEL_NUMBER;i++){
        pressure.target[i] = 0;
    }

    static bool setup_flag = false;
    static bool PressureFeedback_flag = false;
    static bool VisualFeedback_flag = false;
    static bool angle_debug_flag = false;
    static bool file_output_flag = false;
    static bool a_flag = false;
    static double start_time;
    static double action_start_time;
    static double start_orientation;
    double torque;
    double Pdf;
    double now_time = 0.0;
    static bool is_first = true;
    static bool is_first2 = true;
    float angle_buf = 0.0;

    //File output preparation
#ifdef FILE_OUTPUT

    char filename[64];
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

    ofs << "cycle" << ',' << "Time[s]" << ',';

    ofs << "x2[m]" << ',' << "y2[m]" << ',' << "z2[m]" << ','
        << "xt[m]" << ',' << "yt[m]" << ',' << "zt[m]" << ','
        << "diviation x[m]" << ',' << "diviation y[m]" << ',' << "diviation z[m]" << ',' 
        << "diviation[m]" << ','
        << "x1[m]" << ',' << "y1[m]" << ',' << "z1[m]" << ','
        << "qc0[deg]" << ',' << "qt0[deg]" << ',' 
        << "qc1[deg]" << ',' << "qt1[deg]" << ',' 
        << "qc2[deg]" << ',' << "qt2[deg]" << ',' 
        << "Fx[N]" << ',' << "Fy[N]" << ',' << "Fz[N]" << ',' 
        << "torque0[Nm]" << ',' << "torque0_angle[Nm]" << ','
        << "torque1[Nm]" << ',' << "torque1_angle[Nm]" << ','
        << "torque2[Nm]" << ',' << "torque2_angle[Nm]" << ','
        << "torque_g0[Nm]" << ',' << "torque_g1[Nm]" << ',' << "torque_g2[Nm]" << ',' ;

    /*ofs << "Visual_P_x " << k.visual_P << ',' << "Visual_I_x " << k.visual_I << ','
        << "Visual_P_y " << k.visual_P << ',' << "Visual_I_y " << k.visual_I << ','
        << "Visual_P_z " << k.visual_P << ',' << "Visual_I_z " << k.visual_I << ',' ;*/

    ofs << "Visual_P_x " << k.visual_P[0] << ',' << "Visual_I_x " << k.visual_I[0] << ','
        << "Visual_P_y " << k.visual_P[1] << ',' << "Visual_I_y " << k.visual_I[1] << ','
        << "Visual_P_z " << k.visual_P[2] << ',' << "Visual_I_z " << k.visual_I[2] << ',' ;

    for (int i = 0; i < 6; i++) { 
        ofs << "Target pressure" << i << "[kPa]" << ',' << "PressureOutput" << i << "[kPa]" << ',' 
            << "Current pressure" << i << "[kPa]" << ","; 
        }
    for (int i=0; i<DEGREE_OF_FREEDOM ; i++){
        ofs << "basePressure" << i << ','
            << "Pdf" << i << ',';
    }
    for(int i=0; i<3 ; i++){
        for(int j=0; j<DEGREE_OF_FREEDOM ; j++){
            ofs << "J_T" << i << j << ",";
        }
    }
    for(int i=0; i<6 ; i++){
        ofs << "Actuator" << i << "_P" << ',' << "Actuator" << i << "_I" << ','<< "Actuator" << i << "_D" << ',';
    }
    ofs << "Link Output[kPa]"     << ',';
    ofs << "Link_Angle[deg]" << link_angle * 180.0F / M_PI << ',';
    for(int i=0 ; i<3 ; i++){
        ofs << "Target_Stiff" << i << "= " << param.target_stiffness[i] << ','; 
    }
    ofs << "Pressure_P= " << k.pressure_P << ',' << "Pressure_I= " << k.pressure_I << ',' << "Pressure_D= " << k.pressure_D << ',';
    ofs << std::endl;
#endif
    
    while (ros::ok()) {
        ros::spinOnce();
        int key = getch();

        //全圧力値0
        if(key=='r'){
            setup_flag  = false;
            VisualFeedback_flag = false;
            PressureFeedback_flag = false;
            file_output_flag = false;
            angle_debug_flag = false;
            is_first = true;
            for(int i=0;i<DA_CHANNEL_NUMBER;i++){
                pressure.output[i] = 0;
                pressure.target[i] = 0;
            }
        }
        //リンクを膨らませる
        if(key=='l'){
            pressure.output[LINK_CHANNEL_NUMBER]  = pressure.base[LINK_CHANNEL_NUMBER];
        }

        //デバック用角度制御
        if(key=='d'){
            PressureFeedback_flag = true;
            angle_debug_flag = true;
            file_output_flag = true;
            set_position_target(target_x,target_y,target_z);
            target_inverse();
        }

        //初期状態
        if(key=='s'){
            setup_flag = false;
            for(int i=0; i < DA_CHANNEL_NUMBER; i++){
                if(i==3){
                    pressure.output[i] = pressure.base[i] - 7.0F;
                }
                else if(i==5){
                    pressure.output[i] = pressure.base[i] - 10.0F;
                }
                else if(i==4){
                    pressure.output[i] = pressure.base[i] + 3.0F;
                }
                else{
                    pressure.output[i] = pressure.base[i];
                }
            }
        }

        //初期位置で関節剛性制御に切り替え
        if(key=='a'){
            PressureFeedback_flag = true;
            //first_position();
            a_flag = true;
        }
        
        if(key=='b'){
            setup_flag = true;
            VisualFeedback_flag = true;
            PressureFeedback_reset();
            PressureFeedback_flag = true;
            a_flag = false;
            file_output_flag = true;
            set_position_target(target_x,target_y,target_z);
        }

        //aを押した時
        if(a_flag){
            first_position();
        }

        //bを押した後に一回のみ実行される。タイマースタート。
        if(setup_flag && is_first){
            timer.base = ros::Time::now();
            timer.buf = ros::Time::now();
            is_first = false;
        }

        //bを押したとき
        if(setup_flag){
            timer.now_t = ros::Time::now();
            timer.now_d = timer.now_t - timer.base;
            timer.sampling = timer.now_t - timer.buf;
		    timer.buf = timer.now_t;

            now_time = (float)timer.now_d.sec + (float)timer.now_d.nsec/1e9;
            printf("%lf\n",now_time);

            diviation.P.x[0] = pos.target.x[2] - pos.current.x[2];
            diviation.P.y[0] = pos.target.y[2] - pos.current.y[2];
            diviation.P.z[0] = pos.target.z[2] - pos.current.z[2];
            
           /*if(is_first2){
                if(now_time >= change_time){
                    VisualFeedback_flag = true;
                    PressureFeedback_reset();
                    is_first2 = false;
                    }
            }*/

        }

        //デバッグ用
        if(angle_debug_flag){
            VisualFeedbackControl_angle();
        }

       
        if(VisualFeedback_flag){
            ornt.current.q[0] = LowPassFilter_angle0(ornt.current.q[0]);
            ornt.current.q[1] = LowPassFilter_angle1(ornt.current.q[1]);
            ornt.current.q[2] = LowPassFilter_angle2(ornt.current.q[2]);
            VisualFeedbackControl();

            //デバッグ用
            //target_inverse();
            //VisualFeedbackControl_angle();
        }

        if(PressureFeedback_flag){
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
            pub_msg.voltage_output.emplace_back(io.output.voltage[i]);
        }
        ros_pub.publish(pub_msg);
        loop_rate.sleep();

        //File output
    #ifdef FILE_OUTPUT

        if(file_output_flag){
        ofs << cycle << ","<< (float)timer.now_d.sec + (float)timer.now_d.nsec / 1e9 << ',';

        ofs << pos.current.x[2] << ',' << pos.current.y[2] << ',' << pos.current.z[2] << ','
            << pos.target.x[2] << ',' << pos.target.y[2] << ',' << pos.target.z[2] << ','
            << diviation.P.x[0] << ',' << diviation.P.y[0] << ',' << diviation.P.z[0] << ',' 
            << sqrt(pow(diviation.P.x[0],2.0)+pow(diviation.P.y[0],2.0)+pow(diviation.P.z[0],2.0)) << ','
            << pos.current.x[1] << ',' << pos.current.y[1] << ',' << pos.current.z[1] << ','
            << ornt.current.q[0]*180.0F/M_PI << ',' << q0_t*180.0F/M_PI << ','
            << ornt.current.q[1]*180.0F/M_PI << ',' << q1_t*180.0F/M_PI << ','
            << ornt.current.q[2]*180.0F/M_PI << ',' << q2_t*180.0F/M_PI << ',' 
            << param.force.x[0] << ',' << param.force.y[0] << ',' << param.force.z[0] << ',' 
            << param.torque[0] << ',' << param.torque_buf[0] << ','
            << param.torque[1] << ',' << param.torque_buf[1] << ','
            << param.torque[2] << ',' << param.torque_buf[2] << ','
            << param.torque_g[0] << ',' << param.torque_g[1] << ',' << param.torque_g[2] << ',' ;

        for(int i=0 ; i<3 ; i++){
            ofs << element_visualP[i] << ',' << element_visualI[i] << ',' ;
        }
            
        for (int i = 0; i < 6; i++) { 
            ofs << pressure.target[i] << ',' << pressure.output[i] << ',' 
                << pressure.current[i] << "," ; 
            }
        for (int i=0; i < DEGREE_OF_FREEDOM ; i++){
            ofs << element_Base[i] << ','
                << element_Pdf[i] << ',';
            }    
        for(int i=0; i<3 ; i++){
         for(int j=0; j<DEGREE_OF_FREEDOM ; j++){
            ofs << param.J_T[i][j] << ",";
            }
        }
        for(int i=0; i<6 ; i++){
            for(int j=0 ; j<3 ; j++){
                ofs << element_pre[i][j] << ',';
            }
        }
        ofs << pressure.output[LINK_CHANNEL_NUMBER] << ',';
        ofs << std::endl;
        }
    #endif
        cycle++;
    }

    return 0;
} //main()

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


//圧力目標値の制限
int CheckOverPressure_Target(void) {
    for(int i = 0 ; i < DA_CHANNEL_NUMBER ; i++){
        if(pressure.target[i] > MAXIMUM_PRESSURE){
            pressure.target[i] = MAXIMUM_PRESSURE;
            flag_overpressure[i] = false;
            ROS_WARN("Over maximum pressure %d\n", i);
        }
        else{
            flag_overpressure[i] = true;
        }
    }

	for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
		if (pressure.target[i] < MINIMUM_PRESSURE) {
			pressure.target[i] = MINIMUM_PRESSURE;
			flag.overpressure[i] = false;
			//ROS_WARN("Under minimum pressure %d\n", i);
		}
		else {
			flag.overpressure[i] = true;
		}
	}

	return 0;
} //CheckOverPressure_Target()


//電空レギュレータによる制限
int CheckOverPressure_Output(void) {
    for(int i = 0; i < DA_CHANNEL_NUMBER ; i++){
        if(pressure.output[i] > MAXIMUM_PRESSURE){
            pressure.output[i] = MAXIMUM_PRESSURE;
            flag_overpressure[i] = false;
            //ROS_WARN("Over maximum pressure %d\n", i);
        }
        else{
            flag_overpressure[i] = true;
        }
    }

	for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
		if (pressure.output[i] < MINIMUM_PRESSURE) {
			pressure.output[i] = MINIMUM_PRESSURE;
			flag.overpressure[i] = false;
			//ROS_WARN("Under minimum pressure %d\n", i);
		}
		else {
			flag.overpressure[i] = true;
		}
	}

	return 0;
} //CheckOverPressure_Output()

//Distance between two points
float GetDistance(float x1, float y1, float z1, float x2, float y2, float z2) {
	float distance;
	distance = pow((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1), 0.5F);

	return distance;
}

//返り値：重力によるトルク
void CompensateGravity() {
    float m[DEGREE_OF_FREEDOM];   //Mass summation of link and actuator
	float x_l[DEGREE_OF_FREEDOM]; //Link center of gravity
	float y_l[DEGREE_OF_FREEDOM];
	float z_l[DEGREE_OF_FREEDOM];
	float x_g[DEGREE_OF_FREEDOM]; //Link and actuator center of gravity
	float y_g[DEGREE_OF_FREEDOM];
	float z_g[DEGREE_OF_FREEDOM];

    //現在のそれぞれのリンクの重心座標を求める。
    x_l[1] = (1.0F / 2.0F) * param.link[1] * sinf(ornt.current.q[1]) * cosf(ornt.current.q[0]);
    y_l[1] = (1.0F / 2.0F) * param.link[1] * sinf(ornt.current.q[1]) * sinf(ornt.current.q[0]);
    z_l[1] = (1.0F / 2.0F) * param.link[1] * cosf(ornt.current.q[1]);
    x_l[2] = pos.current.x[1] + (1.0F / 2.0F) * param.link[2] * sinf(ornt.current.q[1] + ornt.current.q[2]) * cosf(ornt.current.q[0]);
	y_l[2] = pos.current.y[1] + (1.0F / 2.0F) * param.link[2] * sinf(ornt.current.q[1] + ornt.current.q[2]) * sinf(ornt.current.q[0]);
	z_l[2] = pos.current.z[1] + (1.0F / 2.0F) * param.link[2] * cosf(ornt.current.q[1] + ornt.current.q[2]);

    //リンクとアクチュエータを合わせた重心座標を求める。
    for (int i = 1; i < DEGREE_OF_FREEDOM; i++) {
		x_g[i] = (param.m_l[i] * x_l[i] + param.m_a[i] * pos.current.x[i]) / (param.m_l[i] + param.m_a[i]);
		y_g[i] = (param.m_l[i] * y_l[i] + param.m_a[i] * pos.current.y[i]) / (param.m_l[i] + param.m_a[i]);
		z_g[i] = (param.m_l[i] * z_l[i] + param.m_a[i] * pos.current.z[i]) / (param.m_l[i] + param.m_a[i]);
		m[i] = param.m_l[i] + param.m_a[i];
		param.link_g[i] = GetDistance(x_g[i], y_g[i], z_g[i], pos.current.x[i], pos.current.y[i], pos.current.z[i]);
	}

    //重力補償トルクを求める。
    param.torque_g[0] = 0.0F;
    param.torque_g[1] = (-1 * m[1] * GRAVITY_ACC * param.link_g[1] - m[2] * GRAVITY_ACC * param.link[1]) * sinf(ornt.current.q[1])
		              + (-1 * m[2] * GRAVITY_ACC * param.link_g[2]) * sinf(ornt.current.q[1] + ornt.current.q[2]);
    param.torque_g[2] = (-1 * m[2] * GRAVITY_ACC * param.link_g[2]) * sinf(ornt.current.q[1] + ornt.current.q[2]);

} //CompensateGravity()


//Pressure - Voltage conversion
int DAconversion() {
    int j=0;
	for (int i = 0; i < DA_CHANNEL_NUMBER; i++) {
        if((i!=6) && (i!=7)){
        j = i;
        if(i==0) j = 7;
        if(i==4) j = 6;

        io.output.voltage[j] = k.DA_a[i] * pressure.output[i] * pressure.output[i]
                			   + k.DA_b[i] * pressure.output[i]
                			   + k.DA_c[i];
        }
    }
	return 0;
}


//Voltage - pressure conversion
int ADconversion() {
    int j=0;
	for (int i = 0; i < AD_CHANNEL_NUMBER; i++) {
        if((i!=6) && (i!=7)){
        j = i;
        if (i==0) j = 7;
        if (i==4) j = 6;

		/*pressure.current[i] = k.AD_a[i] * io.input.voltage[i] * io.input.voltage[i]
		                      + k.AD_b[i] * io.input.voltage[i]
			                  + k.AD_c[i];*/
        pressure.current[i] = k.AD_a[j] * io.input.voltage[j] * io.input.voltage[j]
		                      + k.AD_b[j] * io.input.voltage[j]
			                  + k.AD_c[j];
        if(pressure.current[i] < 0){
            pressure.current[i] = 0;
        }
        }
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

float LowPassFilter_angle0(float input){
    float samplerate = (float)SAMPLING_FREQUENCY;  //サンプリング周波数
    float freq = 5;  //カットオフ周波数
    float q = sqrtf(2.0) / 2.0;    //フィルタのQ値
    float omega = 2.0f * 3.14159265f * freq/samplerate;
    float alpha = sin(omega) / (2.0f * q);
 
    float a0 =  1.0f + alpha;
    float a1 = -2.0f * cos(omega);
    float a2 =  1.0f - alpha;
    float b0 = (1.0f - cos(omega)) / 2.0f;
    float b1 =  1.0f - cos(omega);
    float b2 = (1.0f - cos(omega)) / 2.0f;

    float output;
    static float in1, in2, out1, out2; //0:x, 1:y, 2:z
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
    in1 = input;      // 1つ前の入力信号を更新

    out2 = out1;      // 2つ前の出力信号を更新
    out1 = output;    // 1つ前の出力信号を更新

    cycle++;
    return output;
}//LowPassFilter_angle0()

float LowPassFilter_angle1(float input){
    float samplerate = (float)SAMPLING_FREQUENCY;  //サンプリング周波数
    float freq = 5;  //カットオフ周波数
    float q = sqrtf(2.0) / 2.0;    //フィルタのQ値
    float omega = 2.0f * 3.14159265f * freq/samplerate;
    float alpha = sin(omega) / (2.0f * q);
 
    float a0 =  1.0f + alpha;
    float a1 = -2.0f * cos(omega);
    float a2 =  1.0f - alpha;
    float b0 = (1.0f - cos(omega)) / 2.0f;
    float b1 =  1.0f - cos(omega);
    float b2 = (1.0f - cos(omega)) / 2.0f;

    float output;
    static float in1, in2, out1, out2; //0:x, 1:y, 2:z
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
    in1 = input;      // 1つ前の入力信号を更新

    out2 = out1;      // 2つ前の出力信号を更新
    out1 = output;    // 1つ前の出力信号を更新

    cycle++;
    return output;
}//LowPassFilter_angle1()

float LowPassFilter_angle2(float input){
    float samplerate = (float)SAMPLING_FREQUENCY;  //サンプリング周波数
    float freq = 5;  //カットオフ周波数
    float q = sqrtf(2.0) / 2.0;    //フィルタのQ値
    float omega = 2.0f * 3.14159265f * freq/samplerate;
    float alpha = sin(omega) / (2.0f * q);
 
    float a0 =  1.0f + alpha;
    float a1 = -2.0f * cos(omega);
    float a2 =  1.0f - alpha;
    float b0 = (1.0f - cos(omega)) / 2.0f;
    float b1 =  1.0f - cos(omega);
    float b2 = (1.0f - cos(omega)) / 2.0f;

    float output;
    static float in1, in2, out1, out2; //0:x, 1:y, 2:z
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
    in1 = input;      // 1つ前の入力信号を更新

    out2 = out1;      // 2つ前の出力信号を更新
    out1 = output;    // 1つ前の出力信号を更新

    cycle++;
    return output;
}//LowPassFilter_angle2()

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
