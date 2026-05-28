//utils.hpp

//すべてのプログラムで使用する関数と定数を定義するヘッダーファイル
//LowPassFilter はclass.hppに定義しているが，実装はclass.cppに書いている。


#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include "class.hpp"

//関数の宣言
char getch();
int CheckOverPressure_Target(void);
int CheckOverPressure_Output(void);
int DAconversion();
int ADconversion();
void LowPassFilterMarkers();
void mul_R(double left[3][3],double right[3][3],double res[3][3]);
void Trans_R(double R[3][3],double res[3][3]);
void print_matrix(double matrix[3][3]);
void SetPositionTarget(float x, float y, float z);
void ForwardKinematics(float q1, float q2);
void DetermineImpactDirection();
void DetermineImpactDirection2();
void ImpactDrivenControl(ImpactState& state, int pressure_idx_pos, int pressure_idx_neg);
void target_inverse();
void convert_R(const geometry_msgs::msg::TransformStamped& pose, double R[3][3]);
void print_quartanion(const geometry_msgs::msg::TransformStamped& pose);


//定数の宣言
#define DA_CHANNEL_NUMBER 12
#define AD_CHANNEL_NUMBER 8     //=number of pressure sensor

#define DA_Conversion_K 0.1     //Voltage = K * Pressure
#define AD_Conversion_K 25.2    //Pressure = K * Voltage + A
#define AD_Conversion_A -25.4
#define SAMPLING_FREQUENCY 200
#define GRAVITY_ACC 9.80665

#define MINIMUM_PRESSURE 0.0F   //アクチュエータの最小圧力[kPa]
#define MAXIMUM_PRESSURE 60.0F  //アクチュエータの最大圧力[kPa]

#define BASE_PRESSURE 30.0F     //基本圧力[kPa]
#define LINK_PRESSURE 4.0F      //リンク膨張用圧力

#define GRAPPING_WEIGHT 0.0  //手先に吊り下げる荷重[kg]

extern double basePressure;
extern double linkPressure;
extern double targetJointStiffness;

//ROS time
extern unsigned int cycle;

//sensor
extern double voltageInput[AD_CHANNEL_NUMBER];
extern double voltageOutput[DA_CHANNEL_NUMBER];

