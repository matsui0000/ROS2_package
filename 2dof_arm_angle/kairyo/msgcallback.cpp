#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "inflatable/msg/voltage_input.hpp"

#include "utils.hpp"
#include "dof2_arm_angle_control.hpp"


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

