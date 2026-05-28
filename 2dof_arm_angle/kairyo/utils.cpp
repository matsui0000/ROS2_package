#pragma once

#include <stdio.h>
#include <cmath>
#include <termios.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "rclcpp/rclcpp.hpp"
#include "utils.hpp"


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



//クォータニオンを表示する関数(デバック用)
void print_quartanion(const geometry_msgs::msg::TransformStamped::SharedPtr pose){
    double x,y,z,w;
    x = pose->transform.rotation.x;
    y = pose->transform.rotation.y;
    z = pose->transform.rotation.z;
    w = pose->transform.rotation.w;
    printf("x=%lf y=%lf x=%lf w=%lf\n\n",x,y,z,w);
} //print_quartanion()



void DetermineImpactDirection() {}

void DetermineImpactDirection2() {}

void ImpactDrivenControl(ImpactState& state, int pressure_idx_pos, int pressure_idx_neg) {}

void target_inverse() {}