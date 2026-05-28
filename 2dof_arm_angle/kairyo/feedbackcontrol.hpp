#pragma once

double VisualFeedbackControl_angle(double target_q, double current_q, int joint_num);
void VisualFeedbackControl_position();
void PressureFeedbackControl();
void PressureFeedbackReset();