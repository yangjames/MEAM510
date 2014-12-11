#pragma once
#include "config.h"

void init_localization_params();
int match_points(uint16_t* constellation,
		  uint16_t ordered_points[][2]);
int match_three(uint16_t* constellation,
		 uint16_t ordered_points[][2]);
int localize(uint16_t ordered_points[][2], 
	      float* center, float* orientation, float* height);
int inverse_kinematics(float* center, float* orientation, float* height,
			float* x, float* y, float* yaw);
