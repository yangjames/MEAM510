void match_points(uint16_t* constellation,
		  uint16_t ordered_points[][2]);
void localize(uint16_t ordered_points[][2], 
	      float* center, float* orientation, float* height);
void inverse_kinematics(float* center, float* orientation, float* height,
			float* x, float* y, float* yaw);
