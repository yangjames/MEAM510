const float p_gnd[4][2] = {{0, 14.5},
		     {-11.655, 8.741},
		     {10.563, 2.483},
		     {0, -14.5}};
float distances[6];
float triangles[4][3];

void init_localization_params();
void match_points(uint16_t* constellation,
		  uint16_t ordered_points[][2]);
void match_three(uint16_t* constellation,
		 uint16_t ordered_points[][2]);
void localize(uint16_t ordered_points[][2], 
	      float* center, float* orientation, float* height);
void inverse_kinematics(float* center, float* orientation, float* height,
			float* x, float* y, float* yaw);
