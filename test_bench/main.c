#include <stdint.h>
#include <stdio.h>
#include <math.h>
#define PI 3.1415926535

void init_localization_params();
void match_points(uint16_t* constellation, uint16_t ordered_points[][2]);

void localize(uint16_t ordered_points[][2],
	      float* center, float* orientation, float* height);

void inverse_kinematics(float* center, float* orientation, float* height, 
			float* x, float* y, float* yaw);

void match_three(uint16_t* constellation, uint16_t ordered_points[][2]);

void find_triangle(uint16_t* star1, uint16_t* star2, uint16_t* star3, 
		   uint16_t* const1, uint16_t* const2, uint16_t* const3, uint16_t* const4, 
		   float* error);


float p_gnd[4][2] = {{0, 14.5},
		     {-11.655, 8.741},
		     {10.563, 2.483},
		     {0, -14.5}};
float distances[6];
float triangles[4][3];

int main(void) {
  int i,j;
  init_localization_params();
  uint16_t ordered_points[4][2];
  uint16_t constellation[12] = {636,576,3,576,622,6,600,652,9,641,633,12};
  float center[2], orientation, height, x, y, yaw;

  match_points(constellation, ordered_points);
  localize(ordered_points, center, &orientation, &height);
  inverse_kinematics(center, &orientation, &height, &x, &y, &yaw);

  for (i = 0; i < 4; i++) {
    for (j = 0; j < 2; j++)
      printf("%d ", ordered_points[i][j]);
    printf("\n");
  }
  printf("\n");
  printf("%5.6f %5.6f %5.6f\n", center[0], center[1], orientation*180/PI);
  printf("\n");
  printf("%5.6f %5.6f %5.6f\n", x, y, yaw*180/PI);
  return 0;
}

void init_localization_params() {
  /* assign distances from ground data */
  int i, j, k, iterator = 0;
  for (i = 0; i < 4; i++) {
    for (j = i+1; j < 4; j++) {
      distances[iterator++] = (p_gnd[i][0]-p_gnd[j][0])*(p_gnd[i][0]-p_gnd[j][0])+(p_gnd[i][1]-p_gnd[j][1])*(p_gnd[i][1]-p_gnd[j][1]);
    }
  }

  /* assign triangles */
  triangles[0][0] = distances[0]/distances[3];
  triangles[0][1] = distances[1]/distances[3];
  triangles[0][2] = distances[0]/distances[1];
  triangles[1][0] = distances[1]/distances[2];
  triangles[1][1] = distances[5]/distances[2];
  triangles[1][2] = distances[1]/distances[5];
  triangles[2][0] = distances[0]/distances[2];
  triangles[2][1] = distances[4]/distances[2];
  triangles[2][2] = distances[0]/distances[4];
  triangles[3][0] = distances[5]/distances[4];
  triangles[3][1] = distances[3]/distances[4];
  triangles[3][2] = distances[5]/distances[3];
}

void match_points(uint16_t* constellation, uint16_t ordered_points[][2]) {
  int i, j;
  /* calculate average */
  float avg[2] = {(constellation[0] + constellation[3] 
		   + constellation[6] + constellation[9])/4.0,
		  (constellation[1] + constellation[4] 
		   + constellation[7] + constellation[10])/4.0};

  /* calculate sum of square differences */
  float sq_differences[4][2] = {{constellation[0]-avg[0],constellation[1]-avg[1]},
			     {constellation[3]-avg[0],constellation[4]-avg[1]},
			     {constellation[6]-avg[0],constellation[7]-avg[1]},
			     {constellation[9]-avg[0],constellation[10]-avg[1]}};

  for (i = 0; i < 4; i++)
    for (j = 0; j < 2; j++)
      sq_differences[i][j]*=sq_differences[i][j];

  /* calculate L2 norm */
  float abs_differences[4] = {sq_differences[0][0]+sq_differences[0][1],
			      sq_differences[1][0]+sq_differences[1][1],
			      sq_differences[2][0]+sq_differences[2][1],
			      sq_differences[3][0]+sq_differences[3][1]};

  /* get difference of max and min values in abs_differences */
  float min = abs_differences[0], max = abs_differences[0];
  for (i = 1; i < 4; i++) {
    if (abs_differences[i] < min) min = abs_differences[i];
    if (abs_differences[i] > max) max = abs_differences[i];
  }

  float range = max-min;
  if (range < 3000) {
    float sort_mat[4][4] = {{abs_differences[0], 1, constellation[0], constellation[1]},
			    {abs_differences[1], 2, constellation[3], constellation[4]},
			    {abs_differences[2], 3, constellation[6], constellation[7]},
			    {abs_differences[3], 4, constellation[9], constellation[10]}};

    /* sort the matrix */
    float buf[4];
    float val_s;
    int idx_s;
    for (i = 0; i < 4; i++) {
      idx_s = i;
      val_s = sort_mat[i][0];
      for (j = i+1; j < 4; j++) {
	if (sort_mat[j][0] < val_s) {
	  idx_s = j;
	  val_s = sort_mat[j][0];
	}
      }
      if (val_s != sort_mat[i][0]) {
	for (j = 0; j < 4; j++) {
	  buf[j] = sort_mat[idx_s][j];
	  sort_mat[idx_s][j] = sort_mat[i][j];
	  sort_mat[i][j] = buf[j];
	}
      }
    }

    ordered_points[0][0] = (int)sort_mat[1][2];
    ordered_points[0][1] = (int)sort_mat[1][3];
    ordered_points[1][0] = (int)sort_mat[2][2];
    ordered_points[1][1] = (int)sort_mat[2][3];
    ordered_points[2][0] = (int)sort_mat[0][2];
    ordered_points[2][1] = (int)sort_mat[0][3];
    ordered_points[3][0] = (int)sort_mat[3][2];
    ordered_points[3][1] = (int)sort_mat[3][3];
    
  }
  else {
    //match_three(constellation, ordered_points);
    ordered_points[0][0] = 1023;
    ordered_points[0][1] = 1023;
    ordered_points[1][0] = 1023;
    ordered_points[1][1] = 1023;
    ordered_points[2][0] = 1023;
    ordered_points[2][1] = 1023;
    ordered_points[3][0] = 1023;
    ordered_points[3][1] = 1023;
  }
}

void match_three(uint16_t* constellation, uint16_t ordered_points[][2]) {
  uint16_t constellation_set[4][8];
  float error[4];
  find_triangle(&ordered_points[0][0], &ordered_points[1][0], &ordered_points[2][0],
		&constellation_set[0][0], &constellation_set[0][2], &constellation_set[0][4], &constellation_set[0][6],
		&error[0]);
  find_triangle(&ordered_points[0][0], &ordered_points[1][0], &ordered_points[3][0],
		&constellation_set[1][0], &constellation_set[1][2], &constellation_set[1][4], &constellation_set[1][6],
		&error[1]);
  find_triangle(&ordered_points[0][0], &ordered_points[2][0], &ordered_points[3][0],
		&constellation_set[2][0], &constellation_set[2][2], &constellation_set[2][4], &constellation_set[2][6],
		&error[2]);
  find_triangle(&ordered_points[1][0], &ordered_points[2][0], &ordered_points[3][0],
		&constellation_set[3][0], &constellation_set[3][2], &constellation_set[3][4], &constellation_set[3][6], &error[3]);
  
}

void find_triangle(uint16_t* star1, uint16_t* star2, uint16_t* star3,
		   uint16_t* const1, uint16_t* const2, uint16_t* const3, uint16_t* const4,
		   float* error) {
  float sides[3] = {(float)(star1[0]-star2[0])*(star1[0]-star2[0]) + (star1[1]-star2[1])*(star1[1]-star2[1]),
		    (float)(star1[0]-star3[0])*(star1[0]-star3[0]) + (star1[1]-star3[1])*(star1[1]-star3[1]),
		    (float)(star2[0]-star3[0])*(star2[0]-star3[0]) + (star2[1]-star3[1])*(star2[1]-star3[1])};
  
  float m[3][3] = {{1,2,sides[0]},
		   {1,3,sides[1]},
		   {2,3,sides[2]}};
  float buf[3];
  int i, j, idx_s;
  float val_s;
  for (i = 0; i < 3; i++) {
    idx_s = i;
    val_s = m[i][2];
    for (j = i+1; j < 3; j++) {
      if (m[j][2] < val_s) {
	idx_s = j;
	val_s = m[j][2];
      }
    }
    if (val_s != m[i][2]) {
      for (j = 0; j < 3; j++) {
	buf[j] = m[idx_s][j];
	m[idx_s][j] = m[i][j];
	m[i][j] = buf[j];
      }
    }
  }

  float mT[3] = {m[0][2]/m[2][2],
		 m[1][2]/m[2][2],
		 m[0][2]/m[1][2]};
  float mT_diff[4] = {abs(mT[0]-triangles[0][0])+abs(mT[1]-triangles[0][1])+abs(mT[2]-triangles[0][2]),
		      abs(mT[0]-triangles[1][0])+abs(mT[1]-triangles[1][1])+abs(mT[2]-triangles[1][2]),
		      abs(mT[0]-triangles[2][0])+abs(mT[1]-triangles[2][1])+abs(mT[2]-triangles[2][2]),
		      abs(mT[0]-triangles[3][0])+abs(mT[1]-triangles[3][1])+abs(mT[2]-triangles[3][2])};

  float likelihood_table[4][2] = {{123,mT_diff[0]},
				  {134,mT_diff[1]},
				  {124,mT_diff[2]},
				  {234,mT_diff[3]}};

  val_s = likelihood_table[0][1];
  for (i = 1; i < 4; i++) {
    if (likelihood_table[i][1] < val_s) {
      idx_s = i;
      val_s = likelihood_table[i][1];
    }
  }
}

void localize(uint16_t ordered_points[][2],
	      float* center, float* orientation, float* height) {
  if (ordered_points[0][0] != 1023 && ordered_points[0][1] != 1023 
      && ordered_points[3][0] != 1023 && ordered_points[3][1] != 1023) {
    center[0] = (ordered_points[0][0]+ordered_points[3][0])/2.0-512.0;

    center[1] = (ordered_points[0][1]+ordered_points[3][1])/2.0-384.0;
    *orientation = -atan2(((float)ordered_points[3][1]-ordered_points[0][1]),
			 ((float)ordered_points[3][0]-ordered_points[0][0]))+PI/2;
    *height = 0;
  }
  else {
    center[0] = 1023;
    center[1] = 1023;
    *orientation = 0;
    *height = 0;
  }
}

void inverse_kinematics(float* center, float* orientation, float* height,
			float* x, float* y, float* yaw) {
  float x_const = 750*center[0]*32/1024*PI/180, y_const = 750*center[1]*32/1024*PI/180;
  *yaw = -(*orientation);
  *x = -cos(*yaw)*x_const - sin(*yaw)*y_const;
  *y = sin(*yaw)*x_const - cos(*yaw)*y_const;
}
