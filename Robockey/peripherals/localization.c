#include "m_includes.h"
#include "localization.h"

void match_points(uint16_t* constellation, uint16_t ordered_points[][2]) {
  int i, j;
  /* calculate average */
  float avg[2] = {(constellation[0] + constellation[3] + constellation[6] + constellation[9])/4.0,
		  (constellation[1] + constellation[4] + constellation[7] + constellation[10])/4.0};

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

    /* assign ordered points */
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
    //TODO: 3 point detection
    ordered_points[0][0] = 0;
    ordered_points[0][1] = 0;
    ordered_points[1][0] = 0;
    ordered_points[1][1] = 0;
    ordered_points[2][0] = 0;
    ordered_points[2][1] = 0;
    ordered_points[3][0] = 0;
    ordered_points[3][1] = 0;
  }
}

void localize(uint16_t ordered_points[][2],
	      float* center, float* orientation, float* height) {

}
