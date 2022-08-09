#include "cordiccart2pol.h"
#include "xmath.h"

void cordiccart2pol(data_t *x, data_t *y, data_t *r,  data_t *theta)
{
	// Write your code here
#pragma HLS INTERFACE m_axi port=x offset=slave depth=1
#pragma HLS INTERFACE m_axi port=y offset=slave depth=1
#pragma HLS INTERFACE m_axi port=r offset=slave depth=1
#pragma HLS INTERFACE m_axi port=theta offset=slave depth=1
#pragma HLS INTERFACE s_axilite port=return bundle = CTRL
	//The cordic_phase array holds the angle for the current rotation
	data_t cordic_phase[NO_ITER] = {
    	45, 26.56, 14.036, 7.125,
    	3.576, 1.790, 0.895, 0.448,
		0.224, 0.112, 0.056, 0.028,
		0.014, 0.007, 0.003, 0.002
	};
#pragma HLS ARRAY_PARTITION variable=cordic_phase complete dim=0

    //Set the initial vector that we will rotate
    data_t current_cos;
    data_t current_sin;
    data_t theta0;
    if((*x)>=0)
	{
//    	current_cos = sqrt(*x*x + y*y) * 0.607252935;
    	current_cos = sqrt((*x)*(*x) + (*y)*(*y)) * 0.607252935;
    	current_sin = 0 * 0.607252935;
   		theta0 = 0;
   		//flag = 1;
	}
    else
	{
//    	current_cos = -sqrt(x*x + y*y) * 0.607252935;
    	current_cos = -sqrt((*x)*(*x) + (*y)*(*y)) * 0.607252935;
    	current_sin = 0 * 0.607252935;
    	theta0 = 180;
    	//flag = 2;
	}

    //Factor is the 2^(-L) value
    data_t factor = 1.0;

    //This loop iteratively rotates the initial vector to find the
    //sine and cosine value corresponding to the input theta angle
#pragma HLS PIPELINE II=4
    for(int j = 0; j < NO_ITER; j++){
    	//Determine if we are rotating by a positive or negative angle
    	int sigma = (current_sin/current_cos - (*y)/(*x) > 0) ? -1 : 1;

    	//Save the current_cos,so that it can be used in the sine calculation
    	data_t temp_cos = current_cos;

    	//Perform the rotation: x = xi-1 - sigma*yi-1*factor  y = xi-1*sigma*factor + yi-1
    	current_cos = current_cos - current_sin * sigma * factor;
    	current_sin = temp_cos * sigma * factor + current_sin;

    	//Determine the new theta
    	theta0 = theta0 + sigma * cordic_phase[j];

    	//Calculata next 2^(-L) value
    	factor = factor / 2;
    	//flag++;
    }

    //Set the final values
    *theta = ((theta0 + (((*x) >= 0 || (*y) >= 0) ? 0 : -360)) / (float)180.0 ) * PI;
    *r = sqrt(current_cos*current_cos + current_sin*current_sin);
}
