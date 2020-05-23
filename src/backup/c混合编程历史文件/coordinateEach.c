# include <stdio.h>
# include <math.h>

void getNewPoint(float* thetaAtrans, float *latDeviation, int arr_num, float (*out_arr)[2]){

    for(int i=0;i<arr_num;i++){
        *(*(out_arr+i)+0) = -sin(thetaAtrans[i]) * latDeviation[i];
        *(*(out_arr+i)+1) = cos(thetaAtrans[i]) * latDeviation[i];
    }
}
