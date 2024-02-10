#include "PID_control.h"
#include <windows.h>

#define FILT_ORDER 8
#define SYS_ORDER  6

double lowpass[FILT_ORDER+1] = {-0.00614041469794508,	-0.0135816744769618, 0.0512322973427179, 0.265655560905451,	
0.405668461853476, 0.265655560905451, 0.0512322973427179, -0.0135816744769618, -0.00614041469794508};

double discrete_time_system[SYS_ORDER+1] = {-0.00153697143536478, 0.0462012754832368, 0.252776229703695,
 0.405118932496866, 0.252776229703695, 0.0462012754832368, -0.00153697143536478};

void add_value(double *arr, int length, double new_val) {
    for(int i = 0; i < length-1; i++) {
        arr[i] = arr[i+1];
    }
    arr[length-1] = new_val;
}

double filter(double *filter, int length, double *input) {
    double output = 0;
    for(int i = 0; i < length; i++) {
        output += filter[i] * input[length-1 - i];
    }

    return output;
}


int main(int argc, char **argv) {
    PID_Controller_t cnt;
    initialize_PID(&cnt, 0.3, 0.7, 0.05, 100., 200., 0.);
    
    double mv[FILT_ORDER+1] = {0,0,0, 0,0,0, 0,0,0};
    double sys_out[SYS_ORDER+1] = {5,5,5, 5,5,5,5};
    double sys_in[SYS_ORDER+1] = {0,0,0, 0,0,0,0};
    
    for(int i = 0; i < 100; i++) {
        double mv_current = step(&cnt, sys_out[FILT_ORDER], 0.1);
        // printf("mv = %lf  ,", mv_current);
        add_value(mv, FILT_ORDER+1, mv_current);
        
        double mv_filtered = filter(lowpass, FILT_ORDER+1, mv);
        // printf("mvfilt = %lf  ,", mv_filtered);
        add_value(sys_in, SYS_ORDER+1, mv_filtered);

        double current_out = filter(discrete_time_system, SYS_ORDER+1, sys_in);
        // printf("cv = %lf, \n", current_out);
        printf("%lf\n", current_out);
        add_value(sys_out, SYS_ORDER+1, current_out);

        // Sleep(500);
    }



}