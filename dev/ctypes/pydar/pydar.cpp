// #include <Python.h>  // don't need for ctypes
#include <iostream>
double scan_data[2*360] = {0.0};


typedef struct ScanData{
    int len;
    double *data;
} ScanData;

// https://stackoverflow.com/questions/5081875/ctypes-beginner
// cpp?
extern "C" void get_data(ScanData* data){
    int i;
    // printf(data->len);
    std::cout << data->len << std::endl;
    for (i=0; i<data->len; i+=2){
        data->data[i] = scan_data[i]+i%360;  // copy angle
        data->data[i+1] = scan_data[i+1]+i*3.14;  // copy range

        // std::cout << data->data[i] << ' ' << data->data[i+1] << std::endl;
    }
}
