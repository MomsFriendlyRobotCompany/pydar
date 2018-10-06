#include <Python.h>
#include <iostream>
double scan_data[2*360];


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
        data->data[i] = scan_data[i];  // copy angle
        data->data[i+1] = scan_data[i+1];  // copy range
    }
}
