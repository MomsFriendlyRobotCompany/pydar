#include <Python.h>

double scan_data[2][360];


typedef struct ScanData{
    int len;
    double *data;
} ScanData;

// https://stackoverflow.com/questions/5081875/ctypes-beginner
// cpp?
extern "C" void get_data(ScanData* data){
    int i;
    for (i=0; i<data->len; ++i){
        data->data[0][i] = scan_data[0][i];  // copy angle
        data->data[1][i] = scan_data[1][i];  // copy range
    }
}
