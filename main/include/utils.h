#ifndef _UTILS_H_
#define _UTILS_H_


typedef struct _faceroi_
{
    int conf;
    int x;
    int y;
    int w;
    int h;
    int eye_r_x;
    int eye_r_y;
    int eye_l_x;
    int eye_l_y;
    int nose_x;
    int nose_y;
    int mouth_r_x;
    int mouth_r_y;
    int mouth_l_x;
    int mouth_l_y;
} faceroi_t;


#endif