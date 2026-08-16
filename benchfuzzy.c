#include "fuzzy.h"

const kp_output_matrix kp_matrix={
    .kp_output_matrix={
        {4,4,4,3,2},
        {4,3,3,2,1},
        {3,3,2,3,3},
        {1,2,3,3,4},
        {2,3,4,4,4}
    }
};

const kp_output_label kp_label={
    /*
    .kp_output_label={
        -0.0570761f,
        -0.0233455f,
         0.0f,
         0.0233455f,
         0.0570761f
    }
    */
    .kp_output_label={
        -0.035f,
        -0.014f,
         0.0f,
         0.014f,
         0.035f
    }
};

const ki_output_matrix ki_matrix={
    .ki_output_matrix={
        {0,0,1,0,0},
        {1,1,2,1,1},
        {1,3,4,3,1},
        {1,1,2,1,1},
        {0,0,1,0,0}
    }
};

const ki_output_label ki_label={
    /*
    .ki_output_label={
        -0.00285381f,
        -0.00116728f,
         0.0f,
         0.00116728f,
         0.00285381f
    }
    */
    .ki_output_label={
        -0.0048f,
        -0.002f,
         0.0f,
         0.002f,
         0.0048f
    }
};



const kd_output_matrix kd_matrix={
    .kd_output_matrix={
        4,3,2,3,4        
    }
};

const kd_output_label kd_label={
    /*
    .kd_output_label={
        -0.00217804f,
        -0.000933797f,
         0.0f,
         0.000933797f,
         0.00217804f
    }
    */
   .kd_output_label={
        -0.0036f,
        -0.0015f,
         0.0f,
         0.0015f,
         0.0036f
    }
};


//theta label init
const kp_output_label_theta kp_label_theta={
    .kp_output_label_theta={
        -0.030f,
        -0.010f,
         0.0f,
         0.010f,
         0.030f
    }
};

const ki_output_label_theta ki_label_theta={
    .ki_output_label_theta={
        -0.0008f,
        -0.0003f,
         0.0f,
         0.0003f,
         0.0008f
    }
};

const kd_output_label_theta kd_label_theta={
    .kd_output_label_theta={
        -0.0015f,
        -0.0006f,
         0.0f,
         0.0006f,
         0.0015f
    }
};

float trimf_upper(float x, float a, float b, float c)
{
    if (a == b && x == a) return 1.0f;
    if (b == c && x == c) return 1.0f;

    if (x <= a || x >= c)
        return 0.0f;
    else if (x == b)
        return 1.0f;
    else if (x > a && x < b)
        return (x - a) / (b - a);
    else
        return (c - x) / (c - b);
}

float trapmf_upper(float x, float a, float b, float c, float d)
{
    if (x <= a || x >= d) {
        if ((x == a && a == b) || (x == d && c == d))
            return 1.0f;
        return 0.0f;
    }
    else if (x >= b && x <= c) {
        return 1.0f;
    }
    else if (x > a && x < b) {
        return (x - a) / (b - a);
    }
    else {
        return (d - x) / (d - c);
    }
}
//高度邊界設定 ellipse:0.65511728 flower:0.50110239 初始:0.7
float trimf_lower(float x, float a, float b, float c)
{
    if (a == b && x == a) return 0.65511728f;
    if (b == c && x == c) return 0.65511728f;

    if (x <= a || x >= c)
        return 0.0f;
    else if (x == b)
        return 0.65511728f;
    else if (x > a && x < b)
        return 0.65511728f * ((x - a) / (b - a));
    else
        return 0.65511728f * ((c - x) / (c - b));
}

float trapmf_lower(float x, float a, float b, float c, float d)
{
    if (x <= a || x >= d) {
        if ((x == a && a == b) || (x == d && c == d))
            return 0.65511728f;
        return 0.0f;
    }
    else if (x >= b && x <= c) {
        return 0.65511728f;
    }
    else if (x > a && x < b) {
        return 0.65511728f * ((x - a) / (b - a));
    }
    else {
        return 0.65511728f * ((d - x) / (d - c));
    }
}
