#ifndef _LQR_CONFIG_H_
#define _LQR_CONFIG_H_

// LQR controller config
#define LQR_SYSTEM_ORDER  ((uint32_t)4)
#define LQR_SYSTEM_INPUTS ((uint32_t)2)


/*
const float lqr_k[8] = {
    0.302359f, 0.006674f, -0.000000f, -0.000000f,
    0.000000f, 0.000000f, 0.089502f, 0.002031f
};

const float lqr_ku[4] = {
    0.089172f, 0.000000f,
    0.000000f, 0.221270f
};
*/
 

const float lqr_k[8] = {
    0.994501f, 0.022365f, -0.000000f, -0.000000f,
    -0.000000f, -0.000000f, 0.917121f, 0.015678f
};

#endif // _LQR_CONFIG_H_