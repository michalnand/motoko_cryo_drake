#ifndef _LQR_CONFIG_H_
#define _LQR_CONFIG_H_

// LQR controller config
#define LQR_SYSTEM_ORDER  ((uint32_t)4)
#define LQR_SYSTEM_INPUTS ((uint32_t)2)


const float lqr_k[8] = {
    0.215872f, 0.004880f, -0.000000f, -0.000000f,
    -0.000000f, -0.000000f, 0.364956f, 0.006729f
};

const float lqr_ku[4] = {
    0.070380f, -0.000000f,
    -0.000000f, 0.403740f
};



#endif // _LQR_CONFIG_H_