#ifndef _LQR_CONFIG_H_
#define _LQR_CONFIG_H_

// LQR controller config
#define LQR_SYSTEM_ORDER  ((uint32_t)4)
#define LQR_SYSTEM_INPUTS ((uint32_t)2)

const float lqr_k[8] = {
    0.664898f, 0.014631f, -0.000000f, -0.000000f,
    0.000000f, 0.000000f, 0.552535f, 0.009712f
};

const float lqr_ku[4] = {
    0.122962f, 0.000000f,
    0.000000f, 0.488654f
};

#endif // _LQR_CONFIG_H_