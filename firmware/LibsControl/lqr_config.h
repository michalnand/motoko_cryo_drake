#ifndef _LQR_CONFIG_H_
#define _LQR_CONFIG_H_

// LQR controller config
#define LQR_SYSTEM_ORDER  ((uint32_t)4)
#define LQR_SYSTEM_INPUTS ((uint32_t)2)



const float lqr_k[8] = {
    5.935804f, 0.081035f, 0.000000f, 0.000000f,
    0.000000f, 0.000000f, 0.061063f, 0.001307f
};

const float lqr_ku[4] = {
    0.647662f, 0.000000f,
    0.000000f, 0.627133f
};

    
#endif // _LQR_CONFIG_H_