#pragma once

struct CarParams{
    double wheelbase;
    double friction_coeff;
    double cg_height;       // height of center of gravity
    double l_f;             // length from CG to front axle
    double l_r;             // length from CG to rear axle
    double cs_f;            // cornering stiffness coefficient for front wheels
    double cs_r;            // cornering stiffness coefficient for rear wheels
    double mass;
    double I_z;             // moment of inertia aboue z axis from CG
};
