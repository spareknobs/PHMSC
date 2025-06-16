// PluginPHMStringCollision.hpp
// rm (spare knobs@site.com)

#pragma once

#include "SC_PlugIn.hpp"

namespace PHMStringCollision {

static const int gnmodesmax=200;
static const int gncollidersmax=20;

class PHMStringCollision : public SCUnit {
public:
    PHMStringCollision();
    // ~PHMStringCollision();

private:
    // Calc function
    void next(int nSamples);

    // Member variables
    float _b1[gnmodesmax];
    float _a1[gnmodesmax];
    float _a2[gnmodesmax];
    float _win[gnmodesmax];
    float _wout[gnmodesmax];
    float _cwin[gncollidersmax][gnmodesmax];
    float _cwout[gncollidersmax][gnmodesmax];
    float _y1[gnmodesmax];
    float _y2[gnmodesmax];
    int   _ncoll{0}; // number of colliders
    float _cpos[gncollidersmax];   
    float _cdispl[gncollidersmax]; // string displ @ collision point
    float _mdispl[gncollidersmax]; // mass displ @ collision point
    float _mvel[gncollidersmax]; // mass vel @ collision point
    float _mass[gncollidersmax]; // if not rigid, colliders are point-like inertial masses connected to spring and damp
    float _mstiff[gncollidersmax];
    float _mdamp[gncollidersmax];
    float _cpmax{-1};
    float _mvar{0.f};
    RGen& _rgen;
};

} // namespace PHMStringCollision
