// PluginPHMResonCollision.hpp
// rm (spare knobs@site.com)

#pragma once

#include "SC_PlugIn.hpp"

namespace PHMResonCollision {

static const int gnmodesmax=100;
static const float maxfreq = 12000.f;

class PHMResonCollision : public SCUnit {
public:
    PHMResonCollision();

    // Destructor
    // ~PHMResonCollision();

private:
    // Calc function
    void next(int nSamples);

    void computeRandomModes( float* pA1, 
                            float* pA2, 
                            float* pB1, 
                            const float fmin,
                            const float fmax,
                            const float detune,
                            const float d1,
                            const float d2,
                            const int nmodes); 


    // Member variables
    float _b1[gnmodesmax];
    float _a1[gnmodesmax];
    float _a2[gnmodesmax];
    float _win[gnmodesmax];
    float _wout[gnmodesmax];
    float _cwin[gnmodesmax];
    float _cwout[gnmodesmax];
    float _y1[gnmodesmax];
    float _y2[gnmodesmax];
    float _cdispl{0.f};
    float _cvel{0.f};
    float _fc[gnmodesmax];
    float _fmin{50};
    float _fmax{5000};

};

} // namespace PHMResonCollision
