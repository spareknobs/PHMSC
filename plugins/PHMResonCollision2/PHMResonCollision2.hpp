// PluginPHMResonCollision.hpp
// rm (spare knobs@site.com)

#pragma once

#include "SC_PlugIn.hpp"

namespace PHMResonCollision2 {

static const int gnmodesmax = 600;

class PHMResonCollision2 : public SCUnit {
public:
    PHMResonCollision2();

private:
    // Calc function
    void next(int nSamples);

    void computeRandomModes( const float fmin,
                            const float fmax,
                            const float d1,
                            const float d2,
                            const float posin,
                            const float cposin,
                            const int nmodes,
                            const float detune,
                            const float spread ); 

    // Member variables
    float _b1[gnmodesmax];
    float _a1[gnmodesmax];
    float _a2[gnmodesmax];
    float _win[gnmodesmax];
    float _cwin[gnmodesmax];
    float _y1[gnmodesmax];
    float _y2[gnmodesmax];
    float _cdispl{0.f};
    float _cvel{0.f};
    float _fc[gnmodesmax];
    float _fmin{0};
    float _fmax{0};
    float _spread{0};
    int _nmodes{0};
    
};

} // namespace PHMResonCollision2
