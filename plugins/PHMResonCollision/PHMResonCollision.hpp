// PluginPHMResonCollision.hpp
// rm (spare knobs@site.com)

#pragma once

#include "SC_PlugIn.hpp"
#define VERSION 0.1
#define OPTIMIZE_LOOP 0  // Set to 1 to enable loop optimization, 0 to disable

namespace PHMResonCollision {

static const int gnmodesmax = 600;

class PHMResonCollision : public SCUnit {
public:
    PHMResonCollision();

    // Destructor
    // ~PHMResonCollision();

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
                            const float detune); 

    // Member variables
    alignas(16) float _fc[gnmodesmax];
    alignas(16) float _b1[gnmodesmax];
    alignas(16) float _a1[gnmodesmax];
    alignas(16) float _a2[gnmodesmax];
    alignas(16) float _win[gnmodesmax];
    alignas(16) float _cwin[gnmodesmax];
    alignas(16) float _y1[gnmodesmax];
    alignas(16) float _y2[gnmodesmax];
    float _cdispl{0.f};
    float _cvel{0.f};
    float _fmin{50};
    float _fmax{5000};
    int _nmodes{0};
};

} // namespace PHMResonCollision
