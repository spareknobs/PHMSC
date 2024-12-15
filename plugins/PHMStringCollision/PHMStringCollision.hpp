// PluginPHMStringCollision.hpp
// rm (spare knobs@site.com)

#pragma once

#include "SC_PlugIn.hpp"

namespace PHMStringCollision {

static const int gnmodesmax=200;

class PHMStringCollision : public SCUnit {
public:
    PHMStringCollision();

    // Destructor
    // ~PHMStringCollision();

private:
void computeStringModes( int& nmodes, 
                                const float L,
                                const float f0,
                                const float disprs,
                                const float sigma, 
                                const float d1, 
                                const float d2,
                                const int nmodes_req);

    // Calc function
    void next(int nSamples);

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

};

} // namespace PHMStringCollision
