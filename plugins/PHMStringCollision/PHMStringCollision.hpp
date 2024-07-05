// PluginPHMStringCollision.hpp
// rm (spare knobs@site.com)

#pragma once

#include "SC_PlugIn.hpp"

namespace PHMStringCollision {

static const int gnmodesmax=100;
static const float maxfreq = 12000.f;

class PHMStringCollision : public SCUnit {
public:
    PHMStringCollision();

    // Destructor
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
    float _cwin[gnmodesmax];
    float _cwout[gnmodesmax];
    float _y1[gnmodesmax];
    float _y2[gnmodesmax];
    float _cdispl{0.f};
    float _cvel{0.f};

};

} // namespace PHMStringCollision
