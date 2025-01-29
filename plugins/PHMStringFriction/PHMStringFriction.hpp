// PluginPHMStringFriction.hpp
// rm (spare knobs@site.com)

#pragma once

#include "SC_PlugIn.hpp"

namespace PHMStringFriction {

static const int gnmodesmax=200;

class PHMStringFriction : public SCUnit {
public:
    PHMStringFriction();

    // Destructor
    // ~PHMStringFriction();

private:
    // Calc function
    void next(int nSamples);

    // Member variables
    float _b1[gnmodesmax];
    float _a1[gnmodesmax];
    float _a2[gnmodesmax];
    float _wout[gnmodesmax];
    float _cwin[gnmodesmax];
    float _cwout[gnmodesmax];
    float _y1[gnmodesmax];
    float _y2[gnmodesmax];
    float _cdispl{0.f};
    float _cvel{0.f};
};

} // namespace PHMStringFriction
