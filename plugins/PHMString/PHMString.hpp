// PluginPHMString.hpp
// rm (spare knobs@site.com)

#pragma once

#include "SC_PlugIn.hpp"

namespace PHMString {

static const int gnmodesmax=200;

class PHMString : public SCUnit {
public:
    PHMString();

    // Destructor
    // ~PHMString();

private:
    // Calc function
    void next(int nSamples);

    // Member variables
    float _b1[gnmodesmax];
    float _a1[gnmodesmax];
    float _a2[gnmodesmax];
    float _wout[gnmodesmax];
    float _y1[gnmodesmax];
    float _y2[gnmodesmax];
};

} // namespace PHMString
