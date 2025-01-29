// PluginPHMFlue1.hpp
// A simple flue model consisting of a nonlinear excitation mechanism and a modal resonator.
// Adapted from SDK::Blow by Perry R. Cook and Gary P. Scavone, 1995--2023. 
// riccardo marogna 2024


#pragma once

#include "SC_PlugIn.hpp"
#include "SC_Unit.h"

namespace PHMFlue1 {

static const int gnmodesmax=200;

class PHMFlue1 : public SCUnit {
public:
    PHMFlue1();
    // ~PHMFlue1();

private:
void computeModes( int& nmodes, 
                    const float f0,
                    const float disprs,
                    const float d1, 
                    const float d2,
                    const int nmodes_req);

    // Calc function
    void next(int nSamples);

    // Member variables
    float _b1[gnmodesmax];
    float _a1[gnmodesmax];
    float _a2[gnmodesmax];
    float _y1[gnmodesmax];
    float _y2[gnmodesmax];
    float res_out_prev{0.f};

    //BrownNoise mnoise;
    
};

} // namespace PHMFlue1
