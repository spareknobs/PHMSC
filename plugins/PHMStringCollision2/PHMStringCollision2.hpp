// PluginPHMStringCollision2.hpp
// String-Bow model adapted from Demoucron's violin model 
// (....)

#pragma once

#include "SC_PlugIn.hpp"
#include "SC_Unit.h"

namespace PHMStringCollision2 {

static const int cmaxnmodes = 200;

class PHMStringCollision2 : public SCUnit {

public:
    PHMStringCollision2();

private:
    void next(int nSamples);
    void reset();
    void UpdateFilterCoeffs();
    
private:
    // Parameters
    int _nmodes_req{0};
    float _f0{55.f};         // string fund freq
    float _L {1.1f};         // scale (m)
    float _dispersion{0};
    //float _dia {2.8e-3f};    // diameter
    //float _E {4e9f};         // Young
    float _rho {0.00234f};   // linear density
    float _d1 {3.1f};        // damps
    float _d2 {0.1f};
    float _posin{0.22f};    // pos (norm to scale)
    float _finger_pos{0.7f};     // pos (norm to scale)
    float _K {0.f};            // finger stiff

private:
    int _nmodes{0};
    
    alignas(16) float _X1[cmaxnmodes];
    alignas(16) float _X2[cmaxnmodes];
    alignas(16) float _X3[cmaxnmodes];
    alignas(16) float _Y1[cmaxnmodes];
    alignas(16) float _Y2[cmaxnmodes];
    alignas(16) float _Y3[cmaxnmodes];
    alignas(16) float _Phi0[cmaxnmodes];
    alignas(16) float _Phi1[cmaxnmodes];
    alignas(16) float _Alpha0[cmaxnmodes];
    alignas(16) float _Alpha1[cmaxnmodes];
    alignas(16) float _Beta0[cmaxnmodes];
    alignas(16) float _Beta1[cmaxnmodes];
    alignas(16) float _Kbridge[cmaxnmodes];
    alignas(16) float _z[cmaxnmodes];
    alignas(16) float _zdot[cmaxnmodes];
    alignas(16) float _z_h[cmaxnmodes];
    alignas(16) float _zdot_h[cmaxnmodes];
    
    float _A01{0.f};
    float _A11{0.f};
    float _B00{0.f};
    float _B01{0.f};
    float _C01{0.f};
    float _C02{0.f};
    float _C11{0.f};
    float _C12{0.f};
};

} // namespace PHMStringCollision2 