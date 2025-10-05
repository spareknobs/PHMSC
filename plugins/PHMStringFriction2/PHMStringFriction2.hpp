// PluginPHMStringFriction2.hpp
// String-Bow model adapted from:
//  Matthias Demoucron, "On the control of virtual violins" (2008)
// https://theses.hal.science/tel-00349920/document

#pragma once

#include "SC_PlugIn.hpp"
#include "SC_Unit.h"

namespace PHMStringFriction2 {

static const int cmaxnmodes = 200;
static constexpr float cBowForceThres = 0.002f;

class PHMStringFriction2 : public SCUnit {

public:
    PHMStringFriction2();

private:
    void next(int nSamples);
    void reset();
    void UpdateFilterCoeffs();
    void UpdateFingerCoeffs();
    void UpdateBowForce();
    
private:
    // Parameters
    int _nmodes_req{0};
    float _f0{55.f};         // string fund freq
    float _L {1.1f};         // scale (m)
    float _dia {2.8e-3f};    // diameter
    float _E {4e9f};         // Young
    float _rho {0.00234f};   // linear density
    float _d1 {3.1f};        // damps
    float _d2 {0.1f};
    float _bow_pos{0.22f};    // bow pos (norm to scale)
    float _finger_pos{0.7f};     // finger pos (norm to scale)
    float _V0 {0.01f};       // sort-of-stribeck vel - must be > 0
    float _Mu_s  {1.f};      // static friction coeff
    float _Mu_d  {0.5f};     // dynamic friction coeff
    float _K {0.f};            // finger stiff

private:
    int _nmodes{0};
    
    // friction state
    enum state {
        stick=0,
        slip
    };

    state mstate{stick};
    
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

} // namespace PHMStringFriction2
