// String-Bow model adapted from Demoucron's violin model 
// (....)

#pragma once

#include "SC_PlugIn.hpp"
#include "SC_Unit.h"

namespace PHMResonFriction {

static const int cmaxnmodes = 500;
static constexpr float cBowForceThres = 0.0001f;

class PHMResonFriction : public SCUnit {

public:
    PHMResonFriction();

private:
    void next(int nSamples);
    void reset();
    void UpdateFilterCoeffs(const float fmin,const float fmax);
    void UpdateFingerCoeffs();
    void UpdateBowForce();

private:
    // Parameters
    int _nmodes_req{0};
    float _fmin{-1};         // 
    float _fmax{-1};         // 
    float _detune{0.f};         // 
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
    float _E {4e9f};         // Young
    float _rho {0.00234f};   // linear density
    
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
    float _fc[cmaxnmodes];
    
    float _A01{0.f};
    float _A11{0.f};
    float _B00{0.f};
    float _B01{0.f};
    float _C01{0.f};
    float _C02{0.f};
    float _C11{0.f};
    float _C12{0.f};
};

} // namespace PHMResonFriction
