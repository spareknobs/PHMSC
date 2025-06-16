#include "PHMStringCollision2.hpp"
#include "../utils.hpp"

static InterfaceTable* ft;

namespace PHMStringCollision2 {


PHMStringCollision2::PHMStringCollision2() {
    std::memset(_X2,0.f,sizeof(_X2));
    std::memset(_X3,0.f,sizeof(_X3));
    std::memset(_Y1,0.f,sizeof(_Y1));
    std::memset(_Y2,0.f,sizeof(_Y2));
    std::memset(_Y3,0.f,sizeof(_Y3));
    std::memset(_Phi0,0.f,sizeof(_Phi0));
    std::memset(_Phi1,0.f,sizeof(_Phi1));
    std::memset(_Alpha0,0.f,sizeof(_Alpha0));
    std::memset(_Alpha1,0.f,sizeof(_Alpha1));
    std::memset(_Beta0,0.f,sizeof(_Beta0));
    std::memset(_Beta1,0.f,sizeof(_Beta1));
    std::memset(_Kbridge,0.f,sizeof(_Kbridge));
    std::memset(_z,0.f,sizeof(_z));
    std::memset(_zdot,0.f,sizeof(_zdot));
    std::memset(_z_h,0.f,sizeof(_z_h));
    std::memset(_zdot_h,0.f,sizeof(_zdot_h));
    mCalcFunc = make_calc_function<PHMStringCollision2, &PHMStringCollision2::next>();
    next(1);
}

void PHMStringCollision2::next(int nSamples) {
    
    const float* vpin = in(0);
    float* vpout = out(0);
    float sr = sampleRate();
    float dt = 1.f / sr;
    float nyq = sampleRate() * 0.5f;
    float flimit = sr * 0.25f;
    
    // get control rate parameters
    // f0=55.0, L=1.0, d1=1.0, d2=0.001, posin=0.33, fingerpos=0.7, fingerK=0, gain=1.0, nmodes=80
    _f0 = sc_clip( in0(1), 20.f, nyq );
    _L = sc_clip(in0(2), 0.01, 10.0);
    _rho = sc_clip(in0(3), 0.001, 10.0);
    _dispersion = sc_clip(in0(4), 0.0, 30.0);
    _d1 = sc_clip( in0(5), 0.0001f, 100.f);
    _d2 = sc_clip( in0(6), 0.f, 1.f );
    _posin = sc_clip(in0(7), 0.f, 1.f );
    _finger_pos = sc_clip(in0(8), 0.f, 1.f);
    _K = sc_clip( in0(9), 0.f, 1e5f );
    float gain = in0(10) * sqrtf( 2.f / _L );
    _nmodes_req = in0(11);

    UpdateFilterCoeffs();

    int vs = nSamples;
    
    do {    
        float y1_h = 0.f;
        float v0_h = 0.f;
        
        for (int n=0; n<_nmodes; ++n) {
            // free devel state
            _z_h[n] = zapgremlins( _X1[n] * _z[n] + _X2[n] * _zdot[n]);
            _zdot_h[n] = _Y1[n] * _z[n] + _Y2[n] * _zdot[n]; 
            // free state @ finger
            y1_h += _Phi1[n] * _z_h[n];
            // free state @ posin
            v0_h += _Phi0[n] * _zdot_h[n];
        }
        
        float Fext = *vpin++;
    
        // Finger force 
        float F1 = _C11 * y1_h  + _C12 * Fext;        
        float fbridge = 0.f;

        // apply forces
       for ( int n=0; n<_nmodes; ++n ) {
            _z[n] = zapgremlins(_z_h[n] + _Alpha0[n] * Fext + _Alpha1[n] * F1);
            _zdot[n] = _zdot_h[n] + _Beta0[n] * Fext + _Beta1[n] * F1;
            fbridge += _Kbridge[n] * _z[n];
        }

        *vpout++ = zapgremlins(fbridge) * gain;

    } while (--vs);
}

void PHMStringCollision2::reset(){
    std::memset(_z,0.f,sizeof(_z));
    std::memset(_zdot,0.f,sizeof(_zdot));
    std::memset(_z_h,0.f,sizeof(_z_h));
    std::memset(_zdot_h,0.f,sizeof(_zdot_h));
}

void PHMStringCollision2::UpdateFilterCoeffs(){
    
    float sr = sampleRate();
    float dt = 1.f / sr;
    float nyq = sr * 0.5f;
    float piL = pi / _L;
    //float I = ( pi * powf( _dia, 4 ) ) / 64.f;  // inertia
    float T = 4.f * _f0 * _f0 * _L * _L * _rho;
    _A01 = 0.f;
    _A11 = 0.f;
    _B00 = 0.f;
    _B01 = 0.f;
    
    float vposin = _posin * _L;
    float vfingerpos = _finger_pos * _L;
    float s2L = sqrtf(2.f / _L);
    _nmodes=0;
    
    for ( int n=0; n < _nmodes_req; ++n ) {       
        
        float k = (n+1) * piL;
        float d = _d1 + _d2 * n * n;
        
        //float omega0 = sqrt( T / _rho*k*k + _E*I/_rho*k*k*k*k );
        float omega0 = sqrt( T / _rho * k*k + _dispersion * k*k*k*k );

        float omega = sqrt( omega0 * omega0 - d*d );
        float vfreq = omega / 2.f / pi;
        
        if (vfreq >= nyq){
            //printf("warning: string: freq over Nyquist vfreq = %f\n",vfreq);
            break;
        }
        
        _nmodes++;
        
        float theta = omega * dt;
        float R = expf(-d * dt);
        
        // free devel coeffs
        float vx1 = ( cosf(theta) + d / omega * sinf(theta) ) * R;
        float vx2 = 1.f / omega*sinf(theta)*R;
        float vx3 = 1.f / ( _rho * omega0 * omega0) * ( 1.f - vx1);

        float vy1 = -(omega + d*d/omega) * sinf(omega*dt)*R;
        float vy2 = ( cosf(omega * dt) - d/omega * sinf( omega * dt) ) * R;
        float vy3 = -1.f / ( omega0 * omega0 * _rho ) * vy1;
   
        // coeffs for bridge force calc
        _Kbridge[n] = T * n * pi / _L + _dispersion / _rho * k * k * k;
        
        // store coeffs
        _X1[n] = vx1;
        _X2[n] = vx2;
        _X3[n] = vx3;
        _Y1[n] = vy1;
        _Y2[n] = vy2;
        _Y3[n] = vy3;
        
        // kernels @ pos in
        float vphi0 = s2L * sinf( k * vposin );
        float va0 = vphi0 * _X3[n];
        float vb0 = vphi0 * _Y3[n];

        // kernels @ finger pos
        float vphi1 = s2L * sinf( k * vfingerpos );
        float va1 = vphi1 * _X3[n];
        float vb1 = vphi1 * _Y3[n];
        
        // coeffs for finger force calculation
        _A01 += vphi1 * va0; // cross-coeff (in 2 finger pos)
        _A11 += vphi1 * va1;
        
        // coeffs for friction force calc
        _B00 += vphi0 * vb0;
        _B01 += vphi0 * vb1; // cross-coeff (in 2 finger pos)
        
        //printf( "mode %d \t %f \t %f \t %f \t %f \t %f \n", n, vphi0, vphi1, va0, vb1, vb0 );

        _Phi0[n] = vphi0;
        _Phi1[n] = vphi1;
        _Alpha0[n] = va0;
        _Alpha1[n] = va1;
        _Beta0[n] = vb0;
        _Beta1[n] = vb1;
    }
    
    _C11 = -_K / ( 1.f + _K * _A11 );
    _C12 = _A01 * _C11;
    _C01 = 1.f / ( _B00 + _B01 * _C12 );
    _C02 = -_B01 * _C11 * _C01;

   printf( "_C11 %f \t _C12 %f \t _C01 %f \t _C02 %f  \n", _C11,_C12, _C01, _C02 );
}
    
} // namespace


PluginLoad(PHMStringCollision2UGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<PHMStringCollision2::PHMStringCollision2>(ft, "PHMStringCollision2", false);
} 