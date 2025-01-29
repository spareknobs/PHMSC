
#include "PHMStringFriction2.hpp"
#include "../utils.hpp"

static InterfaceTable* ft;

namespace PHMStringFriction2 {


PHMStringFriction2::PHMStringFriction2() {
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
    mCalcFunc = make_calc_function<PHMStringFriction2, &PHMStringFriction2::next>();
    next(1);
}

void PHMStringFriction2::next(int nSamples) {
    
    float* vpout = out(0);
    float sr = sampleRate();
    float dt = 1.f / sr;
    float nyq = sampleRate() * 0.5f;

    // Control rate parameters
    float bow_vel = in0(0); // bow velocity (typical: 0.2 - 0.5 m/s)
    float bow_pressure = in0(1); // bow force   (0.1 to 2 N)
    _bow_pos = sc_clip(in0(2), 0.001f, 0.999f);
    _finger_pos = sc_clip(in0(3), 0.001f, 0.999f);
    _f0 = sc_clip( in0(4), 20.f, nyq );
    _d1 = sc_clip( in0(5), 0.0001f, 100.f);
    _d2 = sc_clip( in0(6), 0.f, 1.f );
    float gain = in0(7) * sqrtf( 2.f / _L );
    _K = in0(8);    // finger stiff
    _nmodes_req = in0(9);

    UpdateFilterCoeffs();
    UpdateFingerCoeffs();

    // update bow force - don't allow nonzero bow force with zero bow vel
    float Fb = bow_pressure * (float)(bow_vel!=0.f);
    float Fmax = Fb * _Mu_s;

    // noise gen
    RGen& rgen = *this->mParent->mRGen;                                                                                
    uint32 s1 = rgen.s1;
    uint32 s2 = rgen.s2;
    uint32 s3 = rgen.s3;

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
            // free state @ bow
            v0_h += _Phi0[n] * _zdot_h[n];
        }
        
        // friction force
        float F0 = 0.f;
        if (mstate == stick){
            // compute force with dV = 0
            F0 = _C01 * (bow_vel - v0_h) + _C02 * y1_h;
            // Test: if F > Fmax -> slip
            if ( fabs(F0) > Fmax ){
                mstate = slip;
            }
        }
        else{ // slip
            // compute discriminant
            float Vc = _V0 * copysignf( 1.0, bow_vel );
            //printf("Vc = %f \n", Vc );
            float Fc = Fb * copysignf(1.0, bow_vel);
            float c2 = -_C01 / Vc;
            float c1 = _C01 * ( Vc - bow_vel + v0_h ) / Vc + _Mu_d * Fc/Vc - _C02 * y1_h / Vc;
            float c0 = _C01 * (bow_vel-v0_h) + _C02 * y1_h - _Mu_s * Fc;
            float Delta = c1 * c1 - 4 * c0 * c2;
            
            // if Delta < 0 -> go to stick
            if ( Delta < 0 ){
                mstate = stick;
                F0 = _C01 * ( bow_vel - v0_h ) + _C02 * y1_h;
            }
            else {
                float dV = ( -c1 + sqrtf(Delta) ) / ( 2 * c2 );
                // if dV*bow_vel > 0 -> stick
                if ( dV * bow_vel > 0 ){
                    mstate = stick;
                    F0 = _C01 * ( bow_vel - v0_h ) + _C02 * y1_h;
                }
                else{
                    // else we are slipping, compute using:
                    F0 = ( _Mu_d + ( _Mu_s - _Mu_d )/ ( 1 - dV/Vc ) ) * Fc;
                    float wnz = frand2(s1, s2, s3);
                    F0 += wnz;
                }
            }
        }
        
        // zero force if pressure is below thres
        F0 = (float)( Fb >= cBowForceThres ) * F0;
        
        // Finger force 
        float F1 = _C11 * y1_h  + _C12 * F0;        
        float fbridge = 0.f;

        // apply forces
        for ( int n=0; n<_nmodes; ++n ) {
            _z[n] = zapgremlins(_z_h[n] + _Alpha0[n] * F0 + _Alpha1[n] * F1);
            _zdot[n] = _zdot_h[n] + _Beta0[n]*F0 + _Beta1[n] * F1;
            fbridge += _Kbridge[n] * _z[n];
        }

        if (isnan(fbridge) || isinf(fbridge)){
            //printf("warning - NaN detected, r \n"); 
            reset();
        }
        
        *vpout++ = zapgremlins(fbridge) * gain;

    } while (--vs);
}


/*void PHMStringFriction2::NoteOn(const int acnote){
    int vdelta = acnote - Tuning;
    if (vdelta>=0){
        vdelta = vdelta%24;
        float vnormlen = 1.f / powf( 2.f, (float)(vdelta) / 12.f );
        SetFingerPosNorm(vnormlen);
    }
}*/

void PHMStringFriction2::reset(){
    std::memset(_z,0.f,sizeof(_z));
    std::memset(_zdot,0.f,sizeof(_zdot));
    std::memset(_z_h,0.f,sizeof(_z_h));
    std::memset(_zdot_h,0.f,sizeof(_zdot_h));
}

void PHMStringFriction2::UpdateFilterCoeffs(){
    float sr = sampleRate();
    float dt = 1.f / sr;
    float nyq = sr * 0.5f;
    float piL = pi / _L;
    float I = ( pi * powf( _dia, 4 ) ) / 64.f;  // inertia
    float T = 4.f * _f0 * _f0 * _L * _L * _rho;
    _nmodes=0;
    //printf("---------------------------------------\n"); 
    for (int n=0; n<_nmodes_req; ++n) {       
        float k = (n+1) * piL;
        float d = _d1 + _d2 * n * n;
        float omega0 = sqrt( T / _rho*k*k + _E*I/_rho*k*k*k*k );
        float omega = sqrt( omega0*omega0 - d*d );
        float vfreq = omega / 2.f / pi;
        
        if (vfreq >= nyq){
            //printf("warning: string: freq over Nyquist vfreq = %f\n",vfreq);
            break;
        }
        
        _nmodes++;
        
        float theta = omega*dt;
        float R = expf(-d*dt);
        
        // free devel coeffs
        float vx1 = ( cosf(theta) + d/omega*sinf(theta) ) * R;
        float vx2 = 1.f / omega*sinf(theta)*R;
        float vx3 = 1.f / ( _rho * omega0 * omega0) * ( 1.f - vx1);

        float vy1 = -(omega + d*d/omega) * sinf(omega*dt)*R;
        float vy2 = ( cosf(omega * dt) - d/omega * sinf( omega * dt) ) * R;
        float vy3 = -1.f / ( omega0 * omega0 * _rho ) * vy1;
   
        // coeffs for bridge force calc
        _Kbridge[n] = T * n * pi / _L + _E * I * k * k * k;
        
        // save coeffs
        _X1[n] = vx1;
        _X2[n] = vx2;
        _X3[n] = vx3;
        _Y1[n] = vy1;
        _Y2[n] = vy2;
        _Y3[n] = vy3;

        //printf( "mode %d \t %f \t %f \t %f \t %f \t %f \n", n, vfreq, _Kbridge[n], vy1, vy2, vy3 );
    }
}

void PHMStringFriction2::UpdateFingerCoeffs(){
    _A01=0.f;
    _A11=0.f;
    _B00=0.f;
    _B01=0.f;
    //printf("---------------------------------------\n"); 
    float vbowpos = _bow_pos * _L;
    float vfingerpos = _finger_pos * _L;
    float s2L = sqrtf(2.f / _L);
    float piL = pi / _L;
    
    for (int n=0; n < _nmodes; ++n) {
            
        float k = (n+1) * piL;
        
        // kernels @ bow pos
        float vphi0 = s2L * sinf( k * vbowpos );
        float va0 = vphi0 * _X3[n];
        float vb0 = vphi0 * _Y3[n];

        // kernels @ finger pos
        float vphi1 = s2L * sinf( k * vfingerpos );
        float va1 = vphi1 * _X3[n];
        float vb1 = vphi1 * _Y3[n];
        
        // coeffs for finger force calculation
        _A01 += vphi1 * va0; // cross-coeff (bow 2 finger pos)
        _A11 += vphi1 * va1;
        
        // coeffs for friction force calc
        _B00 += vphi0 * vb0;
        _B01 += vphi0 * vb1; // cross-coeff (bow 2 finger pos)
        
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

    //printf( "_C11 %f \t _C12 %f \t _C01 %f \t _C02 %f  \n", _C11,_C12, _C01, _C02 );
}

} // namespace


PluginLoad(PHMStringFriction2UGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<PHMStringFriction2::PHMStringFriction2>(ft, "PHMStringFriction2", false);
}
