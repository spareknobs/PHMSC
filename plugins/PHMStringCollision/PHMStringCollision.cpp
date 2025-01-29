// PluginPHMStringCollision.cpp
// rm (spare knobs@site.com)

#include "PHMStringCollision.hpp"
//#include "../utils.hpp"

static InterfaceTable* ft;

namespace PHMStringCollision {

PHMStringCollision::PHMStringCollision() {
    std::memset(_y1,0.f,sizeof(_y1));
    std::memset(_y2,0.f,sizeof(_y2));
    mCalcFunc = make_calc_function<PHMStringCollision, &PHMStringCollision::next>();
    next(1);
    printf("loading: PHMStringCollision v 0.2 \n");
}

void PHMStringCollision::next(int nSamples) {
    
    const float* input = in(0);
    float* outbuf = out(0);
    float sr = sampleRate();
    float sr1 = 1.f / sr;
    float nyq = sr * 0.5f;
    float flimit = sr * 0.25f;

    // get control rate parameters
    const float gain = in0(1);
    const float f0 = sc_clip(in0(2), 15.f, nyq );
    float L = sc_clip(in0(3), 0.01, 10.0);
    const float d1 = sc_clip(in0(4), 0.0001, 100.0);
    const float d2 = sc_clip(in0(5), 0.f, 10.f);
    const float disprs = sc_clip(in0(6), 0.f, 20.f );
    const float posin = sc_clip(in0(7), 0.f, 1.f );
    const float posout = sc_clip(in0(8), 0.f, 1.f);
    float cThres = sc_clip(in0(9), -10.f, -0.00001 );
    float cK = sc_clip( in0(10), 0.f, 1e5f );
    const float cpmin = sc_clip( in0(11), 0.f, 1.f );
    const float cpmax =  sc_clip( in0(12), 0.f, 1.f );
    const int nmodes_req = in0(13);
    const int ncoll = sc_clip( in0(14),0,gncollidersmax);
    const int randp = in0(15);
    float dia = 0.0005f;
    float density = 6000.f;
    float sigma = dia * dia * 0.25 * pi * density;
    int nmodes = nmodes_req;
    float Tension = 4.f * f0 * f0 * L * L * sigma; // approx without damps & stiffness
    float f2 = ( Tension/sigma + d1*d2/(2*sigma*sigma ) );
    float f3 = powf( d1 / (2*sigma),2 );
    float sigma1 = 1.f / sigma;
    float fac = 2.f / L;
    
    // update collision pos only when param changes, unless 'rand' flag is 1
    // in that case, the collision positions change randomly runtime in the given range
    if ( randp || ( cpmax !=_cpmax || ncoll != _ncoll ) ){
       // printf("\n updating collisions pos: \n");
        _cpmax = cpmax;
        _ncoll = ncoll;
        for (int c=0; c < ncoll; ++c) {
            float rnd = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            _cpos[c] = cpmin + rnd * (cpmax - cpmin);
            //printf(" %f \t", _cpos[c]);
        }
        //printf("\n");
    }

    for (int i=0; i < nmodes_req; ++i) {
        float b1,a1,a2,win,wout,cwin,cwout;
        float g = (i+1) * pi / L;
        float omega = sqrt(  disprs * g * g * g * g + f2 * g * g - f3 );
        float freq = omega / pi / 2.f;
        
        if ( freq > flimit ){
            nmodes=i;
            //printf("fc exceeds maxfreq\n");
            //printf("nmodes reduced to: %d \n", nmodes);
            break;
        }
        else{
            float d = d1 + d2 * g * g;
            float poleR = -d / ( 2 * sigma * sr);
            float omgsr = omega * sr1;
            _a1[i]  =  2.f * expf(poleR) * cosf( omgsr );
            _a2[i] = -expf( 2.f * poleR ); 
            _b1[i] =  ( sigma1 / omega * sr1 ) * sinf( omgsr );
            _win[i] =  sinf( posin * g );
            _wout[i] = fac * sinf( posout * g );
            
            for (int c=0; c < ncoll; ++c) {
                _cwin[c][i] = sinf( _cpos[c] * g );
                _cwout[c][i] = fac * _cwin[c][i];
            }
        }
    }
// for (int i=0; i < 10; ++i) {
//     for (int c=0; c < ncoll; ++c) {
//     printf(" %f \t", _cwin[c][i]);
//     }
//     printf("\n");
// }
    int vs=nSamples;
    const float* vpin = input;
    float* vpout = outbuf;
    do {
        float x = *vpin++;
        float displ = 0; //  @ pickup position
        float fc[ncoll]; // visco-elastic collision
        std::memset(fc,0.f,sizeof(fc));
        for (int c=0; c < ncoll; ++c) {
            float vdelta = cThres - _cdispl[c];
            if ( vdelta > 0 ){
                fc[c] = vdelta * cK;
            }
        }
        std::memset(_cdispl,0.f,sizeof(_cdispl));

        for (int n=0; n<nmodes; ++n) {
            float b1 = _b1[n];
            float a1 =  _a1[n];
            float a2 =  _a2[n];
            float win = _win[n];
            float wout = _wout[n];
            float fcw = 0.f;
            for (int c=0; c < ncoll; ++c) {
                fcw += _cwin[c][n] * fc[c];
            }

            float y = b1 * ( win * x + fcw ) + a1 * _y1[n] + a2 * _y2[n];
            _y2[n] = _y1[n];
            _y1[n] = zapgremlins(y);
            displ += y * wout;
            
            for (int c=0; c < ncoll; ++c) {
                 _cdispl[c] += y * _cwout[c][n];
             }
        }
        *vpout++ = zapgremlins(displ) * gain;
    } while (--vs);
}

} // namespace PHMStringCollision


PluginLoad(PHMStringCollisionUGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<PHMStringCollision::PHMStringCollision>(ft, "PHMStringCollision", false);
}
