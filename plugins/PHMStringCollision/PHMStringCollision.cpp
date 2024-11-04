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

void PHMStringCollision::computeStringModes( 
                                int& nmodes, 
                                const float L,
                                const float f0,
                                const float disprs,
                                const float sigma, 
                                const float d1, 
                                const float d2,
                                const int nmodes_req ){   
    float sr = sampleRate();
    float sr1 = 1.0 / sr;
    float flimit = sr * 0.25f;
    float nyq = sr * 0.5f;
    float Tension = 4.f * f0 * f0 * L * L * sigma; // approx without damps & stiffness
    float f2 = ( Tension/sigma + d1*d2/(2*sigma*sigma ) );
    float f3 = powf( d1 / (2*sigma),2 );
    float sigma1 = 1.f / sigma;
    for (int i=0; i < nmodes_req; ++i) {

        float b1,a1,a2,win,wout,cwin,cwout;
        float g = (i+1) * pi / L;
        float omega = sqrt(  disprs * g * g * g * g + f2 * g * g - f3 );
        float freq = omega / pi / 2.f;
        
        if ( freq > flimit ){
            //printf("fc exceeds maxfreq\n");
            nmodes=i;
            //printf("nmodes reduced to: %d \n", nmodes);
            //printf("max freq: %f", freq );    
            break;
        }
        else{
            // fix as in FTM
            float d = d1 + d2 * g * g;
            float poleR = -d / ( 2 * sigma * sr);
            float omgsr = omega * sr1;
            a1  =  2.f * expf(poleR) * cosf( omgsr );
            a2 = -expf( 2.f * poleR ); 
            b1 =  ( sigma1 / omega * sr1 ) * sinf( omgsr );
        }
        _a1[i]=a1;
        _a2[i]=a2;
        _b1[i]=b1;
    }
}

void PHMStringCollision::next(int nSamples) {
    
    const float* input = in(0);
    float* outbuf = out(0);
    float sr = sampleRate();
    float sr1 = 1.f / sr;
    float nyq = sr * 0.5f;

    // Control rate parameters
    const float gain = in0(1);
    const float f0 = sc_clip(in0(2), 15.f, nyq );
    float L = sc_clip(in0(3), 0.01, 10.0);
    const float d1 = sc_clip(in0(4), 0.0001, 100.0);
    const float d2 = sc_clip(in0(5), 0.f, 10.f);
    const float disprs = sc_clip(in0(6), 0.f, 10.f );
    float posin = sc_clip(in0(7), 0.f, 1.f );
    float posout = sc_clip(in0(8), 0.f, 1.f);
    float cThres = sc_clip(in0(9), -10.f, -0.00001 );
    float cK = sc_clip( in0(10), 0.f, 1e5f );
    float cpos = sc_clip( in0(11), 0.f, 1.f );
    const int nmodes_req = in0(12);

    float dia = 0.0005f;
    float density = 6000.f;
    float sigma = dia * dia * 0.25 * pi * density;
    int nmodes = nmodes_req;
    float fac = 2.f / L;
    computeStringModes( nmodes, L, f0, disprs, sigma, d1, d2, nmodes_req );

	for (int i=0; i<nmodes; ++i) {
		float g = (i+1) * pi;
		_win[i] =  sinf( posin * g );
        _wout[i] = fac * sinf( posout * g );
        _cwin[i] = sinf( cpos * g );
        _cwout[i] = fac * sinf( cpos * g );
    }
    int vs=nSamples;
    const float* vpin = input;
    float* vpout = outbuf;
    do {
        float x = *vpin++;
        float displ = 0; //  @ pickup position
        float cdispl = 0; //  @ collision position
        float fc = 0; // visco-elastic collision
        float vdelta = cThres - _cdispl;
        if ( vdelta > 0 ){
            fc = vdelta * cK;
        }
        for (int n=0; n<nmodes; ++n) {
            float b1 = _b1[n];
            float a1 =  _a1[n];
            float a2 =  _a2[n];
            float win = _win[n];
            float wout = _wout[n];
            float cwout = _cwout[n];
            float cwin = _cwin[n];
            float y = b1 * ( win * x + cwin * fc ) + a1 * _y1[n] + a2 * _y2[n];
            _y2[n] = _y1[n];
            _y1[n] = y;
            displ += y * wout;
            cdispl += y * cwout;
        }

        _cvel = (cdispl - _cdispl) * sr;
        _cdispl = cdispl;
        *vpout++ = displ * gain;
    } while (--vs);
}

} // namespace PHMStringCollision

PluginLoad(PHMStringCollisionUGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<PHMStringCollision::PHMStringCollision>(ft, "PHMStringCollision", false);
}
