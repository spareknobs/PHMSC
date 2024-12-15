// PluginPHMResonCollision.cpp

#include "PHMResonCollision.hpp"
#include "../utils.hpp"

static InterfaceTable* ft;

namespace PHMResonCollision {

PHMResonCollision::PHMResonCollision() {
    std::memset(_y1,0.f,sizeof(_y1));
    std::memset(_y2,0.f,sizeof(_y2));
    mCalcFunc = make_calc_function<PHMResonCollision, &PHMResonCollision::next>();
    next(1);
}

void PHMResonCollision::computeRandomModes( float* pA1, 
                                            float* pA2, 
                                            float* pB1, 
                                            const float fmin,
                                            const float fmax,
                                            const float d1,
                                            const float d2,
                                            const float posin, 
                                            const float cposin,
                                            const int nmodes ){   
    float sr = sampleRate();
    float sr1 = 1.f / sr;
    float vnyq = sr * 0.5f;

    if (fmin != _fmin || fmax != _fmax){
        _fmin=fmin;
        _fmax=fmax;

        // uniform random dist
        for (int i=0; i<nmodes; ++i) {
            float rnd = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            _fc[i] = fmin + rnd * sc_clip(fmax - fmin, 0.f, fmax);
            //printf("fc = %f \n",_fc[i]);
        }

        // place the first mode @ fmin
        // distribute random freqs in ascending order
        // :TODO: provide some prob dist (uniform, gaussian, geom...)
        /*_fc[0] = fmin;
        for (int i=1; i<nmodes; ++i) {
            float rnd = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            _fc[i] = _fc[i-1] + rnd * sc_clip(fmax - _fc[i-1], 0.f,fmax);
            printf("fc = %f \n",_fc[i]);
        }*/

        // :TODO: n modess per octave
        /*_fc[0] = fmin;
        for (int i=1; i<nmodes; ++i) {
            float rnd = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            _fc[i] = _fc[i-1] + rnd * sc_clip( fmax - _fc[i-1], 0.f,fmax);
            printf("fc = %f \n",_fc[i]);
        }*/
    }
    
    for (int i=0; i<nmodes; ++i) {
        float b1,a1,a2;
        float vf = sc_clip( _fc[i] * ( 1.f ), 20.f, vnyq );
        float omega = vf * twopi;
        float d = d1 + d2 * powf( vf * sr1, 2 );
        float omgsr = omega * sr1;
        float b1c =  1.f / omega * sinf( omgsr );
        float poleR = -d * 0.5 * sr1;
        a1  =  2.f * expf(poleR) * cosf( omgsr );
        a2 = -expf( 2.f * poleR ); 
        pA1[i]=a1;
        pA2[i]=a2;
        pB1[i]=b1c;
		float g = (i+1) * pi;
		_win[i] = sinf( posin * g );
        _cwin[i] = sinf( cposin * g );
    }
}

void PHMResonCollision::next(int nSamples) {
    
    const float* input = in(0);
    float* outbuf = out(0);
    float sr = sampleRate();
    float sr1 = 1.f / sr;
    float nyq = sr * 0.5f;

    // Control rate parameters
    const float gain = in0(1);
    const float fmin = in0(2);
    const float fmax = in0(3);
    const float d1 = in0(4);
    const float d2 = in0(5);
    const float cthres = in0(6);
    const float cK = in0(7);
    const float posin = in0(8);
    const float cposin = in0(9);
    const int nmodes = in0(10);
    
    computeRandomModes(_a1,_a2,_b1,fmin,fmax, d1,d2,posin,cposin,nmodes);

    int vs=nSamples;
    const float* vpin = input;
    float* vpout = outbuf;
    do {
        float x = *vpin++;
        float displ = 0;  //  @ pickup position
        float cdispl = 0; //  @ collision position
        float fc = 0;     // elastic collision
        float vdelta = cthres - _cdispl;
        if ( vdelta > 0 ){
            fc = vdelta * cK;
        }
        for (int n=0; n<nmodes; ++n) {
            float b1 = _b1[n];
            float a1 =  _a1[n];
            float a2 =  _a2[n];
            float win = _win[n];
            float cwin = _cwin[n];
            float y = b1 * ( win * x + cwin * fc ) + a1 * _y1[n] + a2 * _y2[n];
            _y2[n] = _y1[n];
            _y1[n] = y;
            displ += win * y;
            cdispl += cwin * y;
        }

        _cdispl = cdispl;
        *vpout++ = displ * gain;
    } while (--vs);
}

} // namespace PHMResonCollision

PluginLoad(PHMResonCollisionUGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<PHMResonCollision::PHMResonCollision>(ft, "PHMResonCollision", false);
}
