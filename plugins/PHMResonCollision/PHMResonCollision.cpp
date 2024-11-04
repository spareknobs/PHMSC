// PluginPHMResonCollision.cpp
// rm (spare knobs@site.com)

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
                                            const float detune,
                                            const float d1,
                                            const float d2,
                                            const int nmodes ){   
    //float sr = //sampleRate();
    float sr = sampleRate();
    float sr1 = 1.f / sr;
    float vnyq = sr * 0.5f;

    if (fmin != _fmin || fmax != _fmax){
        _fmin=fmin;
        _fmax=fmax;

        // uniform dist
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
        float vf = sc_clip( _fc[i] * ( 1.f + detune ), 20.f, vnyq );
        float omega = vf * twopi;
        float d = d1 + d2 * powf( vf * sr1, 2 );
        float bw = omega * sc_clip(d,0.000001, 20.0);
        float r = expf( -bw * sr1 );
        float b1c = 1.0;  // :TODO: fix to normalize gain
        a1 =  2.f * r  * cosf( omega * sr1 );
        a2 = - r*r;
        pA1[i]=a1;
        pA2[i]=a2;
        pB1[i]=b1c;
     
		//float g = (i+1) * pil;
		_win[i] = 1.f; //sinf( posin * g );
        _wout[i] = 1.f; //sinf( posout * g );
        _cwin[i] = 1.f; //2.0 / L * sinf( posin * g );
        _cwout[i] = 1.f; //sinf( posin * g );
    }
}

void PHMResonCollision::next(int nSamples) {
    
    const float* input = in(0);
    float* outbuf = out(0);
    float sr = sampleRate();
    float sr1 = 1.0/sr;
    float nyq = sampleRate()*0.5f;

    // Control rate parameters
    const float gain = in0(1);
    const float fmin = in0(2);
    const float fmax = in0(3);
    const float detune = in0(4);
    const float d1 = in0(5);
    const float d2 = in0(6);
    float cThres = in0(7);
    float cK = in0(8);
    float cD = in0(9);
    const int nmodes = in0(10);

    //float cpos = 0.2*L;
    
    computeRandomModes(_a1,_a2,_b1,fmin,fmax,detune, d1,d2,nmodes);

    int vs=nSamples;
    const float* vpin = input;
    float* vpout = outbuf;
    do {
        float x = *vpin++;
        float displ = 0; //  @ pickup position
        float cdispl = 0; //  @ collision position
        float fc = 0; // visco-elastic collision
        //float vdelta = cThres - _cdispl;
        //if ( vdelta > 0 ){
          //  fc = vdelta * cK - cD * _cvel;
        //}
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
            //cdispl += y * cwout;
        }

        _cvel = (cdispl - _cdispl) * sr;
        //_cdispl = cdispl;
        *vpout++ = displ * gain;
    } while (--vs);
}

} // namespace PHMResonCollision

PluginLoad(PHMResonCollisionUGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<PHMResonCollision::PHMResonCollision>(ft, "PHMResonCollision", false);
}
