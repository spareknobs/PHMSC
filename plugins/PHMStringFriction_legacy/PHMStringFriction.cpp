
#include "PHMStringFriction.hpp"
#include "../utils.hpp"

static InterfaceTable* ft;

namespace PHMStringFriction {

PHMStringFriction::PHMStringFriction() {
    std::memset(_y1,0.f,sizeof(_y1));
    std::memset(_y2,0.f,sizeof(_y2));
    mCalcFunc = make_calc_function<PHMStringFriction, &PHMStringFriction::next>();
    next(1);
}

void PHMStringFriction::next(int nSamples) {
    
    //const float* input = in(0);
    float* outbuf = out(0);
    float sr = sampleRate();
    float sr1 = 1.0/sr;
    float nyq = sampleRate()*0.5f;

    // Control rate parameters
    const float vel = in0(0);
    const float pressure = in0(1);
    const float gain = in0(2);
    const float f0 = in0(3);
    const float d1 = in0(4);
    const float d2 = in0(5);
    const float disprs = 0.01;
    const int nmodes_req = 80;
    float dia = 0.0005f;
    float length = 1.0f;
    
    float L = sc_clip(length,0.0,1.0);
    float density = 6000.f;
    float vs = 0.1; // stribeck
    float posin = 0.25*L;
    float posout = 0.3*L;
    float sigma = dia * dia * 0.25 * pi * density;
    //float b1c = sr1 * sr1 / sigma;
    int nmodes = nmodes_req;

    float mud = 0.197;
    float mus = 0.97;
    float fs = mus * pressure; // stiction force
    float fc = mud * pressure;

    computeStringModes(_a1,_a2,_b1,nmodes,L,f0,disprs,sigma,d1,d2,nmodes_req,sr);
    
    float pil = pi / L;
	for (int i=0; i<nmodes; ++i) {
		float g = (i+1) * pil;
		_wout[i] = sinf( posout * g );
        _cwin[i] = 2.0 / L * sinf( posin * g );
        _cwout[i] = sinf( posin * g );
    }

    for (int i = 0; i < nSamples; ++i) {
        float displ = 0; //  @ pickup position
        float cdispl = 0; //  @ frict position
        
        // compute the friction force
        float frict=0;
        float dv = _cvel - vel;
        frict =  ( fc + (fs-fc) * expf( -powf( dv/vs, 2) ) );
        if (dv<0){
            frict *= -1;
        }	
        if (vel==0){
            frict=0;
        }

        for (int n=0; n<nmodes; ++n) {
            float y = _b1[n] * ( _cwin[n] * frict ) + _a1[n] * _y1[n] + _a2[n] * _y2[n];
            _y2[n] = _y1[n];
            _y1[n] = y;
            displ += y * _wout[n];
            cdispl += y * _cwout[n];
        }
        _cvel = (cdispl - _cdispl) * sr;
        _cdispl = cdispl;
        outbuf[i] = displ * gain;
    }
}

} // namespace PHMStringFriction

PluginLoad(PHMStringFrictionUGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<PHMStringFriction::PHMStringFriction>(ft, "PHMStringFriction", false);
}
