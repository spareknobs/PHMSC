// PluginPHMStringCollision.cpp
// rm (spare knobs@site.com)

#include "PHMStringCollision.hpp"
#include "../utils.hpp"

static InterfaceTable* ft;

namespace PHMStringCollision {

PHMStringCollision::PHMStringCollision() {
    std::memset(_y1,0.f,sizeof(_y1));
    std::memset(_y2,0.f,sizeof(_y2));
    mCalcFunc = make_calc_function<PHMStringCollision, &PHMStringCollision::next>();
    next(1);
}

void PHMStringCollision::next(int nSamples) {
    
    const float* input = in(0);
    float* outbuf = out(0);
    float sr = sampleRate();
    float sr1 = 1.0/sr;
    float nyq = sampleRate()*0.5f;

    // Control rate parameters
    const float gain = in0(1);
    const float f0 = in0(2);
    float L = sc_clip(in0(3),0.01,10.0);
    const float d1 = in0(4);
    const float d2 = in0(5);
    const float disprs = in0(6);
    float posin = in0(7)*L;
    float posout = in0(8)*L;
    float cThres = in0(9);
    float cK = in0(10);
    float cD = in0(11);
    const int nmodes_req = in0(12);

    float dia = 0.0005f;
    float density = 6000.f;
    float cpos = 0.2*L;
    float sigma = dia * dia * 0.25 * pi * density;
    float b1c = sr1 * sr1 / sigma;
    int nmodes = nmodes_req;

    computeStringModes(_a1,_a2,_b1,nmodes,L,f0,disprs,sigma,d1,d2,nmodes_req,sr);

	float pil = pi / L;
	for (int i=0; i<nmodes; ++i) {
		float g = (i+1) * pil;
		_win[i] = sinf( posin * g );
        _wout[i] = sinf( posout * g );
        _cwin[i] = 2.0 / L * sinf( posin * g );
        _cwout[i] = sinf( posin * g );
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
            fc = vdelta * cK - cD * _cvel;
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
