// PluginPHMFlue1.cpp
// A simple flue model consisting of a nonlinear excitation mechanism and a modal resonator.
// Adapted from SDK::Blow by Perry R. Cook and Gary P. Scavone, 1995--2023. 
// riccardo marogna 2024

#include "PHMFlue1.hpp"
#include <cstring>

static InterfaceTable* ft;

namespace PHMFlue1 {

PHMFlue1::PHMFlue1() {
    std::memset(_y1,0.f,sizeof(_y1));
    std::memset(_y2,0.f,sizeof(_y2));
    mCalcFunc = make_calc_function<PHMFlue1, &PHMFlue1::next>();
    next(1);
    printf("loading: PHMFlue1 v 0.1 \n");
}

void PHMFlue1::computeModes( int& nmodes, 
                                const float f0,
                                const float disprs,
                                const float d1, 
                                const float d2,
                                const int nmodes_req ){   
    float sr = sampleRate();
    float sr1 = 1.0 / sr;
    float sr2 = sr*sr;
    float nyq = sr * 0.5f;
    float b1 = 1 / sr2;
    for (int i=0; i < nmodes_req; ++i) {
        
        float fc = f0 * (i+1);
        if ( fc > nyq ){
            nmodes=i;
            //printf("nmodes reduced to: %d \n", nmodes);
            break;
        }
        else{
            float omega = 2*pi*fc;
            float d = d1 + d2 * (fc/sr) * (fc/sr);
            float Q = 0.5f / d;
            float bw = 2.f * fc * d;
            float  r = expf( -omega * sr1 * 2  * d );
            float a1 = 2.f * r * cosf( omega * sr1 );
            float a2 = -(r*r);
            _a1[i]=a1;
            _a2[i]=a2;
            _b1[i]=b1;
        }
    }
}

void PHMFlue1::next(int nSamples) {
    
    const float* input = in(0);
    float* outbuf = out(0);
    float sr = sampleRate();
    float sr1 = 1.f / sr;
    float nyq = sr * 0.5f;

    // Control rate parameters
    const float gain = in0(1);
    const float f0 = sc_clip(in0(2), 15.f, nyq );
    const float d1 = sc_clip(in0(3), 0.0001, 10.0);
    const float d2 = sc_clip(in0(4), 0.f, 10.f);
    const float disprs = sc_clip(in0(5), 0.f, 10.f );
    const float maxPressure = sc_clip(in0(6), 0.f, 200.f );
    const float vibratoGain = sc_clip(in0(7), 0.f, 100.f);
    const float noiseGain = sc_clip(in0(8), 0.f, 100.f);
    const int nmodes_req = in0(9);
    int nmodes = nmodes_req;
    computeModes( nmodes, f0, disprs, d1, d2, nmodes_req );

	int vs = nSamples;
    const float* vpin = input;
    float* vpout = outbuf;
    RGen& rgen = *this->mParent->mRGen;                                                                                \
    uint32 s1 = rgen.s1;                                                                                               \
    uint32 s2 = rgen.s2;                                                                                               \
    uint32 s3 = rgen.s3;
    
    do {
        float env = *vpin++;
        
        //# Calculate the breath pressure (envelope + vibrato)
        float breathPressure = maxPressure * env;
        //breathPressure += vibratoGain * sinf(2*pi*5*t/sr);

        //# pressure difference at the pipe entrance
        float pressureDiff = breathPressure - res_out_prev;

        //# add random turbulence
        float wnz = frand2(s1, s2, s3);
        
        float randPressure = noiseGain * wnz;
        randPressure *= breathPressure;
        randPressure *= (1.0 + pressureDiff);

        //# Polynomial calculation (x^3 - x), which approximates the jet sigmoid behavior.
        float pd = pressureDiff;
        float jt = pd * ( pd * pd - 1.0 );
        float jet = sc_clip(jt, -1.0, 1.0); // Saturate at +/- 1.0.
        
        float p_in = breathPressure + randPressure  - ( jet * pressureDiff );
        float displ=0.f;

        // update resonator filterbank
        for (int n=0; n<nmodes; ++n) {
            float b1 = _b1[n];
            float a1 =  _a1[n];
            float a2 =  _a2[n];
            float y = b1 * p_in + a1 * _y1[n] + a2 * _y2[n];
            _y2[n] = _y1[n];
            _y1[n] = y;
            displ += y;
        }

        res_out_prev = displ;
        *vpout++ = displ * gain;
    } while (--vs);

    rgen.s1 = s1;                                                                                                      \
    rgen.s2 = s2;                                                                                                      \
    rgen.s3 = s3;

}

} // namespace PHMFlue1

PluginLoad(PHMFlue1UGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<PHMFlue1::PHMFlue1>(ft, "PHMFlue1", false);
}
