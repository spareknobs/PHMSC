#include "PHMCoupledStrings.hpp"

static InterfaceTable* ft;

namespace PHMCoupledStrings {

PHMCoupledStrings::PHMCoupledStrings() {

    std::memset(_fb1.y1,0.f,sizeof(_fb1.y1));
    std::memset(_fb1.y2,0.f,sizeof(_fb1.y2));
    std::memset(_fb2.y1,0.f,sizeof(_fb2.y1));
    std::memset(_fb2.y2,0.f,sizeof(_fb2.y2));
    
    mCalcFunc = make_calc_function<PHMCoupledStrings, &PHMCoupledStrings::next>();
    next(1);
    printf("loading: PHMCoupledStrings v 0.1 \n");
    printf("n outs: %d\n",this->mNumOutputs);
}

void PHMCoupledStrings::compute_filterbank( fbank& fb,
                                            const float L,
                                            const float f0,
                                            const float disprs,
                                            const float sigma, 
                                            const float d1, 
                                            const float d2,
                                            const float posin,
                                            const float posout,
                                            const float cpos,
                                            const int nmodes_req ){   
    float sr = sampleRate();
    float sr1 = 1.0 / sr;
    float flimit = sr * 0.25f;
    float nyq = sr * 0.5f;
    float Tension = 4.f * f0 * f0 * L * L * sigma; // approx without damps & stiffness
    float f2 = ( Tension/sigma + d1*d2/(2*sigma*sigma ) );
    float f3 = powf( d1 / (2*sigma),2 );
    float sigma1 = 1.f / sigma;
    float fac = 2.f / L;
    fb.nmodes = nmodes_req;
    for (int i=0; i < nmodes_req; ++i) {

        float b1,a1,a2,win,wout,cwin,cwout;
        float g = (i+1) * pi / L;
        float omega = sqrt(  disprs * g * g * g * g + f2 * g * g - f3 );
        float freq = omega / pi / 2.f;
        
        if ( freq > flimit ){
            //printf("fc exceeds maxfreq\n");
            fb.nmodes=i;
            //printf("nmodes reduced to: %d \n", nmodes);
            //printf("max freq: %f", freq );    
            break;
        }
        else{
            float d = d1 + d2 * g * g;
            float poleR = -d / ( 2 * sigma * sr);
            float omgsr = omega * sr1;
            a1  =  2.f * expf(poleR) * cosf( omgsr );
            a2 = -expf( 2.f * poleR ); 
            b1 =  ( sigma1 / omega * sr1 ) * sinf( omgsr );
        }
        fb.a1[i]=a1;
        fb.a2[i]=a2;
        fb.b1[i]=b1;
		fb.win[i] =  sinf( posin * g );
        fb.wout[i] = fac * sinf( posout * g );
        fb.cwin[i] = sinf( cpos * g );
        fb.cwout[i] = fac * sinf( cpos * g );
    }
}

void PHMCoupledStrings::next(int nSamples) {
    
    const float* input = in(0);
    float* vpoutl = out(0);
    float* vpoutr = out(1);
    float sr = sampleRate();
    float sr1 = 1.f / sr;
    float nyq = sr * 0.5f;

    // Control rate parameters
    const float gain = in0(1);
    const float f1 = sc_clip(in0(2), 15.f, nyq );
    const float f2 = sc_clip(in0(3), 15.f, nyq );
    float L = sc_clip(in0(4), 0.01, 10.0);
    const float d11 = sc_clip(in0(5), 0.0001, 100.0);
    const float d12 = sc_clip(in0(6), 0.f, 10.f);
    const float d21 = sc_clip(in0(7), 0.0001, 100.0);
    const float d22 = sc_clip(in0(8), 0.f, 10.f);
    const float disprs = sc_clip(in0(9), 0.f, 20.f );
    float posin = sc_clip(in0(10), 0.f, 1.f );
    float posout = sc_clip(in0(11), 0.f, 1.f);
    float cK = sc_clip( in0(12), 0.f, 1e5f );
    float cpos1 = sc_clip( in0(13), 0.f, 1.f );
    float cpos2 = sc_clip( in0(14), 0.f, 1.f );
    const int nmodes_req = in0(15);
    float dia = 0.0005f;
    float density = 6000.f;
    float sigma = dia * dia * 0.25 * pi * density;
    
    compute_filterbank( _fb1, L, f1, disprs, sigma, d11, d12, posin, posout, cpos1, nmodes_req );
    compute_filterbank( _fb2, L, f2, disprs, sigma, d21, d22, posin, posout, cpos2, nmodes_req );

    const float* vpin = input;
    int vs = nSamples;
    do {
        float x = *vpin++;
        float dspl1 = 0; //  @ pickup position
        float dspl2 = 0; //  @ pickup position
        float cdspl1 = 0; //  @ connection position
        float cdspl2 = 0; //  @ connection position
        float fc = 0;     // elastic interaction
        // if 2 > 1 -> force directed upwards, positive for 1, negative for 2
        float vdelta = _fb2.cdspl - _fb1.cdspl;
        if ( vdelta > 0 ){
            fc = vdelta * cK;
        }
        for (int n=0; n < _fb1.nmodes; ++n) {
            float y1 = _fb1.b1[n] * ( _fb1.win[n] * x + _fb1.cwin[n] * fc ) + _fb1.a1[n] * _fb1.y1[n] + _fb1.a2[n] * _fb1.y2[n];
            _fb1.y2[n] = _fb1.y1[n];
            _fb1.y1[n] = y1;
            dspl1 += y1 * _fb1.wout[n];
            cdspl1 += y1 * _fb1.cwout[n];
        }
        for (int n=0; n < _fb2.nmodes; ++n) {
            float y1 = _fb2.b1[n] * ( _fb2.cwin[n] * (-fc) ) + _fb2.a1[n] * _fb2.y1[n] + _fb2.a2[n] * _fb2.y2[n];
            _fb2.y2[n] = _fb2.y1[n];
            _fb2.y1[n] = y1;
            dspl2 += y1 * _fb2.wout[n];
            cdspl2 += y1 * _fb2.cwout[n];
        }

        _fb1.cdspl = cdspl1;
        _fb2.cdspl = cdspl2;
        
        *vpoutl++ = dspl1 * gain;
        *vpoutr++ = dspl2 * gain; 

    } while (--vs);
}

} // namespace PHMCoupledStrings

PluginLoad(PHMStringCollisionUGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<PHMCoupledStrings::PHMCoupledStrings>(ft, "PHMCoupledStrings", false);
}
