// Plugin PHMString.cpp
// rm (spare knobs@site.com)

#include "SC_PlugIn.hpp"
#include "PHMString.hpp"

static InterfaceTable* ft;

namespace PHMString {

PHMString::PHMString() {
    std::memset(_y1,0.f,sizeof(_y1));
    std::memset(_y2,0.f,sizeof(_y2));
    mCalcFunc = make_calc_function<PHMString, &PHMString::next>();
    next(1);
}

void PHMString::next(int nSamples) {
    
    const float* input = in(0);
    float* outbuf = out(0);

    // Control rate parameters
    const float gain = in0(1);
    const float f0 = in0(2);
    const float d1 = in0(3);
    const float d2 = in0(4);
    const float disprs = in0(5);

    const int nmodes = 80;
    float dia = 0.0005f;
    float length = 1.0f;
    float L = sc_clip(length,0.0,1.0);
    float density = 6000.f;
    float posin = 0.25*L;
    float posout = 0.3*L;
    float sr = sampleRate();
    float sigma = dia * dia * 0.25 * pi * density;

	for (int i=0; i<nmodes; ++i) {
		float g = (i+1) * pi / L;
		float fc = f0 * (i+1); 
		fc +=  disprs * g * g / ( 2*pi);
		float b1,a1,a2,win,wout,cwin,cwout;
		if (fc > maxfreq) {
           // printf("fc exceeds maxfreq\n");
			b1 = 0.f;
			a1 = 0.f;
			a2 = 0.f;
			win=0.f;
			wout=0.f;
			cwin=0.f;
			cwout=0.f;
            _y1[i]=0.f;
            _y2[i]=0.f;
		}
		else{
			float omega = fc * twopi;
			float d = d1 + d2 * powf( fc / sr, 2 );
			float Q = 0.5 / sc_clip(d,0.000001, 20.0);
			float bw = fc / Q;
			float r = expf(-twopi/sr*bw); 
			a1 =  2.0 * r  * cosf( omega / sr );
		    a2 = - r*r;
		    b1 = 1.0 / (sr*sr) / sigma;
			win =  2.0 / L * sinf( posin * g );
    		wout =  sinf( posout * g );
		}
        _a1[i]=a1;
        _a2[i]=a2;
        _b1[i]=b1;
        _win[i]=win;
        _wout[i]=wout;
        
        //printf("sr = %f \n",sr);
        //printf("L = %f \n",L);
        //printf("fc = %f \n",fc);
        //printf("sigma = %f \n",sigma);
        //printf("a1 = %f \n",a1);
        //printf("a2 = %f \n",a2);
        //printf("b1 = %f \n",b1);
        //printf("win = %f \n",win);
        //printf("wout = %f \n",wout);
    }

    for (int i = 0; i < nSamples; ++i) {
        float x = input[i];
        float displ = 0; //  @ pickup position
        for (int n=0; n<nmodes; ++n) {
            float b1 = _b1[n];
            float a1 =  _a1[n];
            float a2 =  _a2[n];
            float win = _win[n];
            float wout = _wout[n];
            float cwout = _cwout[n];
            float cwin = _cwin[n];
            float y = b1 * ( win * x ) + a1 * _y1[n] + a2 * _y2[n];
            _y2[n] = _y1[n];
            _y1[n] = y;
            displ += y * wout;
        }
        outbuf[i] = displ * gain;
    }
}

} // namespace PHMString

PluginLoad(PHMStringUGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<PHMString::PHMString>(ft, "PHMString", false);
}
