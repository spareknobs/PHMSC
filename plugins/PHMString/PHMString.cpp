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
    float sr = sampleRate();
    float sr1 = 1.0/sr;
    float nyq = sampleRate()*0.5f;

    // Control rate parameters
    const float gain = in0(1);
    const float f0 = in0(2);
    const float d1 = in0(3);
    const float d2 = in0(4);
    const float disprs = in0(5);
    const float L = sc_clip(in0(6),0.01,2.0);
    const float dia = in0(7);
    const float posin = in0(8)*L;
    const float posout = in0(9)*L;
    const int nmodes_req = sc_clip(in0(10),1,gnmodesmax);
    float density = 6000.f;
    float sigma = dia * dia * 0.25 * pi * density;
    float b1c = sr1 * sr1 / sigma;
    int nmodes = nmodes_req;

	for (int i=0; i<nmodes_req; ++i) {
		float g = (i+1) * pi / L;
		float fc = f0 * (i+1); 
		fc +=  disprs * g * g / twopi;
		float b1,a1,a2,win,wout;
		if (fc > nyq) {
           // printf("fc exceeds maxfreq\n");
			nmodes=i;
            break;
		}
		else{
			float omega = fc * twopi;
			float d = d1 + d2 * powf( fc / sr, 2 );
			float bw = twopi * fc / 0.5 * sc_clip(d,0.000001, 20.0);
			float r = expf(-bw*sr1); 
			a1 =  2.f * r  * cosf(omega*sr1);
		    a2 = - r*r;
			win =  2.f / L * sinf( posin * g );
    		wout =  sinf( posout * g );
            b1 = b1c * win;
        }
        _a1[i]=a1;
        _a2[i]=a2;
        _b1[i]=b1;
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
        float displ = 0;
        for (int n=0; n<nmodes; ++n) {
            float b1 = _b1[n];
            float a1 =  _a1[n];
            float a2 =  _a2[n];
            float wout = _wout[n];
            float y = b1 * x + a1 * _y1[n] + a2 * _y2[n];
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
