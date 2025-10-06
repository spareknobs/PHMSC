
#include "PHMResonCustom.hpp"
#include "../utils.hpp"
#include <fstream>
#include <cstring>

static InterfaceTable* ft;

namespace PHMResonCustom {

PHMResonCustom::PHMResonCustom() {
    std::memset(_y1,0.f,sizeof(_y1));
    std::memset(_y2,0.f,sizeof(_y2));
    _cdispl = 0.f;
    m_fbufnum = -1e9f;
    m_failedBufNum = -1e9f;

    mCalcFunc = make_calc_function<PHMResonCustom, &PHMResonCustom::next>();
    next(1);
}

void PHMResonCustom::computeModes(const float d1,
                            const float d2,
                            const float posin,
                            const float cposin,
                            const float detune,
                            const int nmodes){ 
    // Ensure we don't try to use more modes than available 
    _nmodes = std::min(nmodes, _navailmodes);
    
    float flim = sampleRate() * 0.4f;
    float sr1 = 1.f / sampleRate();
    float sr1_squared = sr1 * sr1;
 
    for (int i = 0; i < _nmodes; ++i) {
        float vf = sc_clip(_freqs[i] * detune, 1.f, flim);
        float omega = vf * twopi;
        float d = d1 + d2 * vf; // (vf * vf * sr1_squared);
        float omgsr = omega * sr1;
        float poleR = -d * 0.5f * sr1;
        float exp_poleR = expf(poleR);
        
        _a1[i] = 2.f * exp_poleR * cosf(omgsr);
        _a2[i] = -exp_poleR * exp_poleR;
        _b1[i] = sinf(omgsr) / omega;
        
        const float g = (i + 1) * pi;
        _win[i] = _mags[i] * sinf(posin * g);  // Use the loaded magnitude and the pos?
        _cwin[i] = sinf(cposin * g);
       // Print("PHMResonCustom: %d a1: %f a2: %f b1: %f  win: %f\n", i, _a1[i],_a2[i],_b1[i], _win[i]);
    }
}

void PHMResonCustom::next(int nSamples) {
    
    const float* input = in(0);
    float* outbuf = out(0);
    
    // Control rate parameters
    const float fbufnum = in0(1);   
    const float gain = sc_clip(in0(2), 0.f, 20.f);
    const float d1 = sc_clip(in0(3), 0.000000001f, 1000.f);
    const float d2 = sc_clip(in0(4), 0.0f, 1000.f);
    const float cthres = sc_clip(in0(5), -10.f, 0.f);
    const float cK = sc_clip(in0(6), 0.f, 5000.f);
    const float posin = sc_clip(in0(7), 0.0001f, 0.9999f);
    const float cposin = sc_clip(in0(8), 0.0001f, 0.9999f);
    const float detune = sc_clip(in0(9), 0.1f, 10.f);
    const int nmodes_req = sc_clip(static_cast<int>(in0(10)), 1, gnmodesmax);

    if (fbufnum != m_fbufnum) {
        uint32 bufnum = (int)fbufnum;
        World* world = mWorld;
        if (bufnum >= world->mNumSndBufs){
            bufnum = 0;
            Print("PHMResonCustom: invalid buffer number.\n");
            return;
        }
        m_fbufnum = fbufnum;
        m_buf =  world->mSndBufs + bufnum;
        const SndBuf* buf = m_buf;
        ACQUIRE_SNDBUF_SHARED(buf);
        const float* bufData __attribute__((__unused__)) = buf->data;
        uint32 bufChannels __attribute__((__unused__)) = buf->channels;
        uint32 bufSamples __attribute__((__unused__)) = buf->samples;
        uint32 bufFrames = buf->frames;
        
        if (bufChannels != 2){
            Print("PHMResonCustom: invalid buffer: must contain 2 channels. \n");
            return;
        }                                                                                             
        else if (!bufData) {                                                                                                    
            if (mWorld->mVerbosity > -1 && !mDone && (m_failedBufNum != fbufnum)) {                      
                Print("PHMResonCustom: no buffer data\n");                                                                    
                m_failedBufNum = fbufnum; 
                return;
            }                                                                                                              
        }           
        _navailmodes = static_cast<int>(bufFrames);
        //Print("Got buffer! available nmodes: %d  \n", _navailmodes);
        _freqs.resize(_navailmodes,0);
        _mags.resize(_navailmodes,0);
        for (int i = 0; i < _navailmodes; ++i) {
            const float* mode = bufData + i * bufChannels;
            float freq = mode[0];
            float mag = mode[1];
            _freqs[i] = freq;
            _mags[i] = mag;
           // Print("PHMResonCustom: Loading mode %d freq: %f  mag: %f\n", i, freq, mag);
        }
    }
   
    computeModes(d1, d2, posin, cposin, detune, nmodes_req);

    int vs = nSamples;
    const float* vpin = input;
    float* vpout = outbuf;
    float cdispl = _cdispl;
    
    do {
        float x = *vpin++;
        float displ = 0.f;
        float cdispl_local = 0.f;
        float vdelta = cthres - cdispl;
        float fc = (vdelta > 0.f) ? vdelta * cK : 0.f;
        
        for (int n = 0; n < _nmodes; ++n) {
            float y = _b1[n] * (_win[n] * x + _cwin[n] * fc) + 
                          _a1[n] * _y1[n] + 
                          _a2[n] * _y2[n];
            _y2[n] = _y1[n];
            _y1[n] = zapgremlins(y);
            displ += _win[n] * y;
            cdispl_local += _cwin[n] * y;
        }
        
        cdispl = zapgremlins(cdispl_local);
        *vpout++ = zapgremlins(displ) * gain;
    } while (--vs);
    
    _cdispl = cdispl;
}

} // namespace PHMResonCustom

PluginLoad(PHMResonCustomUGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<PHMResonCustom::PHMResonCustom>(ft, "PHMResonCustom", false);
} 