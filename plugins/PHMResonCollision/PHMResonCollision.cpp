
#include "PHMResonCollision.hpp"
#include "../utils.hpp"


static InterfaceTable* ft;

namespace PHMResonCollision {

PHMResonCollision::PHMResonCollision() {
    std::memset(_y1,0.f,sizeof(_y1));
    std::memset(_y2,0.f,sizeof(_y2));
    _cdispl = 0.f;
    _fmin = 0.f;
    _fmax = 0.f;
    mCalcFunc = make_calc_function<PHMResonCollision, &PHMResonCollision::next>();
    next(1);
    printf("PHMResonCollision v: %.1f\n", VERSION);
    printf("Optimized loop: %d\n", OPTIMIZE_LOOP);
}

void PHMResonCollision::computeRandomModes( const float fmin,
                                            const float fmax,
                                            const float d1,
                                            const float d2,
                                            const float posin, 
                                            const float cposin,
                                            const int nmodes,
                                            const float detune ){   

    const float flim = sampleRate() * 0.33f;
    const float sr1 = 1.f / sampleRate();
    const float sr1_squared = sr1 * sr1;
    
    if (fmin != _fmin || fmax != _fmax) {
        _fmin = sc_clip(fmin, 20.f, flim);
        _fmax = sc_clip(fmax, _fmin, flim);
        _nmodes = ((nmodes + 2) & ~3);

        for (int i = 0; i < _nmodes; ++i) {
            float rnd = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            _fc[i] = _fmin + rnd * (_fmax - _fmin);
        }
    }
    
    for (int i = 0; i < _nmodes; ++i) {
        const float vf = sc_clip(_fc[i] * (1 + detune), 20.f, flim);
        const float omega = vf * twopi;
        const float d = d1 + d2 * (vf * vf * sr1_squared);
        const float omgsr = omega * sr1;
        const float poleR = -d * 0.5f * sr1;
        const float exp_poleR = expf(poleR);
        
        _a1[i] = 2.f * exp_poleR * cosf(omgsr);
        _a2[i] = -exp_poleR * exp_poleR;
        _b1[i] = sinf(omgsr) / omega;
        
        const float g = (i + 1) * pi;
        _win[i] = sinf(posin * g);
        _cwin[i] = sinf(cposin * g);
    }
}

void PHMResonCollision::next(int nSamples) {
    
    const float* input = in(0);
    float* outbuf = out(0);
    float sr = sampleRate();
    float sr1 = 1.f / sr;
    float nyq = sr * 0.5f;

    // Control rate parameters
    const float gain = sc_clip(in0(1), 0.f, 10.f);
    const float fmin = in0(2);
    const float fmax = in0(3);
    const float d1 = sc_clip(in0(4), 0.00001f, 100.f);
    const float d2 = sc_clip(in0(5), 0.00000001f, 1.f);
    const float cthres = sc_clip(in0(6), -10.f, 0.f);
    const float cK = sc_clip(in0(7), 0.f, 5000.f);
    const float posin = sc_clip(in0(8), 0.001f, 0.999f);
    const float cposin = sc_clip(in0(9), 0.001f, 0.999f);
    const int nmodes_req = sc_clip(static_cast<int>(in0(10)), 1, gnmodesmax);
    const float detune = sc_clip(in0(11), -1.f, 1.f);

    computeRandomModes(fmin, fmax, d1, d2, posin, cposin, nmodes_req, detune);

    int vs = nSamples;
    const float* vpin = input;
    float* vpout = outbuf;
    
    float cdispl = _cdispl;
    
    do {
        float x = *vpin++;
        float displ = 0.f;
        float cdispl_local = 0.f;
        const float vdelta = cthres - cdispl;
        const float fc = (vdelta > 0.f) ? vdelta * cK : 0.f;
        
#if OPTIMIZE_LOOP
        // Process 4 elements at a time with loop unrolling
        int n = 0;
        for (; n < _nmodes - 3; n += 4) {
            // First element
            float y0 = _b1[n] * (_win[n] * x + _cwin[n] * fc) + 
                      _a1[n] * _y1[n] + 
                      _a2[n] * _y2[n];
            _y2[n] = _y1[n];
            _y1[n] = zapgremlins(y0);
            displ += _win[n] * y0;
            cdispl_local += _cwin[n] * y0;
            
            // Second element
            float y1 = _b1[n+1] * (_win[n+1] * x + _cwin[n+1] * fc) + 
                      _a1[n+1] * _y1[n+1] + 
                      _a2[n+1] * _y2[n+1];
            _y2[n+1] = _y1[n+1];
            _y1[n+1] = zapgremlins(y1);
            displ += _win[n+1] * y1;
            cdispl_local += _cwin[n+1] * y1;
            
            // Third element
            float y2 = _b1[n+2] * (_win[n+2] * x + _cwin[n+2] * fc) + 
                      _a1[n+2] * _y1[n+2] + 
                      _a2[n+2] * _y2[n+2];
            _y2[n+2] = _y1[n+2];
            _y1[n+2] = zapgremlins(y2);
            displ += _win[n+2] * y2;
            cdispl_local += _cwin[n+2] * y2;
            
            // Fourth element
            float y3 = _b1[n+3] * (_win[n+3] * x + _cwin[n+3] * fc) + 
                      _a1[n+3] * _y1[n+3] + 
                      _a2[n+3] * _y2[n+3];
            _y2[n+3] = _y1[n+3];
            _y1[n+3] = zapgremlins(y3);
            displ += _win[n+3] * y3;
            cdispl_local += _cwin[n+3] * y3;
        }
#else
        // Original non-optimized implementation
        for (int n = 0; n < _nmodes; ++n) {
            const float y = _b1[n] * (_win[n] * x + _cwin[n] * fc) + 
                          _a1[n] * _y1[n] + 
                          _a2[n] * _y2[n];
            _y2[n] = _y1[n];
            _y1[n] = zapgremlins(y);
            displ += _win[n] * y;
            cdispl_local += _cwin[n] * y;
        }
#endif
        
        cdispl = zapgremlins(cdispl_local);
        *vpout++ = zapgremlins(displ) * gain;
    } while (--vs);
    
    _cdispl = cdispl;
}

} // namespace PHMResonCollision

PluginLoad(PHMResonCollisionUGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<PHMResonCollision::PHMResonCollision>(ft, "PHMResonCollision", false);
}
