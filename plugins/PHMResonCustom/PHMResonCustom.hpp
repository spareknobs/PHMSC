// PluginPHMResonCustom.hpp
// rm (spare knobs@site.com)

#pragma once

#include "SC_PlugIn.hpp"
#include <fstream>
#include <vector>

namespace PHMResonCustom {

static const int gnmodesmax = 500;
                                                                                                 

class PHMResonCustom : public SCUnit {
public:
    PHMResonCustom();

private:
    // Calc function
    void next(int nSamples);

    void computeModes(
                            const float d1,
                            const float d2,
                            const float posin,
                            const float cposin,
                            const float detune,
                            const int nmodes); 

    // Member variables
    float _b1[gnmodesmax];
    float _a1[gnmodesmax];
    float _a2[gnmodesmax];
    float _win[gnmodesmax];
    float _cwin[gnmodesmax];
    float _y1[gnmodesmax];
    float _y2[gnmodesmax];
    float _cdispl{0.f};
    float _cvel{0.f};
    int _navailmodes{0};
    int _nmodes{0};
    std::vector<float> _freqs;  // Store frequencies from file
    std::vector<float> _mags;   // Store magnitudes from file
    float m_fbufnum;
    float m_failedBufNum;
    SndBuf* m_buf;
    
};

} // namespace PHMResonCustom 