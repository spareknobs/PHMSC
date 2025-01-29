// PluginPHMStringCollision.hpp
// rm (spare knobs@site.com)

#pragma once

#include "SC_PlugIn.hpp"

namespace PHMCoupledStrings {

static const int gnmodesmax=200;

struct fbank {
    int nmodes{0};
    float b1[gnmodesmax];
    float a1[gnmodesmax];
    float a2[gnmodesmax];
    float win[gnmodesmax];
    float wout[gnmodesmax];
    float cwin[gnmodesmax];
    float cwout[gnmodesmax];
    float y1[gnmodesmax];
    float y2[gnmodesmax];
    float cdspl{0.f};
};


class PHMCoupledStrings : public SCUnit {
public:
    PHMCoupledStrings();

private:
    void compute_filterbank( fbank& fb,
                        const float L,
                        const float f0,
                        const float disprs,
                        const float sigma, 
                        const float d1, 
                        const float d2,
                        const float posin,
                        const float posout,
                        const float cpos,
                        const int nmodes_req);

    void next(int nSamples);

private:
    fbank _fb1;
    fbank _fb2;
    float _cdispl{0.f};
    float _cvel{0.f};

};

} // namespace PHMCoupledStrings
