#pragma once

#include "SC_PlugIn.hpp"

static void computeStringModes( float* pA1, 
                                float* pA2, 
                                float* pB1, 
                                int& nmodes, 
                                const float L,
                                const float f0,
                                const float disprs,
                                const float sigma, 
                                const float d1, 
                                const float d2,
                                const int nmodes_req,
                                const float sr ){   
    float sr1 = 1.0/sr;
    float flimit = sr*0.25;
    float nyq = sr*0.5f;
    float Tension = ( powf( 2*f0, 2 )) * L * L * sigma; // approx without damps & stiffness
    
    for (int i=0; i<nmodes_req; ++i) {
        float g = (i+1) * pi / L;
        float fc = f0 * (i+1); 
        
        fc +=  disprs * g * g / ( 2*pi);
        float b1,a1,a2,win,wout,cwin,cwout;
        
        if ( fc > nyq ){
           // printf("fc exceeds maxfreq\n");
            nmodes=i;
            //printf("nmodes reduced to: %d", nmodes);
            //printf("max freq: %f", freq );    
            break;
        }
        else{
            float omega = fc * twopi;
            float d = d1 + d2 * powf( fc / sr, 2 );
            float bw = omega * sc_clip(d,0.000001, 20.0);
            float r = expf(-bw*sr1);
            a1 =  2.f * r  * cosf(omega*sr1);
            a2 = - r*r;
        }
        pA1[i]=a1;
        pA2[i]=a2;
        pB1[i]=b1;
    }
}
