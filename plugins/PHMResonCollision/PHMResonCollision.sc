PHMResonCollision : UGen {
	*ar { |input, gain=1.0, fmin=60.0, fmax=6000, detune=0.0, d1=0.01, d2=0.001, thres= -0.5, c_stiff=3000, c_damp=0, nmodes=80 |
		^this.multiNew('audio', input, gain, fmin, fmax, detune, d1, d2, thres, c_stiff, c_damp, nmodes );
	}
	checkInputs {
		^this.checkValidInputs;
	}
}
