PHMResonCollision2 : UGen {
	*ar { |input, gain=1.0, fmin=60.0, fmax=6000, d1=0.01, d2=0.001, thres= -0.5, c_stiff=3000, posin=0.23, cposin=0.33, nmodes=80, detune=0.0, spread=0.0 |
		^this.multiNew('audio', input, gain, fmin, fmax, d1, d2, thres, c_stiff, posin, cposin, nmodes, detune, spread );
	}
	checkInputs {
		^this.checkValidInputs;
	}
}
