PHMResonCustom : UGen {
	*ar { |input, modes_buf=0, gain=1.0, d1=0.01, d2=0.001, thres= -0.5, c_stiff=3000, posin=0.23, cposin=0.33, detune=1.0, nmodes=80 |
		^this.multiNew('audio', input, modes_buf, gain, d1, d2, thres, c_stiff, posin, cposin, detune,nmodes);
	}
	checkInputs {
		^this.checkValidInputs;
	}
} 