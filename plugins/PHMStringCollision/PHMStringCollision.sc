PHMStringCollision : UGen {
	*ar { |input, gain=1.0, f0=55.0, length=1.0, d1=0.01, d2=0.001, dispersion=0.01, posIn=0.23, posOut=0.3, thres= -0.5, c_stiff=3000, cpmin=0.2, cpmax=0.2, nmodes=80, ncoll=1, rand=0 |
		^this.multiNew('audio', input, gain, f0, length, d1, d2, dispersion, posIn, posOut, thres, c_stiff, cpmin, cpmax, nmodes, ncoll, rand );
	}
	checkInputs {
		^this.checkValidInputs;
	}
}
