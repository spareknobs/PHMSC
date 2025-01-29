PHMFlue1 : UGen {
	*ar { |input, gain=1.0, f0=55.0, d1=0.01, d2=0.001, dispersion=0.01, pressure=10, vibgain=0, noisegain=0.02, nmodes=80 |
		^this.multiNew('audio', input, gain, f0, d1, d2, dispersion, pressure, vibgain, noisegain,  nmodes );
	}
	checkInputs {
		^this.checkValidInputs;
	}
}
