PHMStringCollision2 : UGen {
	*ar { |input, f0=55.0, length=1.0, rho=0.00234, dispersion=0.0, d1=1.0, d2=0.001, posin=0.33, fingerpos=0.7, fingerK=0, gain=1.0, nmodes=80 |
		^this.multiNew('audio', input, f0, length, rho, dispersion, d1, d2, posin, fingerpos, fingerK, gain, nmodes );
	}
	checkInputs {
		^this.checkValidInputs;
	}
} 