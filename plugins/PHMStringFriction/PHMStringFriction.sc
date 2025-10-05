PHMStringFriction : UGen {
	*ar { |vel=0.0, pressure=0.0, bowpos=0.33, fingerpos=0.7, f0=55.0, d1=1.0, d2=0.001,  gain=1.0, fingerK=0, nmodes=80 |
		^this.multiNew('audio', vel, pressure, bowpos, fingerpos, f0, d1, d2, gain, fingerK, nmodes );
	}
	checkInputs {
		^this.checkValidInputs;
	}
}
