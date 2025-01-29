PHMStringFriction : UGen {
	*ar { |vel, pressure, gain, f0, d1, d2 |
		/* TODO */
		^this.multiNew('audio', vel,pressure, gain, f0, d1, d2 );
	}
	checkInputs {
		/* TODO */
		^this.checkValidInputs;
	}
}
