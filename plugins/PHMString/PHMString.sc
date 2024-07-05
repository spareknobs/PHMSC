PHMString : UGen {
	*ar { |input, gain, f0, d1, d2, stiff |
		/* TODO */
		^this.multiNew('audio', input, gain, f0, d1, d2, stiff );
	}
	checkInputs {
		/* TODO */
		^this.checkValidInputs;
	}
}
