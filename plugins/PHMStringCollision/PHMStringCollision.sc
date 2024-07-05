PHMStringCollision : UGen {
	*ar { |input, gain, f0, d1, d2, stiff, thres |
		/* TODO */
		^this.multiNew('audio', input, gain, f0, d1, d2, stiff, thres );
	}
	checkInputs {
		/* TODO */
		^this.checkValidInputs;
	}
}
