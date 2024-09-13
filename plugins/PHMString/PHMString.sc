PHMString : UGen {
	*ar { |input, gain, f0=55, loss1=0.001, loss2=0.00001, dispersion=0.1, length=1.0, diameter=0.001, posin=0.2, posout=0.3, nmodes=80  |
		^this.multiNew('audio', input, gain, f0, loss1, loss2, dispersion, length, diameter, posin, posout, nmodes );
	}
	checkInputs {
		^this.checkValidInputs;
	}
}
