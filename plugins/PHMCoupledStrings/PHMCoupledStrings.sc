PHMCoupledStrings : MultiOutUGen {
	*ar { | in, gain=1.0, f1=55.0, f2=60.0, length=1.0, d11=0.01, d12=0.001, d21=0.01, d22=0.001, dispersion=0.01, posin=0.23, posout=0.3, c_stiff=100, c_pos1=0.2, c_pos2=0.56, nmodes=80 |
		^this.multiNew('audio', in, gain, f1, f2, length, d11, d12, d21, d22, dispersion, posin, posout, c_stiff, c_pos1, c_pos2, nmodes );
	}
	
	init {arg ... theInputs;
		inputs = theInputs;
		^this.initOutputs(2, rate);
	}

	checkInputs { ^this.checkNInputs(1); }
}
