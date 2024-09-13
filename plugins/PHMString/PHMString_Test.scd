s.boot;
s.quit;

(
SynthDef('phmstring1',{
	arg f0=100.0;
	var trig = Impulse.ar(1);
	var exc = EnvGen.ar(Env.perc(0.01,0.0,level:400,curve:1),trig);
	var out = PHMString.ar( exc, gain:1, f0:f0, loss1:0.001, loss2:0.000001, dispersion:0.1,length:0.8,diameter:0.0005, posin:0.1, posout:0.8,nmodes:100 );
	Out.ar(0,[out,out]);
}).add;

)

Synth.new(\phmstring1,[\f0,100]);

(
SynthDef('phmstring2',{
	arg f0=100.0;
	var trig = Impulse.ar(4);
	var exc = EnvGen.ar(Env.perc(0.001,0.0,level:100,curve:1),trig);
	var out = PHMString.ar( exc, gain:1, f0:f0, loss1:0.001, loss2:0.000001, dispersion:0.1,length:0.8,diameter:0.0005, posin:LFNoise1.kr(3).range(0,1), posout:0.8,nmodes:160 );
	Out.ar(0,[out,out]);
}).add;

)

Synth.new(\phmstring2,[\f0,100]);





