

s.boot;
(
SynthDef('phmstring',{
	arg f0=100.0;
	var trig = Impulse.ar(1);
	var exc = EnvGen.ar(Env.perc(0.01,0.0,level:100,curve:1),trig);
	var thres = LFNoise1.kr(2.0).range(-0.1,0.0).scope;
	var out = PHMString.ar( exc, gain:1, f0:f0, d1:0.001, d2:0.00001, stiff:0.5 ).scope;
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}).add;
)

var cs = Synth.new(\phmstring,[\f0,200]);



