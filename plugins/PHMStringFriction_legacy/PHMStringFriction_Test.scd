
s.quit;
s.boot;

(
SynthDef('cstr1',{
	arg f0=100.0;
	var out = PHMStringFriction.ar(
		vel:LFNoise1.kr(15).range(-0.2,0.2),pressure:LFNoise1.kr(2).range(0.5,4),
		gain:2, f0:f0, d1:0.001, d2:0.0001);
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}).add;
)

Synth.new(\cstr1,[\f0,50]);




