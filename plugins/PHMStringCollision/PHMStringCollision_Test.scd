

s.boot;
(
SynthDef('cstr1',{
	arg f0=100.0;
	var trig = Impulse.ar(1);
	var exc = EnvGen.ar(Env.perc(0.01,0.0,level:100,curve:1),trig);
	var thres = LFNoise1.kr(2.0).range(-0.1,0.0).scope;
	var out = PHMStringCollision.ar( exc, gain:1, f0:f0, d1:0.001, d2:0.00001, stiff:0.5, thres:thres ).scope;
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}).add;
)

var cs = Synth.new(\cstr,[\f0,200]);

(
SynthDef('cstr2',{
	arg f0=100.0;
	var trig = Dust.ar(4);
	var exc = EnvGen.ar(Env.perc(0.01,0.0,level:100,curve:1),trig).scope;
	var thres = LFNoise2.kr(3.0).range(-0.1,0.1).scope;
	var out = PHMStringCollision.ar( exc, gain:1, f0:200.0, d1:0.0005, d2:0.00001, stiff:0.9, thres:thres ).scope;
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}).add;
)

var cs = Synth.new(\cstr2,[\f0,200]);

(
{
	var trig = Dust.ar(3);
	var exc = EnvGen.ar(Env.perc(0.01,0.0,level:100,curve:1),trig).scope;
	var out = PHMStringCollision.ar( exc, gain:1, f0:100.0, d1:0.001, d2:0.00001, stiff:0.5, thres:-0.001 ).scope;
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}.play;
)


(
SynthDef('cstr',{
	var trig = Impulse.ar(1);
	var exc = EnvGen.ar(Env.perc(0.01,0.0,level:100,curve:1),trig);
	var thres = LFNoise1.kr(2.0).range(-0.1,0.0).scope;
	var out = PHMStringCollision.ar( exc, gain:1, f0:100.0, d1:0.001, d2:0.00001, stiff:0.5, thres:thres ).scope;
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}).add;
)




