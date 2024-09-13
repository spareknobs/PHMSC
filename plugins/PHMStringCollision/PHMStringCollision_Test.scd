
s.quit;
s.boot;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
(
{
	var f0 = 520.0;
	var trig = Impulse.ar(0.1); //Dust.ar(0.3);
	var exc = EnvGen.ar(Env.perc(0.0, 0.005,level:400,curve:-4),trig)*WhiteNoise.ar();
	var thres = LFNoise2.kr(2.0).range(-0.0001,0.0001);
	var pos = LFNoise2.kr(0.5).range(0.33,0.999);
	var out = PHMStringCollision.ar( exc, gain:1, f0:f0, length:1.0, d1:0.0002, d2:0.01, stiff:1.1, posIn:0.23, posOut:0.33, thres:thres ).scope;
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}.play;
)


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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




