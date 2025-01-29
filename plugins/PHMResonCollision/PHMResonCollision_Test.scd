(
//s.options.outDevice = "Audio Out";
//s.options.outDevice = "SQ - Audio";
//s.options.outDevice = "Audient iD14";
s.options.outDevice = "Soundcraft Signature 12 MTK";
s.options.numInputBusChannels = 0;
s.options.numOutputBusChannels = 2;
s.reboot;
s.meter(0,4);
s.scope();
)

s.quit;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// random collisions
(
{
	var trig = Dust.ar(2);
	var attack = LFNoise1.kr(3).range(0.0001,0.04);
	var exc = EnvGen.ar(Env.perc(attack, releaseTime:0.01, level:1,curve:-4),trig) *(1 +  WhiteNoise.ar());
	var out = PHMResonCollision.ar( exc, gain:4, fmin:400, fmax:13000, d1:10, d2:0.001, thres:-0.00, c_stiff:300, posin:0.94, cposin:0.33, nmodes:600, detune:0.0);
	out = LeakDC.ar(out);
	out = Limiter.ar(out, level:0.8, dur:0.1);
	out = Pan2.ar(out,Rand(0,1) );
	Out.ar(0,out);
}.play;
)


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// dense exciter
(
{
	var trig = Dust.ar(16);
	var attack = LFNoise1.kr(3).range(0.0001,0.03);
	var exc = EnvGen.ar(Env.perc(attack, releaseTime:0.02, level:1,curve:-4),trig) * BrownNoise.ar();
	var out = PHMResonCollision.ar( exc, gain:4, fmin:60, fmax:13000, d1:10.3, d2:0.001, thres:-0.001, c_stiff:400, posin:0.4, cposin:0.13, nmodes:400,detune:0.0);
	out = LeakDC.ar(out);
	out = Limiter.ar(out, level:0.9, dur:0.1);
	out = Pan2.ar(out,Rand(0,1) );
	Out.ar(0,out);
}.play;
)


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// LF dense exciter
(
{
	var trig = Dust.ar(17);
	var attack = LFNoise1.kr(3).range(0.001,0.01);
	var exc = EnvGen.ar(Env.perc(attack, releaseTime:0.01, level:1,curve:-4),trig);
	var out = PHMResonCollision.ar( exc, gain:1, fmin:25, fmax:700, d1:15.3, d2:0.001, thres:-0.01, c_stiff:200, posin:0.94, cposin:0.33, nmodes:100,detune:0.0);
	out = LeakDC.ar(out);
	out = Limiter.ar(out, level:0.8, dur:0.1);
	out = Pan2.ar(out,Rand(0,1) );
	Out.ar(0,out);
}.play;
)

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// glissandi
(
{
	var trig = Dust.ar(18);
	//var trig = Impulse.ar(6);
	var attack = LFNoise1.kr(3).range(0.00001,0.001);
	var exc = EnvGen.ar(Env.perc(attack, releaseTime:0.01, level:1,curve:-4),trig) * BrownNoise.ar();
	var out = PHMResonCollision.ar( exc, gain:1, fmin:50, fmax:12000, d1:10, d2:0.01, thres:-0.01, c_stiff:0, posin:0.94, cposin:0.33, nmodes:300,detune:LFNoise2.kr(0.5).range(0,0.1));
	out = LeakDC.ar(out);
	out = Limiter.ar(out, level:0.8, dur:0.1);
	out = Pan2.ar(out,0.5);
	Out.ar(0,out);
}.play;
)