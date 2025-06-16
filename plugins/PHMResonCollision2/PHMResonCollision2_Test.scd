(
//s.options.outDevice = "Audio Out";
//s.options.outDevice = "SQ - Audio";
//s.options.outDevice = "Audient iD14";
//s.options.outDevice = "Soundcraft Signature 12 MTK";
s.options.numInputBusChannels = 0;
s.options.numOutputBusChannels = 2;
s.reboot;
s.meter(0,2);
)

s.quit;

(
{
	var trig = Dust.ar(2);
	var attack = LFNoise1.kr(3).range(0.0001,0.01);
	var exc = EnvGen.ar(Env.perc(attack, releaseTime:0.0, level:1,curve:-4),trig);
	var out = PHMResonCollision2.ar( exc, gain:4, fmin:500, fmax:600, d1:5, d2:0.001, thres:-0.00, c_stiff:0, posin:0.94, cposin:0.33, nmodes:70, detune:0.0, spread:LFNoise1.kr(0.25).range(0,1));
	out = LeakDC.ar(out);
	out = Limiter.ar(out, level:0.9, dur:0.1);
	out = Pan2.ar(out,Rand(0,1) );
	Out.ar(0,out);
}.play;
)
