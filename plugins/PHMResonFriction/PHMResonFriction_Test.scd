s.quit;

(
s.reboot;
s.meter(0,2);
//s.scope();
)

(
{
	var vel = LFNoise1.kr(1.0).range(0.06, 0.1);
//	var vel = LFNoise1.kr(1.0).range(-0.01, 0.01);
	//var vel = SinOsc.ar(3.0).range(-0.1, 0.2);
	var bowpos = LFNoise2.kr(2).range(0.2,0.2);
	var pressure = LFNoise2.kr(1).range(0.4, 0.7);
	var out = PHMResonFriction.ar(vel:vel, pressure:pressure, bowpos:bowpos, fingerpos:1.0, fmin:30.0, fmax:12000, d1:15, d2:0.1,  gain:0.0001, nmodes:500);
	out = LeakDC.ar(out);
	out = Limiter.ar(out, level:0.8, dur:0.1);
	Out.ar(0,[out,out]);
}.play;
)

(
{
	var vel = SinOsc.ar(4.0).range(-0.1, 0.2);
	var bowpos = LFNoise2.kr(2).range(0.21,0.2);
	var pressure = LFNoise2.kr(1).range(0.2, 0.3);
	var out = PHMResonFriction.ar(vel:vel, pressure:pressure, bowpos:bowpos, fingerpos:1.0, fmin:30.0, fmax:14000, d1:20, d2:1.1,  gain:0.0001, nmodes:500);
	out = LeakDC.ar(out);
	out = Limiter.ar(out, level:0.8, dur:0.1);
	Out.ar(0,[out,out]);
}.play;
)




