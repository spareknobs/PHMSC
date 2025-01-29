s.quit;

(
s.reboot;
s.meter(0,2);
//s.scope();
)

(
{
	var vel = LFNoise2.kr(0.5).range(0.1,0.6);
	var bowpos = LFNoise2.kr(1).range(0.2,0.3);
	var pressure = LFNoise2.kr(3.5).range(0.9,1.1);
	var out = PHMStringFriction2.ar(vel:vel, pressure:pressure, bowpos:bowpos,fingerpos:0.7, f0:60.0, d1:0.1, d2:0.2,  gain:0.1, nmodes:100);
	out = LeakDC.ar(out);
	out = Limiter.ar(out, level:0.8, dur:0.1);
	Out.ar(0,[out,out]);
}.play;
)




