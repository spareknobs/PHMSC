s.reboot;
s.quit;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
(
{
	var trig = Dust.ar(2);
	var exc = EnvGen.ar(Env.perc(LFNoise1.kr(3).range(0.00001, 0.001), 0.01,level:0.0001,curve:-4),trig)*BrownNoise.ar();
	var out = PHMResonCollision.ar( exc, gain:0.1, fmin:50, fmax:2000, d1:0.0009, d2:0.0005, c_stiff:0.0, c_damp:0, thres:-0.6, nmodes:100);
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}.play;
)





