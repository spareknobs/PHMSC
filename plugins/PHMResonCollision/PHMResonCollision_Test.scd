(
//s.options.outDevice = "Audio Out";
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
	var attack = LFNoise1.kr(3).range(0.00001,0.001);
	var exc = EnvGen.ar(Env.perc(attack, releaseTime:0.01, level:1,curve:-4),trig) * BrownNoise.ar();
	var out = PHMResonCollision.ar( exc, gain:1, fmin:75, fmax:4000, d1:4.6, d2:0.005, thres:-0.2, c_stiff:3000, posin:0.94, cposin:0.33, nmodes:80);
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}.play;
)


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// dense exciter
(
{
	var trig = Dust.ar(17);
	var attack = LFNoise1.kr(3).range(0.00001,0.001);
	var exc = EnvGen.ar(Env.perc(attack, releaseTime:0.01, level:1,curve:-4),trig) * BrownNoise.ar();
	var out = PHMResonCollision.ar( exc, gain:1, fmin:75, fmax:8000, d1:40.3, d2:0.001, thres:-0.01, c_stiff:4000, posin:0.94, cposin:0.33, nmodes:80);
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}.play;
)



