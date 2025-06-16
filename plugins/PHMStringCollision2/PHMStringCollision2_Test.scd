
s.quit;

(
s.reboot;
s.meter(0,2);
)

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// buzzing
(
{
	var trig = Dust.ar(2);
	var exc = EnvGen.ar(Env.perc(0.005, 0.001,level:1,curve:-4),trig);//*WhiteNoise.ar();
	var out = PHMStringCollision2.ar( exc, f0:270, length:1.0, rho:1, dispersion:20, d1:1, d2:0.01, posin:0.22, fingerpos: 0.73, fingerK:1000, gain:1, nmodes: 150 );
	out = LeakDC.ar(out);
//	out = Limiter.ar(out, level:0.9, dur:0.1);
	Out.ar(0,[out,out]);
}.play;
)

