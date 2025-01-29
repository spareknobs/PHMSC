s.quit;

(
//s.options.outDevice = "Audio Out";
s.options.outDevice = "SQ - Audio";
s.reboot;
s.meter(0,2);
s.scope();
)


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
(
{
	var env = EnvGen.ar(Env.linen(0.1, 60, 2, level:1.0), gate:1, doneAction:2 ) + LFNoise1.kr(6).range(0,0.5);
	var out = PHMFlue1.ar( env, gain:200, f0:380, dispersion:0,
		d1:0.0001, d2:10, pressure:200, vibgain:0, noisegain:0.005, nmodes: 120 );
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}.play;
)



(
{
	var env = EnvGen.ar(Env.linen(0.1, 60, 2, level:1.0), gate:1, doneAction:2 ) + LFNoise1.kr(6).range(0,0.5);
	var out = PHMFlue1.ar( env, gain:200, f0:240, dispersion:0,
		d1:0.00001, d2:10, pressure:200, vibgain:0, noisegain:0.001, nmodes: 120 );
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}.play;
)

(
{
	var env = EnvGen.ar(Env.linen(0.1, 60, 2, level:1.0), gate:1, doneAction:2 ) + LFNoise1.kr(6).range(0,0.8);
	var out = PHMFlue1.ar( env, gain:200, f0:420, dispersion:0,
		d1:0.00001, d2:20, pressure:200, vibgain:0, noisegain:0.001, nmodes: 120 );
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}.play;
)

(
{
	var env = EnvGen.ar(Env.linen(0.1, 60, 2, level:1.0), gate:1, doneAction:2 ) + LFNoise1.kr(6).range(0,0.8);
	var out = PHMFlue1.ar( env, gain:200, f0:69.midicps, dispersion:0,
		d1:0.00001, d2:20, pressure:200, vibgain:0, noisegain:0.001, nmodes: 120 );
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}.play;
)
