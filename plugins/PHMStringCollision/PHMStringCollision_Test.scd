
s.quit;

(
s.reboot;
s.meter(0,2);
)

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// buzzing
(
{
	var f0 = 100.0;
	var trig = Dust.ar(1.3);
	var exc = EnvGen.ar(Env.perc(0.005, 0.001,level:20,curve:-4),trig);//*WhiteNoise.ar();
	var out = PHMStringCollision.ar( exc, gain:1, f0:f0, length:1.0, dispersion:1.4, d1:0.001, d2:0.00001, posIn:0.2, posOut:0.33, thres:-0.01, c_stiff:2000, cpmin:0.01, cpmax:0.5, nmodes: 150, ncoll:20, rand:1 );
	out = LeakDC.ar(out);
	out = Limiter.ar(out, level:0.9, dur:0.1);
	Out.ar(0,[out,out]);
}.play;
)



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// gonglike
(
{
	var f0 = 100.0;
	var trig = Dust.ar(1.3);
	var exc = EnvGen.ar(Env.perc(0.005, 0.001,level:20,curve:-4),trig);//*WhiteNoise.ar();
	var out = PHMStringCollision.ar( exc, gain:1, f0:f0, length:1.0, dispersion:1.4, d1:0.001, d2:0.00001, posIn:0.2, posOut:0.33, thres:-0.0001, c_stiff:2000, cpmin:0.01, cpmax:0.5, nmodes: 150, ncoll:20, rand:1 );
	out = LeakDC.ar(out);
	out = Limiter.ar(out, level:0.9, dur:0.1);
	Out.ar(0,[out,out]);
}.play;
)


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Low rumbling
(
{
	var f0 = 60.0;
//	var trig = Impulse.ar(0.4);
	var trig = Dust.ar(3.8);
	var exc = EnvGen.ar(Env.perc(0.005, 0.001,level:100,curve:-4),trig);//*WhiteNoise.ar();
	var cpos = LFNoise0.kr(29.8).range(0.0, 1.0);
	var out = PHMStringCollision.ar( exc, gain:0.2, f0:f0, length:1.0, dispersion:0.1, d1:0.00001, d2:0.00001, posIn:0.4, posOut:0.33, thres:-0.0001, c_stiff:2000, cpmin:cpos, cpmax:cpos, nmodes: 100 );
	out = LeakDC.ar(out);
	out = Limiter.ar(out, level:0.9, dur:0.1);
	Out.ar(0,[out,out]);
}.play;
)

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// low rumbling 2
(
{
	var f0 = 50.0;
	//var trig = Impulse.ar(0.4); //Dust.ar(0.3);
	var trig = Dust.ar(3.8);
	var exc = EnvGen.ar(Env.perc(0.005, 0.001,level:50,curve:-4),trig);//*WhiteNoise.ar();
	var out = PHMStringCollision.ar( exc, gain:0.3, f0:f0, length:1.0, dispersion:0.4, d1:0.001, d2:0.00001, posIn:0.2, posOut:0.33, thres:-0.002, c_stiff:1000, cpmin:0.3, cpmax:0.3, nmodes: 120, ncoll:1 );
	out = LeakDC.ar(out);
	out = Limiter.ar(out, level:0.9, dur:0.1);
	Out.ar(0,[out,out]);
}.play;
)

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// High warbling
(
{
	var f0 = 500.0;
	var trig = Dust.ar(2.3);
	var exc = EnvGen.ar(Env.perc(0.005, 0.001,level:100,curve:-4),trig);//*WhiteNoise.ar();
	var cpos = LFNoise0.kr(29.8).range(0.0, 1.0);
	var out = PHMStringCollision.ar( exc, gain:1, f0:f0, length:1.0, dispersion:0.1, d1:0.00001, d2:0.00001, posIn:0.4, posOut:0.33, thres:-0.0001, c_stiff:3000, cpmin:cpos, cpmax:cpos, nmodes: 100, ncoll:1 );
	out = LeakDC.ar(out);
	out = Limiter.ar(out, level:0.9, dur:0.1);
	Out.ar(0,[out,out]);
}.play;
)

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// warbles!!
(
{
	var f0 = 200.0;
	var trig = Dust.ar(3);
	var exc = EnvGen.ar(Env.perc(0.005, 0.001,level:100,curve:-4),trig);//*WhiteNoise.ar();
	var cpos = LFNoise0.kr(29.8).range(0.0, 1.0);
	var out = PHMStringCollision.ar( exc, gain:1, f0:f0, length:1.0, dispersion:0, d1:0.00001, d2:0.00001, posIn:0.4, posOut:0.33, thres:-0.0001, c_stiff:2000, cpmin:cpos, cpmax:cpos, nmodes: 100 );
	out = LeakDC.ar(out);
	out = Limiter.ar(out, level:0.9, dur:0.1);
	Out.ar(2,[out,out]);
}.play;
)

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// sustained tone by single collision
(
{
	var f0 = 150.0;
	var trig = Dust.ar(2);
	var exc = EnvGen.ar(Env.perc(0.005, 0.001,level:10,curve:-4),trig);//*WhiteNoise.ar();
	var out = PHMStringCollision.ar( exc, gain:1, f0:f0, length:1.0, dispersion:0.4, d1:0.0001, d2:0.00003, posIn:0.2, posOut:0.33, thres:-0.0001, c_stiff:100, cpmin:0.01, cpmax:0.5, nmodes: 100, ncoll:1, rand:1 );
	out = LeakDC.ar(out);
	out = Limiter.ar(out, level:0.9, dur:0.1);
	Out.ar(2,[out,out]);
}.play;
)