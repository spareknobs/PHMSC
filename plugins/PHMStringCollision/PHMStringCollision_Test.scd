
s.quit;
s.reboot;
s.meter(0,2);
s.scope();
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
(
{
	var f0 = 1000.0;
	var trig = Impulse.ar(0.4); //Dust.ar(0.3);
	var exc = EnvGen.ar(Env.perc(0.005, 0.001,level:100,curve:-4),trig);//*WhiteNoise.ar();
	var out = PHMStringCollision.ar( exc, gain:1, f0:f0, length:1.0, dispersion:10, d1:0.00001, d2:0.00001, posIn:0.4, posOut:0.33, thres:-0.0001, c_stiff:2000, c_pos:LFNoise0.kr(29.8).range(0.2, 0.21).scope, nmodes: 100 );
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}.play;
)


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Low rumbling
(
{
	var f0 = 60.0;
	var trig = Impulse.ar(0.4); //Dust.ar(0.3);
	var exc = EnvGen.ar(Env.perc(0.005, 0.001,level:100,curve:-4),trig);//*WhiteNoise.ar();
	var out = PHMStringCollision.ar( exc, gain:0.5, f0:f0, length:1.0, dispersion:0.1, d1:0.00001, d2:0.00001, posIn:0.4, posOut:0.33, thres:-0.0001, c_stiff:2000, c_pos:LFNoise0.kr(29.8).range(0.0, 1.0).scope, nmodes: 100 );
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}.play;
)

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// High warbling
(
{
	var f0 = 500.0;
	var trig = Impulse.ar(0.4); //Dust.ar(0.3);
	var exc = EnvGen.ar(Env.perc(0.005, 0.001,level:100,curve:-4),trig);//*WhiteNoise.ar();
	var out = PHMStringCollision.ar( exc, gain:1, f0:f0, length:1.0, dispersion:0.1, d1:0.00001, d2:0.00001, posIn:0.4, posOut:0.33, thres:-0.0001, c_stiff:3000, c_pos:LFNoise0.kr(30).range(0.0, 1.0).scope, nmodes: 100 );
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}.play;
)

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// warbles!!
(
{
	var f0 = 200.0;
	var trig = Impulse.ar(0.4); //Dust.ar(0.3);
	var exc = EnvGen.ar(Env.perc(0.005, 0.001,level:100,curve:-4),trig);//*WhiteNoise.ar();
	var out = PHMStringCollision.ar( exc, gain:1, f0:f0, length:1.0, dispersion:0, d1:0.00001, d2:0.00001, posIn:0.4, posOut:0.33, thres:-0.0001, c_stiff:2000, c_pos:LFNoise0.kr(29.8).range(0.2, 0.21).scope, nmodes: 100 );
	out = LeakDC.ar(out);
	Out.ar(0,[out,out]);
}.play;
)