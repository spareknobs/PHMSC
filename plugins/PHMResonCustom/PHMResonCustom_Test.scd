(
s.options.numInputBusChannels = 0;
s.options.numOutputBusChannels = 2;

s.reboot;
s.meter(0,2);
)

s.quit;


// first load modes in buffer
//	Note that multichannel buffers interleave their sample data
(
n = 100;
~freqs = Array.fill(n, {20 + 1000.0.rand});
~mags = Array.fill(n, {1.0.rand});
m = [ Array.newFrom(~freqs), ~mags ]; // a multi-dimensional array
m = m.lace(2*n); // interleave the two collections
b = Buffer.alloc(s, n, 2);
)

(
// load from analysis file
p = "/Users/riccardo/Desktop/test.txt";
a = TabFileReader.read(p, true, true);
~freqs = Array.newClear();
~mags = Array.newClear();
(0..5).do { |n|
	~freqs = ~freqs.add( a[n][0] );
	~mags = ~mags.add(a[n][1]);
};
)

~mags


b = Buffer.alloc(s, n, 2);
b.loadCollection(a,0);



//~mags = Array.fill(n, {1.0.rand});
m = [ Array.newFrom(~freqs), ~mags ]; // a multi-dimensional array
m = m.lace(2*n); // interleave the two collections
b = Buffer.alloc(s, n, 2);
)

b.loadCollection(m,0);
b.plot;


// ... then play
(
{
	var trig, attack, exc, out;
	trig = Dust.ar(4);
	exc = EnvGen.ar(Env.perc(0.001, releaseTime:0.005, level:1,curve:-4),trig) *(1 +  WhiteNoise.ar());
	out = PHMResonCustom.ar( exc, modes_buf:b, gain:1, d1:10, d2:0.000001, thres:-0.00, c_stiff:300, posin:0.94, cposin:0.33, detune:1.0, nmodes:100);
	out = LeakDC.ar(out);
	out = Pan2.ar(out, Rand(0,1) );
	Out.ar(0,out);
}.play;
)


b.free;