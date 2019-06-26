using Caesar, Gadfly, YAML, DelimitedFiles
using Cairo, Fontconfig

#Test with sines
xin = 0:0.001:2*pi
raw = sin.(100*collect(xin)) + sin.(10*collect(xin)) + sin.(collect(xin));
f1 = 80; f2=120;
cztlen=1024;
Fs = 1000;
m = nextpow(2,length(raw))
signal = zeros(m);
signal[1:length(raw)] = raw;

#Test with raw data
#cfgd = YAML.load(open(joinpath(dirname(pathof(Caesar)),"config","SAS2D.yaml")))
#dataFile = joinpath(ENV["HOME"],"data","sas","sample_data","waveform1600.csv");
#raw = readdlm(dataFile,',',Float64,'\n')
#f1 = cfgd["freq_lower_cutoff"]; f2=cfgd["freq_upper_cutoff"];
#Fs = cfgd["freq_sampling"];
#cztlen = cfgd["nfft_czt"];
#m = nextpow(2,size(raw,2)
#signal = zeros(m);
#signal[1:size(raw,2)] = raw[1,:];

w = exp(-2im*pi*(f2-f1)/m/Fs)
myczt = Caesar.prepCZTFilter(m,1,w,cztlen)
out = zeros(Complex{Float64}, cztlen)
@time myczt(collect(signal),out)

pl = Gadfly.plot(
 Gadfly.layer(y=real(signal), Geom.line),
 Gadfly.Coord.cartesian(xmin=0, xmax=m),
 Guide.ylabel(nothing),Guide.xlabel("Samples")
 ) ; pl |> PDF("/tmp/test1.pdf")

 pl = Gadfly.plot(
  Gadfly.layer(y=real(out),Geom.line, Theme(default_color="red")),
  Gadfly.Coord.cartesian(xmin=0, xmax=1024),
  Guide.ylabel(nothing),Guide.xlabel("Samples")
  ) ; pl |> PDF("/tmp/test2.pdf")
