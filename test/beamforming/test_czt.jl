using Caesar, Gadfly, YAML
using Cairo, Fontconfig, DelimitedFiles

#Test with sines
#xin = 0:0.001:2*pi
#raw = sin.(100*collect(xin)) + sin.(10*collect(xin)) + sin.(collect(xin));
#f1 = 80; f2=120;
#cztlen=1024;
#Fs = 1000;

#Test with raw data
cfgd = YAML.load(open(joinpath(dirname(pathof(Caesar)),"config","SAS2D.yaml")))
dataFile = joinpath(ENV["HOME"],"data","sas","sample_data","waveform1545.csv");
raw = readdlm(dataFile,',',Float64,'\n')
f1 = cfgd["freq_lower_cutoff"]; f2=cfgd["freq_upper_cutoff"];
Fs = cfgd["freq_sampling"];
cztlen = cfgd["nfft_czt"];

#fft zero pad if necessary
m = nextpow(2,size(raw,2))
signal = zeros(m);
signal[1:size(raw,2)] = raw[1,:];

w = exp(-2im*pi*(f2-f1)/m/Fs)

myczt = Caesar.prepCZTFilter(m,1,w,cztlen)
out = zeros(Complex{Float64}, cztlen)
@time myczt(collect(signal),out)

Gadfly.plot(
 Gadfly.layer(y=real(signal), Geom.line),
 Gadfly.layer(y=real(out),Geom.line, Theme(default_color="red")),
 Gadfly.Coord.cartesian(xmin=0, xmax=2000, ymin=-15, ymax=25),
 Guide.ylabel(nothing),Guide.xlabel(nothing),
) # for skipping Juno display

pl |> PDF("/tmp/test.pdf");
