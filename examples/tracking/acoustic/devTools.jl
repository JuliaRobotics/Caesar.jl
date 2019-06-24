
## Quick test of envelope detection


pl = Gadfly.plot(y=leftwavdata[0], Geom.line);

pl |> PDF("/tmp/test.pdf",30cm,20cm)



# now abs and correlate with 2^16 size


leftenv = loadPs3WavEnvelopes(joinpath(datadir,"left_wavdata_corrected.jld2"))

pl = vstack(
Gadfly.plot(y=leftenv[0], Geom.line),
Gadfly.plot(y=leftenv[1], Geom.line),
Gadfly.plot(y=leftenv[2], Geom.line),
Gadfly.plot(y=leftenv[3], Geom.line),
);

pl |> PDF("/tmp/test.pdf",20cm,30cm)


##

@async run(`evince /tmp/test.pdf`)
