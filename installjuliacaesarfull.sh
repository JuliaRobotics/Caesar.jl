#!/bin/usr/bash

wget https://julialang.s3.amazonaws.com/bin/linux/x64/0.5/julia-0.5.1-linux-x86_64.tar.gz
tar -xvf julia-0.5.1-linux-x86_64.tar.gz
JULIADIR=$(ls --group-directories-first | head -n1)
./$JULIADIR/bin/julia -O3 -e "Pkg.add(\"Caesar\"); Pkg.checkout(\"Caesar\", \"cloudgraphs\"); using Caesar; Caesar.installcloudgraphs()"
./$JULIADIR/bin/julia -O3 -e "Pkg.checkout(\"IncrementalInference\"); Pkg.checkout(\"RoME\")"
./$JULIADIR/bin/julia -O3 -e "using Caesar"
echo "Julia added a default folder at $HOME/.julia"
