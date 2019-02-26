#!/usr/bin/bash

# bash commands to analyze list of folders
# find `find . -name  'N50_*' -print`/ | grep maxAngErr.txt | xargs cat


# find `find . -name  'N50_*' -print`/ | grep maxAngErr.txt | xargs cat > results/N50maxang.dat
# find `find . -name  'N50_*' -print`/ | grep meanAngErr.txt | xargs cat  > results/N50meanang.dat



find `find . -name  'N50_*' -print`/ | grep maxAngErr.txt | xargs cat > results/N50_maxAngErr.dat
find `find . -name  'N50_*' -print`/ | grep meanAngErr.txt | xargs cat > results/N50_meanAngErr.dat
find `find . -name  'N50_*' -print`/ | grep maxRanErr.txt | xargs cat > results/N50_maxRanErr.dat
find `find . -name  'N50_*' -print`/ | grep meanRanErr.txt | xargs cat > results/N50_meanRanErr.dat

find `find . -name  'N75_*' -print`/ | grep maxAngErr.txt | xargs cat > results/N75_maxAngErr.dat
find `find . -name  'N75_*' -print`/ | grep meanAngErr.txt | xargs cat > results/N75_meanAngErr.dat
find `find . -name  'N75_*' -print`/ | grep maxRanErr.txt | xargs cat > results/N75_maxRanErr.dat
find `find . -name  'N75_*' -print`/ | grep meanRanErr.txt | xargs cat > results/N75_meanRanErr.dat

find `find . -name  'N100_*' -print`/ | grep maxAngErr.txt | xargs cat > results/N100_maxAngErr.dat
find `find . -name  'N100_*' -print`/ | grep meanAngErr.txt | xargs cat > results/N100_meanAngErr.dat
find `find . -name  'N100_*' -print`/ | grep maxRanErr.txt | xargs cat > results/N100_maxRanErr.dat
find `find . -name  'N100_*' -print`/ | grep meanRanErr.txt | xargs cat > results/N100_meanRanErr.dat


find `find . -name  'N150_*' -print`/ | grep maxAngErr.txt | xargs cat > results/N150_maxAngErr.dat
find `find . -name  'N150_*' -print`/ | grep meanAngErr.txt | xargs cat > results/N150_meanAngErr.dat
find `find . -name  'N150_*' -print`/ | grep maxRanErr.txt | xargs cat > results/N150_maxRanErr.dat
find `find . -name  'N150_*' -print`/ | grep meanRanErr.txt | xargs cat > results/N150_meanRanErr.dat



find `find . -name  'N200_*' -print`/ | grep maxAngErr.txt | xargs cat > results/N200_maxAngErr.dat
find `find . -name  'N200_*' -print`/ | grep meanAngErr.txt | xargs cat > results/N200_meanAngErr.dat
find `find . -name  'N200_*' -print`/ | grep maxRanErr.txt | xargs cat > results/N200_maxRanErr.dat
find `find . -name  'N200_*' -print`/ | grep meanRanErr.txt | xargs cat > results/N200_meanRanErr.dat



# find `find . -name  'N350_*' -print`/ | grep maxAngErr.txt | xargs cat > results/N350_maxAngErr.dat
# find `find . -name  'N350_*' -print`/ | grep meanAngErr.txt | xargs cat > results/N350_meanAngErr.dat
# find `find . -name  'N350_*' -print`/ | grep maxRanErr.txt | xargs cat > results/N350_maxRanErr.dat
# find `find . -name  'N350_*' -print`/ | grep meanRanErr.txt | xargs cat > results/N350_meanRanErr.dat


find `find . -name  'N500_*' -print`/ | grep maxAngErr.txt | xargs cat > results/N500_maxAngErr.dat
find `find . -name  'N500_*' -print`/ | grep meanAngErr.txt | xargs cat > results/N500_meanAngErr.dat
find `find . -name  'N500_*' -print`/ | grep maxRanErr.txt | xargs cat > results/N500_maxRanErr.dat
find `find . -name  'N500_*' -print`/ | grep meanRanErr.txt | xargs cat > results/N500_meanRanErr.dat



## sim2b double


echo ../sim1/results/N50_maxAngErr.dat | xargs cat >> results/N50_maxAngErr.dat
echo ../sim1/results/N50_meanAngErr.dat | xargs cat >> results/N50_meanAngErr.dat
echo ../sim1/results/N50_maxRanErr.dat | xargs cat >> results/N50_maxRanErr.dat
echo ../sim1/results/N50_meanRanErr.dat | xargs cat >> results/N50_meanRanErr.dat

echo ../sim1/results/N75_maxAngErr.dat | xargs cat >> results/N75_maxAngErr.dat
echo ../sim1/results/N75_meanAngErr.dat | xargs cat >> results/N75_meanAngErr.dat
echo ../sim1/results/N75_maxRanErr.dat | xargs cat >> results/N75_maxRanErr.dat
echo ../sim1/results/N75_meanRanErr.dat | xargs cat >> results/N75_meanRanErr.dat

echo ../sim1/results/N100_maxAngErr.dat | xargs cat >> results/N100_maxAngErr.dat
echo ../sim1/results/N100_meanAngErr.dat | xargs cat >> results/N100_meanAngErr.dat
echo ../sim1/results/N100_maxRanErr.dat | xargs cat >> results/N100_maxRanErr.dat
echo ../sim1/results/N100_meanRanErr.dat | xargs cat >> results/N100_meanRanErr.dat

echo ../sim1/results/N150_maxAngErr.dat | xargs cat >> results/N150_maxAngErr.dat
echo ../sim1/results/N150_meanAngErr.dat | xargs cat >> results/N150_meanAngErr.dat
echo ../sim1/results/N150_maxRanErr.dat | xargs cat >> results/N150_maxRanErr.dat
echo ../sim1/results/N150_meanRanErr.dat | xargs cat >> results/N150_meanRanErr.dat

echo ../sim1/results/N200_maxAngErr.dat | xargs cat >> results/N200_maxAngErr.dat
echo ../sim1/results/N200_meanAngErr.dat | xargs cat >> results/N200_meanAngErr.dat
echo ../sim1/results/N200_maxRanErr.dat | xargs cat >> results/N200_maxRanErr.dat
echo ../sim1/results/N200_meanRanErr.dat | xargs cat >> results/N200_meanRanErr.dat

echo ../sim1/results/N500_maxAngErr.dat | xargs cat >> results/N500_maxAngErr.dat
echo ../sim1/results/N500_meanAngErr.dat | xargs cat >> results/N500_meanAngErr.dat
echo ../sim1/results/N500_maxRanErr.dat | xargs cat >> results/N500_maxRanErr.dat
echo ../sim1/results/N500_meanRanErr.dat | xargs cat >> results/N500_meanRanErr.dat


## some julia to calc std dev

julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N50_maxAngErr.dat")),digits=2))' > results/N50_maxAngErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N50_meanAngErr.dat")),digits=2))' > results/N50_meanAngErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N50_maxRanErr.dat")),digits=2))' > results/N50_maxRanErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N50_meanRanErr.dat")),digits=2))' > results/N50_meanRanErrStd.txt

julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N75_maxAngErr.dat")),digits=2))' > results/N75_maxAngErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N75_meanAngErr.dat")),digits=2))' > results/N75_meanAngErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N75_maxRanErr.dat")),digits=2))' > results/N75_maxRanErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N75_meanRanErr.dat")),digits=2))' > results/N75_meanRanErrStd.txt

julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N100_maxAngErr.dat")),digits=2))' > results/N100_maxAngErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N100_meanAngErr.dat")),digits=2))' > results/N100_meanAngErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N100_maxRanErr.dat")),digits=2))' > results/N100_maxRanErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N100_meanRanErr.dat")),digits=2))' > results/N100_meanRanErrStd.txt

julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N150_maxAngErr.dat")),digits=2))' > results/N150_maxAngErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N150_meanAngErr.dat")),digits=2))' > results/N150_meanAngErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N150_maxRanErr.dat")),digits=2))' > results/N150_maxRanErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N150_meanRanErr.dat")),digits=2))' > results/N150_meanRanErrStd.txt


julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N200_maxAngErr.dat")),digits=2))' > results/N200_maxAngErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N200_meanAngErr.dat")),digits=2))' > results/N200_meanAngErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N200_maxRanErr.dat")),digits=2))' > results/N200_maxRanErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N200_meanRanErr.dat")),digits=2))' > results/N200_meanRanErrStd.txt

julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N500_maxAngErr.dat")),digits=2))' > results/N500_maxAngErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N500_meanAngErr.dat")),digits=2))' > results/N500_meanAngErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N500_maxRanErr.dat")),digits=2))' > results/N500_maxRanErrStd.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(std(readdlm("results/N500_meanRanErr.dat")),digits=2))' > results/N500_meanRanErrStd.txt




julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N50_maxAngErr.dat")),digits=2))' > results/N50_maxAngErrMean.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N50_meanAngErr.dat")),digits=2))' > results/N50_meanAngErrMean.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N50_maxRanErr.dat")),digits=2))' > results/N50_maxRanErrMean.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N50_meanRanErr.dat")),digits=2))' > results/N50_meanRanErrMean.txt

julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N75_maxAngErr.dat")),digits=2))' > results/N75_maxAngErrMean.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N75_meanAngErr.dat")),digits=2))' > results/N75_meanAngErrMean.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N75_maxRanErr.dat")),digits=2))' > results/N75_maxRanErrMean.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N75_meanRanErr.dat")),digits=2))' > results/N75_meanRanErrMean.txt

julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N100_maxAngErr.dat")),digits=2))' > results/N100_maxAngErrMean.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N100_meanAngErr.dat")),digits=2))' > results/N100_meanAngErrMean.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N100_maxRanErr.dat")),digits=2))' > results/N100_maxRanErrMean.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N100_meanRanErr.dat")),digits=2))' > results/N100_meanRanErrMean.txt

julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N200_maxAngErr.dat")),digits=2))' > results/N200_maxAngErrMean.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N200_meanAngErr.dat")),digits=2))' > results/N200_meanAngErrMean.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N200_maxRanErr.dat")),digits=2))' > results/N200_maxRanErrMean.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N200_meanRanErr.dat")),digits=2))' > results/N200_meanRanErrMean.txt

julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N500_maxAngErr.dat")),digits=2))' > results/N500_maxAngErrMean.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N500_meanAngErr.dat")),digits=2))' > results/N500_meanAngErrMean.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N500_maxRanErr.dat")),digits=2))' > results/N500_maxRanErrMean.txt
julia -O3 -e 'using DelimitedFiles, Statistics; println(round(mean(readdlm("results/N500_meanRanErr.dat")),digits=2))' > results/N500_meanRanErrMean.txt



##  draw table


{
echo "range, \$\\rho\$ &"
echo `cat results/N50_meanRanErrStd.txt`' / '`cat results/N50_maxRanErrStd.txt`' & '
echo `cat results/N75_meanRanErrStd.txt`' / '`cat results/N75_maxRanErrStd.txt`' & '
echo `cat results/N100_meanRanErrStd.txt`' / '`cat results/N100_maxRanErrStd.txt`' & '
echo `cat results/N150_meanRanErrStd.txt`' / '`cat results/N150_maxRanErrStd.txt`' & '
}

{
echo "angle, \$\\rho\$ &"
echo `cat results/N50_meanAngErrStd.txt`' / '`cat results/N50_maxAngErrStd.txt`' & '
echo `cat results/N75_meanAngErrStd.txt`' / '`cat results/N75_maxAngErrStd.txt`' & '
echo `cat results/N100_meanAngErrStd.txt`' / '`cat results/N100_maxAngErrStd.txt`' & '
echo `cat results/N150_meanAngErrStd.txt`' / '`cat results/N150_maxAngErrStd.txt`' & '
}



{
echo "range, \$\\rho\$ &"
echo `cat results/N50_meanRanErrStd.txt`' / '`cat results/N50_maxRanErrStd.txt`' & '
echo `cat results/N100_meanRanErrStd.txt`' / '`cat results/N100_maxRanErrStd.txt`' & '
echo `cat results/N200_meanRanErrStd.txt`' / '`cat results/N200_maxRanErrStd.txt`' & '
echo `cat results/N500_meanRanErrStd.txt`' / '`cat results/N500_maxRanErrStd.txt`' \\'
}



{
echo "range, \$\\rho\$ &"
echo `cat results/N50_meanRanErrMean.txt`' / '`cat results/N50_maxRanErrMean.txt`' & '
echo `cat results/N100_meanRanErrMean.txt`' / '`cat results/N100_maxRanErrMean.txt`' & '
echo `cat results/N200_meanRanErrMean.txt`' / '`cat results/N200_maxRanErrMean.txt`' & '
echo `cat results/N500_meanRanErrMean.txt`' / '`cat results/N500_maxRanErrMean.txt`' \\'
}




{
echo "angle, \$\\theta\$ &"
echo `cat results/N50_meanAngErrStd.txt`' / '`cat results/N50_maxAngErrStd.txt`' & '
echo `cat results/N100_meanAngErrStd.txt`' / '`cat results/N100_maxAngErrStd.txt`' & '
echo `cat results/N200_meanAngErrStd.txt`' / '`cat results/N200_maxAngErrStd.txt`' & '
echo `cat results/N500_meanAngErrStd.txt`' / '`cat results/N500_maxAngErrStd.txt`' \\'
}




#
