
using Pkg
using Caesar


## Read all test lines from file
fid = open("/tmp/caesar/dbgCSMHexDelayMatrix.log","r")
lines = readlines(fid)
close(fid)

## split the lines into usable parts
sstr = split.(lines, r" -- ")
resultFolder = String[]
passedStr = String[]
injDelayStr = String[]
for x in sstr
  pic = split(x[2], r", ")
  push!(passedStr, pic[1])
  push!(injDelayStr, pic[2])
  push!(resultFolder, x[1])
end

## which tests were true and false
maskTrue = passedStr .|> px->split(px, '=')[2] == "true"
maskFalse = passedStr .|> px->split(px, '=')[2] == "false"
@assert sum(maskTrue) + sum(maskFalse) == length(lines) "True/False masks must reflect all lines"


# see latest IIF for functions


## True case

csmCounterTrue, trxTrue = calcCSMOccurancesFolders(resultFolder[maskTrue])
maxOccuranTrue = calcCSMOccuranceMax(csmCounterTrue)

open("/tmp/caesar/maxOccTrue.txt","w") do io
  printCSMOccuranceMax(maxOccuranTrue, fid=io)
end


## False case

csmCounterFalse, trxFalse = calcCSMOccurancesFolders(resultFolder[maskFalse])
maxOccuranFalse = calcCSMOccuranceMax(csmCounterFalse)

open("/tmp/caesar/maxOccFalse.txt","w") do io
  printCSMOccuranceMax(maxOccuranFalse, fid=io)
end




using JSON

fst = 

open("/tmp/caesar/trxTrue.json", "w") do io
  println(io, JSON.json(trxTrue))
end

open("/tmp/caesar/trxFalse.json", "w") do io
  println(io, JSON.json(trxFalse))
end

#