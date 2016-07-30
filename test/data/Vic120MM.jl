#MM dictionary

function MMwithRedirects(mm::Dict{Int64,Int64})
  mmr = Dict{Int64,Int64}()
  for m in mm
    finID = m[2]
    while true
      if haskey(mm, finID)
        finID = mm[finID]
      else
        break;
      end
    end
    mmr[m[1]] = deepcopy(finID)
  end
  return mmr
end

# point to the first instance in the map which may be the same feature
MM = Dict{Int64,Int64}()

MM[103] = 2
MM[273] = 2
MM[105] = 3
MM[274] = 3
MM[106] = 5
MM[270] = 5
MM[111] = 6
MM[275] = 6
MM[281] = 9
# early tests
MM[265] = 1
  MM[286] = 11 # driving data looks convincing
MM[279] = 18
      MM[284] = 20
MM[290] = 22
MM[291] = 23
MM[294] = 26 # repeated with different id as false association
MM[88] = 27
MM[285] = 27
MM[299] = 34
MM[317] = 34
MM[298] = 36
MM[299] = 43
MM[317] = 43

MM[289] = 48
      MM[310] = 54
MM[302] = 56
MM[308] = 59
MM[304] = 60
MM[301] = 61
  MM[287] = 65 # maybe 61, maybe more than one tree
      MM[334] = 68
MM[96] = 75
      MM[355] = 83
MM[107] = 86
MM[323] = 86
      MM[307] = 90
        MM[322] = 92
        MM[347] = 97
MM[261] = 116
      MM[267] = 121
MM[269] = 125
MM[249] = 126
MM[257] = 129
MM[196] = 132
  MM[251] = 132
MM[199] = 134
MM[252] = 134
MM[200] = 138
MM[258] = 138
  MM[182] = 139
  MM[253] = 139 #182
MM[208] = 139
  MM[247] = 144
MM[242] = 145
MM[240] = 149
MM[235] = 158
MM[238] = 164
MM[216] = 165
      MM[225] = 165
MM[227] = 167
  MM[209] = 197
    MM[201] = 193
MM[230] = 219
  MM[239] = 219
MM[226] = 220
#maybes
MM[244] = 135
MM[255] = 135
  MM[309] = 288
      MM[348] = 90#307
  MM[349] = 59# 337

# Beyond 140s
MM[408] = 6
MM[437] = 5
MM[438] = 3
MM[434] = 2
MM[409] = 22
MM[451] = 1
MM[411] = 55
MM[417] = 59
MM[416] = 56
  MM[412] = 20
  MM[409] = 22
  MM[418] = 44
  MM[415] = 48
    MM[378] = 23
      MM[454] = 125
      MM[459] = 121
      MM[457] = 116
      MM[466] = 126
        MM[330] = 91
        MM[472] = 135
        MM[474] = 139
        MM[478] = 144
          MM[605] = 387
          MM[620] = 395
          MM[601] = 381
          MM[606] = 377
            MM[495] = 20
            MM[575] = 22
            MM[553] = 45
            MM[571] = 23
            MM[570] = 26
            MM[568] = 43
              MM[481] = 254
              MM[480] = 167
              MM[231] = 167
              MM[419] = 27
              MM[447] = 9
              MM[421] = 18
              MM[373] = 59
              MM[550] = 90
              MM[560] = 90
              MM[535] = 83
              MM[512] = 99
  # laser to back axel lever arm not modeled in tracker and sighting constraints
  MM[748] = 629
  MM[755] = 631
  MM[741] = 633
  MM[739] = 654
  MM[744] = 654
    MM[761] = 622
    MM[765] = 368#626
    MM[770] = 647
    MM[769] = 653
    MM[771] = 664
# matches at 330s data
MM[828] = 48
MM[823] = 36
MM[817] = 56
MM[801] = 59
MM[835] = 22
MM[836] = 23
MM[831] = 6
MM[838] = 5
MM[846] = 3
MM[845] = 2
MM[832] = 18
MM[834] = 27
MM[826] = 44
MM[816] = 45
MM[818] = 51
MM[803] = 55
MM[847] = 11
MM[830] = 20

#Matches for 350
MM[855] = 125
MM[859] = 121
MM[861] = 116
MM[870] = 138
MM[865] = 126
  MM[867] = 132
MM[872] = 139
MM[883] = 189
MM[876] = 144
MM[882] = 193
MM[871] = 135
MM[879] = 162
MM[884] = 164
MM[877] = 145
MM[889] = 165
MM[881] = 149
  MM[880] = 197
    MM[852] = 1
    MM[863] = 464
      MM[445] = 11
    MM[840] = 26
      MM[504] = 6
      MM[497] = 5
      MM[868] = 467
      MM[486] = 467
    MM[869] = 129 #473
      MM[515] = 85
    MM[780] = 83
      MM[527] = 58
  MM[763] = 372
  MM[793] = 387
    MM[796] = 377
        MM[376] = 59
        MM[337] = 59
          MM[799] = 59
        MM[414] = 36
        MM[499] = 93
          MM[588] = 51
          MM[599] = 56
          MM[374] = 26
          MM[573] = 36
            MM[566] = 36#48
          MM[600] = 48
        MM[594] = 125
        MM[582] = 23
            MM[483] = 61
          # MM[556] = 45
      MM[875] = 134

  MM[848] = 597
  MM[588] = 51
      MM[502] = 463
      MM[562] = 75
        MM[513] = 27
        MM[490] = 27
          MM[470] = 134
          MM[468] = 132
      MM[469] = 138
      MM[473] = 129
    MM[887] = 167
    MM[890] = 167
    MM[886] = 158

      MM[404] = 26
      MM[406] = 26

# more risky associations
MM[297] = 44
MM[592] = 11
MM[589] = 6
MM[580] = 5
MM[627] = 397
MM[632] = 396
MM[617] = 391
MM[767] = 391
MM[613] = 372
  MM[862] = 85
    MM[558] = 54
  MM[496] = 1
  MM[487] = 1
    MM[494] = 48
    MM[574] = 48
    MM[567] = 48
    MM[525] = 48

    MM[488] = 2
    MM[503] = 2
      MM[538] = 43
      MM[533] = 44
        MM[569] = 44
      MM[525] = 48
        MM[556] = 56
      MM[484] = 91
      MM[551] = 82
        MM[562] = 75
        MM[563] = 59

  MM[630] = 371
  MM[626] = 368
    MM[577] = 20



# new for larger T=1400 map, submaps [60;1970;1490], spread=70
MM[3022] = 1489
MM[3021] = 1483
MM[3015] = 1484
MM[3024] = 1493
MM[3019] = 1497
MM[3013] = 1488
MM[3014] = 1494
MM[2995] = 1496
MM[3007] = 1509
MM[2987] = 6 # 1506
MM[2982] = 5 # 1502
MM[2979] = 3 # 1507
MM[2978] = 2 # 1508

MM[1905] = 6
MM[1913] = 5
MM[1911] = 3
MM[1912] = 2
MM[1956] = 2
#triplets
MM[1940] = 1438
MM[1487] = 1438
MM[1935] = 1434
MM[1935] = 1483
MM[1937] = 1436
MM[1489] = 1436
# MM[1929] = 143x
MM[1938] = 1435
MM[1493] = 1435
MM[1927] = 1428
MM[1488] = 1428
MM[1926] = 1427
MM[1503] = 1427
MM[1918] = 1509
MM[1921] = 1420
MM[1496] = 1420
MM[1920] = 1422
MM[1950] = 1494
MM[1946] = 1494
MM[1928] = 1494

MM[1506] = 6
MM[1502] = 5
MM[1507] = 3
MM[1508] = 2

## post 11AM
# from submaps [80;1490], spread=80
MM[1433] = 138
MM[1497] = 138
MM[1418] = 125
MM[1509] = 125
MM[1488] = 126
MM[1428] = 126
MM[1435] = 134
MM[1493] = 134
MM[1434] = 135
MM[1483] = 135
MM[1436] = 132
MM[1489] = 132
MM[1490] = 139
MM[1420] = 116
MM[1496] = 116
MM[1422] = 121

# from submaps [80;950], spread=80
MM[985] = 6
MM[972] = 5
MM[971] = 3
MM[970] = 2

MM[936] = 116
MM[995] = 125
MM[929] = 144
MM[924] = 139 # might be 923
MM[925] = 132
MM[932] = 138
MM[928] = 126
MM[927] = 135
MM[926] = 134
MM[931] = 145
MM[935] = 149

MM[1019] = 20
MM[1018] = 22

# submaps from [3080;2550], spread=100
# upper left corner loop closures
MM[3176] = 2645
MM[3164] = 2628
MM[3180] = 2644
MM[3169] = 2632
MM[3168] = 2638
MM[3176] = 2649 #?
MM[3167] = 2633
MM[3162] = 2625
MM[3170] = 2636
MM[2646] = 2636
MM[3179] = 2629
MM[2647] = 2629
MM[3158] = 2624
MM[3150] = 2613
MM[3141] = 2610
MM[3139] = 2608
MM[3153] = 2614
MM[3137] = 2606


## post 3:20PM, from submaps [100;2550;3080], spread=100
MM[2463] = 138
MM[2455] = 126
MM[2466] = 134
MM[2456] = 132
MM[2464] = 135
MM[2467] = 189#139
MM[2477] = 139# something wrong with (not found) 187
MM[2469] = 144
MM[2480] = 164
MM[2471] = 162
MM[2489] = 165
MM[2481] = 167


## from submaps [100;2900], spread=100
MM[2852] = 138
MM[2849] = 126
MM[2847] = 134
MM[2837] = 132
MM[2840] = 135
MM[2820] = 139
MM[2838] = 189
MM[2813] = 144
MM[2824] = 164
MM[2809] = 162
MM[2801] = 193
MM[2816] = 165
MM[2863] = 125
MM[2866] = 121
MM[2853] = 116
MM[2874] = 6
MM[2864] = 5
MM[2869] = 3
MM[2868] = 2

MM[2892] = 48
MM[2899] = 36
MM[2896] = 45
MM[2889] = 26


## post 5:30 solve, from submaps [700; 1200], spread=100
MM[1243] = 737
MM[1239] = 722
MM[1230] = 717
MM[1247] = 732
MM[1246] = 725
MM[1227] = 707
MM[1225] = 702
MM[1255] = 702
MM[1250] = 702

MM[1202] = 670
MM[1300] = 758
MM[1194] = 684
MM[1295] = 751
MM[1275] = 695

MM[1220] = 700
MM[1267] = 1220

MM[1211] = 691
MM[1260] = 1211
MM[1269] = 1208
MM[1273] = 1216

# from submaps [700;1800], spread=100
MM[1789] = 737
MM[1780] = 722
MM[1770] = 717
MM[1788] = 732
MM[1769] = 707
MM[1767] = 702
MM[1773] = 715
MM[1741] = 698
MM[1848] = 786
MM[1831] = 754
MM[1791] = 654
MM[1792] = 633
MM[1797] = 746
MM[1731] = 686
MM[1793] = 742
MM[1740] = 687
MM[1863] = 747
MM[1833] = 658

# from submaps [100;650;2300], spread=100
MM[2262] = 737
MM[2260] = 722
MM[2266] = 717
MM[2263] = 732
MM[2269] = 707

MM[2300] = 658
MM[747] = 658
MM[2303] = 629
MM[2306] = 631
MM[2297] = 633
MM[2277] = 654
MM[2295] = 746
MM[2278] = 742
MM[2296] = 743
MM[742] = 686
MM[743] = 676

# from submaps [100; 2950]
MM[2990] = 116
MM[2989] = 20
MM[2984] = 18
MM[2966] = 9
MM[2964] = 27
MM[2946] = 86
MM[2945] = 84

MM[2964] = 27
MM[2908] = 43
MM[2909] = 54

MM[2910] = 82
MM[2962] = 75

MM[2913] = 55

## after 8:50 PM solve
MM[3027] = 144
MM[3036] = 164
MM[3028] = 162

MM[2879] = 11
MM[2878] = 22
MM[2902] = 56
MM[2905] = 59
MM[2897] = 44
MM[2880] = 20
MM[2885] = 18
MM[2891] = 27
MM[2877] = 9

# more risky, submaps [650, 1120, 2300], spread=100
MM[2369] = 1147
MM[2310] = 1138
MM[2364] = 1137
MM[2326] = 1163
MM[2316] = 1141
MM[2319] = 1152

MM[1153] = 658
MM[1154] = 629
MM[1157] = 631

MM[1141] = 622
MM[1163] = 647

MM[2327] = 664
MM[2320] = 653

# Thu morn, from submaps [650;2300;1750], spread=100
MM[2308] = 652
MM[1734] = 683

MM[1727] = 676
MM[1795] = 676

MM[2314] = 1812
MM[1804] = 631
MM[1685] = 631
MM[1684] = 633
MM[1812] = 670
MM[2313] = 1732
MM[1805] = 1732
MM[1732] = 684
MM[1818] = 1734

MM[1733] = 681

MM[1803] = 695


#from submaps [400;600;800], spread=100
MM[800] = 381
MM[757] = 652
MM[788] = 652
MM[758] = 670

#from submaps [400;600;2100]
# from submaps [650;2150], spread=100
MM[2094] = 658
MM[2098] = 629
MM[2109] = 654
MM[2092] = 633

MM[2132] = 687
MM[2170] = 686
MM[2128] = 676
MM[2136] = 684
MM[2127] = 670
MM[2137] = 683

MM[2086] = 627


# Saturday, fromSubmaps=[1130,1350], spread=100
MM[1340] = 1134
MM[1321] = 1138
MM[1318] = 1137
MM[1341] = 1133
MM[1403] = 1120
MM[1388] = 1111
MM[1405] = 1113
MM[1384] = 1096
MM[1379] = 1088
MM[1382] = 1105
MM[1398] = 1098
MM[1392] = 1102



# Unsure associations
# MM[923] = 139 # ref 924
# MM[990] = 18
# MM[1298] = 654
# MM[1153] = 754
# MM[2934] = 68
# MM[2147] = 694


# purposeful bad associates, multimodal constraints
# MM[232] = 167
# MM[225] = 167
# MM[260] = 126
# MM[121] = 5
# MM[314] = 109
# MM[294] = 45 #  repeat of 26
# MM[344] = 44
# # slalums
# MM[284] = 105
# MM[556] = 45

#outrageous
# MM[210] = 100
# MM[2658] = 706
# MM[2616] = 693
# MM[2755] = 664
# MM[2720] = 773
# MM[2796] = 715
# MM[2607] = 633
# MM[2778] = 600

# randomly permute some fraction of initially correct loop closures
function MMRandErrs(MM::Dict{Int64,Int64};frac=0.1)
  MMr = Dict{Int64, Int64}()

  allkeys = collect(keys(MM))
  len = length(allkeys)
  @show clen = round(Int, frac*len)
  @show clen2 = round(Int, 0.5*clen)
  p1 = Dict{Int,Void}()
  p2 = Dict{Int,Void}()

  for m in MM    MMr[m[1]] = m[2]; end

  while length(p1) < clen2 && length(p2) < clen2
    prop1 = allkeys[rand(1:len)]
    if !haskey(p1, prop1) && !haskey(p2, prop1)
      for i in 1:100
        prop2 = allkeys[rand(1:len)]
        # want to try MMr[prop1] = MMr[prop2] & MMr[prop2] = MMr[prop1]
        mm1 = MMr[prop1]
        mm2 = MMr[prop2]
        # @show !haskey(p2, prop2), !haskey(p1, prop2), prop1 > mm2, prop2 > mm1
        if !haskey(p2, prop2) && !haskey(p1, prop2) && prop1 > mm2 && prop2 > mm1
          p1[prop1] = nothing
          p2[prop2] = nothing
          MMr[prop1] = mm2
          MMr[prop2] = mm1
          break
        end
      end
    end
  end

  # for i in 1:(clen2-1)
  #   temp = MMr[p[clen2+i]]
  #   MMr[p1[clen2+i]] = MMr[p1[i]]
  #   MMr[p1[i]] = temp
  # end
  return MMr
end

function showMMpermutes(MMr, MMe)
  for m in MMr  if MMe[m[1]] != m[2]   @show m[1], m[2], MMe[m[1]] end end
  nothing
end


function testMMRandErr(MM::Dict{Int64,Int64};frac::Float64=0.1)
  MMe = MMRandErrs(MM,frac=frac)
  fi = map(Int, [m[2] == MMe[m[1]] for m in MM])
  @show sum(fi.==0)/(sum(fi.==1)+0.0+sum(fi.==0))
end
























#
