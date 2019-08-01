
# Utils for every script to load kayak experimental data

function getPaths(expID::String , expType::String ; currloc::String="local" , trialID::Int=-1)
    if currloc == "local"
        topdir = joinpath(ENV["HOME"],"data", "kayaks")
    elseif currloc == "lj"
        topdir = joinpath("/media","data1","data","kayaks")
    end

    if expType == "range"
        expstr = "rangeOnly";
    elseif expType  == "dynsas"
        expstr = expType
    elseif expType == "pointsas"
        expstr = expType
    elseif exptType == "dynsource"
        expstr = expType
    end

    if expID == "dock"
        trialstr = "20_gps_pos";
        rangedir = joinpath(topdir, expstr*"_"*trialstr)
        datadir = joinpath(topdir,trialstr)
        savedir = joinpath(topdir,expstr)
        gtjldpath = ""
    elseif expID == "drift"
        trialstr = "08_10_parsed";
        datadir = joinpath(topdir,trialstr)
        rangedir = joinpath(topdir, expstr*trialstr)
        savedir = joinpath(topdir,expstr)
        if trialID == -1
            error("Specify Trial ID: 1-3")
        else
            gtjldpath = joinpath(datadir,"exp$(trialID).jld")
        end
    end

    return datadir, rangedir, savedir, gtjldpath
end

function loadNav(wIn::Array, allpaths)
    thisIter = 1:wIn[end]-wIn[1]
    navalltemp = zeros(length(thisIter),2);
    for i in thisIter
        navfile = joinpath(allpaths[1],"nav$(wIn[i]).csv")
        navalltemp[i,:] = readdlm(navfile,',',Float64,'\n')
    end
    return navalltemp
end

function loadRanges(wIn::Array, allpaths)
    thisIter = 1:wIn[end]-wIn[1]
    rangealltemp = zeros(7501,2,length(thisIter));
    for i in thisIter
        rfile = joinpath(allpaths[2],"range$(wIn[i]).txt")
        rangealltemp[:,:,i] = readdlm(rfile,',',Float64,'\n')
    end
    return rangealltemp
end

function loadGT(wIn::Array, expID, allpaths)
    if expID == "dock"
        igttemp = [17.0499;1.7832];
    elseif expID == "drift"
        tmpdict = load(allpaths[4])
        igtFull = tmpdict["icarus_gt"]
        windowstart = tmpdict["ibegin"];
        windowend = tmpdict["iend"];

        if length(wIn) > (windowend - windowstart)
            error("Specified trial window is too long")
        else
            igttemp = igtFull[wIn[1]-windowstart:length(wIn),:]
        end
    end
    return igttemp
end
