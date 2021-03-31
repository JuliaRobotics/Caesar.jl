
using ArgParse
function parse_commandline()
  s = ArgParseSettings()

  @add_arg_table! s begin
    "--spreadNH_start"
      help = "spreadNH_start"
      arg_type = Float64
      default = 3.0
    "--spreadNH_stop"
      help = "spreadNH_stop"
      arg_type = Float64
      default = 3.0
    "--spreadNH_step"
      help = "spreadNH_step"
      arg_type = Float64
      default = 1.0
    "--inflation_start"
      help = "inflation_start"
      arg_type = Float64
      default = 5.0
    "--inflation_stop"
      help = "inflation_stop"
      arg_type = Float64
      default = 5.0
    "--inflation_step"
      help = "inflation_step"
      arg_type = Float64
      default = 1.0
    "--useMsgLikelihoods"
      help = "useMsgLikelihoods"
      arg_type = Bool
      default = true
    "--anneal"
      help = "anneal the solve value according to param settings iterations"
      action = :store_true
    "fgtargz"
      help = "fgtargz, but leave out the extension .tar.gz"
      required = true
  end

  return parse_args(s)
end

using RoME
using Distributed
addprocs(10)
using RoME
@everywhere using RoME


function main()
  pargs = parse_commandline()
  println("Parsed args:")
  for (arg,val) in pargs
      println("  $arg  =>  $val")
  end

  fname	= replace(pargs["fgtargz"],".tar.gz"=>"")
  fg = loadDFG(fname);

  getSolverParams(fg).useMsgLikelihoods = pargs["useMsgLikelihoods"]
  
  SNH = pargs["spreadNH_start"]:pargs["spreadNH_step"]:pargs["spreadNH_stop"]
  IFL = pargs["inflation_start"]:pargs["inflation_step"]:pargs["inflation_stop"]

  if pargs["anneal"]
    for i in 1:length(SNH)
      getSolverParams(fg).spreadNH = SNH[i]
      getSolverParams(fg).inflation = IFL[i]
      # solve
      solveTree!(fg, storeOld=true);      
    end
    # store the final result
    snhn = replace(string(SNH),'.'=>'_')
    ifln = replace(string(IFL),'.'=>'_')
    saveDFG(pargs["fgtargz"]*"S$snhn"*"I$ifln", fg)
  else
    for snh in SNH, ifl in IFL
      getSolverParams(fg).spreadNH = snh
      getSolverParams(fg).inflation = ifl
      fg_ = deepcopy(fg)
      solveTree!(fg_, storeOld=true);
      # save the result
      snhn = replace("$snh",'.'=>'_')
      ifln = replace("$ifl",'.'=>'_')
      saveDFG(pargs["fgtargz"]*"S$snhn"*"I$ifln", fg_)
    end
  end

  @info "done"
end

main()

#
