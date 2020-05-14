using ArgParse

function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table! s begin
        "--folder_name"
            help = "The name of the folder containing the images and other data"
            arg_type = String
            default = "labrun5"
        "--previous"
            help = "Compute within a previous results folder."
            arg_type = String
            default = ""
        "--jldfile"
            help = "Which JLD file to load from the previous results folder"
            arg_type = String
            default = ""
        "--failsafe"
            help = "Single process and recursive Bayes tree solving, slower by but more robust with better error stack traces."
            action = :store_true
        "--cxoffset"
            help = "Center x offset"
            arg_type = Float64
            default = 0.0
        "--cyoffset"
            help = "Center y offset"
            arg_type = Float64
            default = 0.0
        "--focalscale"
            help = "Focal distance scaling"
            arg_type = Float64
            default = 1.0
        "--tagscale"
            help = "Tag sized scaling"
            arg_type = Float64
            default = 1.0
        "--pose_trigger_distance"
            help = "Required distance travelled to trigger new pose."
            arg_type = Float64
            default = 0.4
        "--pose_trigger_rotate"
            help = "Required rotation to trigger new pose."
            arg_type = Float64
            default = pi/4
        "--naive_frac"
            help = "Fraction of particles to use naive odo model."
            arg_type = Float64
            default = 0.6
        "--batch_size"
            help = "Batch solve every n poses"
            arg_type = Int64
            default = 20
        "--epochsFlux"
            help = "How many epochs to induce during Flux training."
            arg_type = Int64
            default = 15
        "--fluxGenerations"
            help = "How many times to repeat the Flux training cycles."
            arg_type = Int64
            default = 5
        "--rndSkip"
            help = "Flux training number of interpose starting points to skip; can/should be used in combination with rndChords.  -1 is randomized"
            arg_type = Int64
            default = -1
        "--rndChord"
            help = "Flux training number of interpose chords.  Int[] is randomized"
            arg_type = String
            default = "Int[]"
        "--fgpathsflux"
            help = "Point to a text file containing which results (savedfg.tar.gz destinations) to use for training."
            arg_type = String
            default = ""
        "--numFGDatasets"
            help = "How many fgpathsflux datasets to include in FG for Flux training; default -1 is use all available."
            arg_type = Int64
            default = -1
        "--localprocs"
            help = "How many local Julia processes to use"
            arg_type = Int64
            default = 4
        "--remoteprocs"
            help = "How many remote Julia processes to use"
            arg_type = Int64
            default = 0
        "--remoteserver"
            help = "Which remote server to use for processing"
            arg_type = String
            default = "JL_CLUSTER_HY"
        "--iterposes"
            help = "maximum number of poses to process"
            arg_type = Int64
            default = Int64(2)^63-Int64(1)
        "--msgloops"
            help = "Maximum number or messages to consume in the main loop (ROS/LCM/etc)"
            arg_type = Int64
            default = Int64(2)^63-Int64(1)
        "--show"
            help = "Show pdfs during computation using `evince`"
            action = :store_true
        "--vis2d"
            help = "Draw 2D visualizations"
            action = :store_true
        "--vis3d"
            help = "Draw 3D visualizations"
            action = :store_true
        "--imshow"
            help = "Show camera visualization"
            action = :store_true
        "--batch_resolve"
            help = "Resolve the entire factor graph at the end"
            action = :store_true
        "--report_factors"
            help = "Resolve the entire factor graph at the end"
            action = :store_true
        "--savedfg"
            help = "Store DFG objects to file"
            action = :store_true
        "--usesimmodels"
            help = "Use sim trained models"
            action = :store_true
    end

    return parse_args(s)
end

parsed_args = parse_commandline()

# convert from strings to types
parsed_args["rndChord"] = eval(Meta.parse(parsed_args["rndChord"]))
