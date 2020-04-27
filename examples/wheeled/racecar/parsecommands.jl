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
            default = 0.5
        "--pose_trigger_rotate"
            help = "Required rotation to trigger new pose."
            arg_type = Float64
            default = pi/4
        "--batch_size"
            help = "Batch solve every n poses"
            arg_type = Int64
            default = 20
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
            default = 9999999999
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
    end

    return parse_args(s)
end

parsed_args = parse_commandline()
