using ArgParse

function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table s begin
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
        "--batch_size"
            help = "Batch solve every n poses"
            arg_type = Int64
            default = 20
    end

    return parse_args(s)
end

parsed_args = parse_commandline()

# global folderName, cx, cy, fx, fy, batchSize
#
# for (arg, val) in parsed_args
#     if arg == "folder_name"
#         folderName = val
#     end
#     if arg == "cx"
#         cx = val
#     end
#     if arg == "cy"
#         cy = val
#     end
#     if arg == "fx"
#         fx = val
#     end
#     if arg == "fy"
#         fy = val
#     end
#     if arg == "batch_size"
#         batchSize = val
#     end
# end
