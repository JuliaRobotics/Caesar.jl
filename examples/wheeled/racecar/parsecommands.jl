using ArgParse

function parse_commandline()
    s = ArgParseSettings()

    @add_arg_table s begin
        "--folder_name"
            help = "The name of the folder containing the images and other data"
            arg_type = String
            default = "labrun1"
        "--cx"
            help = "Center x offset"
            arg_type = Float64
            default = 327.986
        "--cy"
            help = "Center y offset"
            arg_type = Float64
            default = 198.066
        "--fx"
            help = "Focal x"
            arg_type = Float64
            default = 349.982
        "--fy"
            help = "Focal y"
            arg_type = Float64
            default = 349.982
        "--batch_size"
            help = "Batch solve every n poses"
            arg_type = Int64
            default = 20

    end

    return parse_args(s)
end

parsed_args = parse_commandline()

global folderName, cx, cy, fx, fy, batchSize

for (arg, val) in parsed_args
    if arg == "folder_name"
        folderName = val
    end
    if arg == "cx"
        cx = val
    end
    if arg == "cy"
        cy = val
    end
    if arg == "fx"
        fx = val
    end
    if arg == "fy"
        fy = val
    end
    if arg == "batch_size"
        batchSize = val

    end
end
