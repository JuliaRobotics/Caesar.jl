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
            arg_type = Float32
            default = 327.986
        "--cy"
            help = "Center y offset"
            arg_type = Float32
            default = 198.066
        "--fx"
            help = "Focal x"
            arg_type = Float32
            default = 349.982
        "--fy"
            help = "Focal y"
            arg_type = Float32
            default = 349.982
        "--batch_size"
            help = "Batch solve every n poses"
            arg_type = Int32
            default = 20

    end

    return parse_args(s)
end

parsed_args = parse_commandline()
for (arg, val) in parsed_args:
    if arg == "folder_name"
        folderName = val
    if arg == "cx"
        cx = val
    if arg == "cy"
        cy = val
    if arg == "fx"
        fx = val
    if arg == "fy"
        fy = val

    end
end
