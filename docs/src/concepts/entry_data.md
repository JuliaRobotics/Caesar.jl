# [Additional (Large) Data](@id section_data_entry_blob_store)

There are a variety of situations that require more data to be stored natively in the factor graph object.
This page will showcase some of `Entry=>Data` features available.


# Adding A FolderStore

Caesar.jl (with DFG) supports storage and retrieval of larger data blobs by means of various database/datastore technologies.  To get going, you can use a conventional `FolderStore`: 
```julia
getSolverParams(fg).logpath = pwd()
storeDir = joinLogPath(fg,"data")
mkpath(storeDir)
# requires IIF v0.15, DFG v0.10
datastore = FolderStore{Vector{UInt8}}(:default_folder_store, storeDir) 
addBlobStore!(fg, datastore)
```

!!! note
    This example places the `data` folder in the `.logpath` location which defaults to `/tmp/caesar/UNIQUEDATETIME`. This is not a long term storage location since `/tmp` is periodically cleared by the operating system. Note that the `data` folder can be used in combination with [loading and saving factor graph objects](https://juliarobotics.org/Caesar.jl/latest/concepts/interacting_fgs/#Saving-and-Loading).

## Adding Data Blobs

Just showcasing a JSON Dict approach
```julia
using JSON2
someDict = Dict(:name => "Jane", :data => randn(100))
addData!(fg, :default_folder_store, :x1, :datalabel, Vector{UInt8}(JSON2.write( someDict )), mimeType="application/json/octet-stream"  )
# see retrieval example below...
```

This approach allows the maximum flexibility, for example it is also possible to do:
```julia
# from https://juliaimages.org/stable/install/
using TestImages, Images, ImageView
img = testimage("mandrill")
imshow(img)

# TODO, convert to Vector{UInt8}
using ImageMagick, FileIO
# convert image to PNG bytestream
io = IOBuffer()
pngSm = Stream(format"PNG", io)
save(pngSm, img)  # think FileIO is required for this
pngBytes = take!(io)
addData!(fg, :default_folder_store, :x1, :testImage, pngBytes, mimeType="image/png", description="mandrill test image"  )
```

## [Retrieving a Data Blob](@id section_retrieve_data_blob)

Data is stored as an `Entry => Blob` relationship, and the entries associated with a variable can be found via
```julia
julia> listDataEntries(fg, :x6)
1-element Array{Symbol,1}:
 :JOYSTICK_CMD_VALS
 :testImage
```

And retrieved via:
```julia
rawData = getData(fg, :x6, :JOYSTICK_CMD_VALS);
imgEntry, imgBytes = getData(fg, :x1, :testImage)
```

Looking at `rawData` in a bit more detail:
```julia
julia> rawData[1]
BlobStoreEntry(:JOYSTICK_CMD_VALS, UUID("d21fc841-6214-4196-a396-b1d5ef95be49"), :default_folder_store, "deeb3ed0cba6ffd149298de21c361af26a207e565e27a3cd3fa6c807b9aaa44d", "DefaultUser|DefaultRobot|Session_851d81|x6", "", "application/json/octet-stream", TimeZones.ZonedDateTime(2020, 8, 15, 14, 26, 36, 397, tz"UTC-04:00"))

julia> rawData[2]
3362-element Array{UInt8,1}:
 0x5b
 0x5b
 0x32
#...
```

For `:testImage` the data was packed in a familiar `image/png` and can be converted backto bitmap (array) format:
```julia
rgb = ImageMagick.readblob(imgBytes); # automatically detected as PNG format

using ImageView
imshow(rgb)
```

In the other case where data was packed as `"application/json/octet-stream"`:
```julia
myData = JSON2.read(IOBuffer(rawData[2]))

# as example
julia> myData[1]
3-element Array{Any,1}:
                2017
 1532558043061497600
                    (buttons = Any[0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0], axis = Any[0, 0.25026196241378784, 0, 0, 0, 0])
```

### Quick Camera Calibration Storage Example

Consider storing camera calibration data inside the factor graph `tar.gz` object for later use:
```julia
fx = 341.4563903808594
fy = 341.4563903808594
cx = 329.19091796875
cy = 196.3658447265625

K = [-fx 0  cx;
      0 fy cy]

# Cheap way to include data as a Blob.  Also see the more hacky `Smalldata` alternative for situations that make sense.
camCalib = Dict(:size=>size(K), :vecK=>vec(K))
addData!(dfg,:default_folder_store,:x0,:camCalib,
         Vector{UInt8}(JSON2.write(camCalib)), mimeType="application/json/octet-stream", 
         description="reshape(camCalib[:vecK], camCalib[:size]...)") 
```


## Working with Binary Data (BSON)

Sometime it's useful to store binary data.  Let's combine the example of storing a Flux.jl Neural Network object using [the existing BSON approach](http://fluxml.ai/Flux.jl/stable/saving/#).  Also [see BSON wrangling snippets here](https://github.com/JuliaRobotics/IncrementalInference.jl/wiki/Coding-Templates#bson-iobuffer-and-base64).

!!! note
    We will store binary data as Base64 encoded string to avoid other framing problems.  See [Julia Docs on Base64](https://docs.julialang.org/en/v1/stdlib/Base64/#Base64.Base64EncodePipe)

```julia
# the object you wish to store as binary
model = Chain(Dense(5,2), Dense(2,3))

io = IOBuffer()

# using BSON
BSON.@save io model

# get base64 binary
mdlBytes = take!(io)

addData!(dfg,:default_folder_store,:x0,:nnModel,
         mdlBytes, mimeType="application/bson/octet-stream", 
         description="BSON.@load PipeBuffer(readBytes) model") 
```


## Experimental Features

Loading images is a relatively common task, hence a convenience function has been developed:
```@docs
Caesar.fetchDataImage
```
