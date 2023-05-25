
@info "Caesar.jl is loading tools using ImageMagick.jl."

export toFormat
export fetchDataImage


"""
    $SIGNATURES

Convert image to PNG bytestream.

Notes
- `using ImageMagick, FileIO`

DevNotes
- TODO somehow consolidate with `MIME"image/png"`.

See also: [`makeImage`](@ref)
"""
function toFormat(
  format::Type{format"PNG"},
  img::AbstractMatrix{<:Colorant}
)
  #
  io = IOBuffer()
  pngSm = Stream(format, io)
  save(pngSm, img)  # think FileIO is required for this
  take!(io)
end

@deprecate toFormat(img::AbstractMatrix{<:Colorant}) toFormat(format"PNG", img)


## Using default FileIO together with ImageIO 
# function DistributedFactorGraphs.packBlob(
#   fmt::Type{format"PNG"}, 
#   img::Union{<:AbstractMatrix{<:Colorant},<:AbstractMatrix{UInt8}}
# )
#   blob = toFormat(fmt, img)
#   mimetype = "image/png"
#   return blob, mimetype
# end
# function DistributedFactorGraphs.unpackBlob(
#   ::Type{format"PNG"}, 
#   blob::Vector{UInt8}
# )
#   io = IOBuffer(blob)
#   ioStr = Stream{format"PNG"}(io)
#   return load(ioStr)
#   # ImageMagick.readblob(imgBytes)
# end

"""
    $SIGNATURES

`Data: Entry => Blob` helper function to load images stored in standard (png, jpg, jpeg) format from supported a DFG data blob store.

Example
```julia
# Skip if AbstractBlobStore is already set up
  ## if a FolderStore already exists (assumed inside `getLogPath(fg)`)
  storeDir = joinLogPath(fg,"data")
  datastore = FolderStore{Vector{UInt8}}(:default_folder_store, storeDir) 
  addBlobStore!(fg, datastore)

# Fetch the image
img = fetchDataImage(fg, :x4, :KEYFRAME_IMG)
```

Notes
- https://juliarobotics.org/Caesar.jl/latest/concepts/interacting_fgs/#Retrieving-a-Data-Blob
"""
function fetchDataImage(dfg::AbstractDFG,
                        varLbl::Symbol,
                        dataLbl::Symbol,
                        getDataLambda::Function = (g,vl,dl) -> getData(g,vl,dl),
                        checkMimeType::Bool=true )
#
  imgEntry, imgBytes = getDataLambda(dfg, varLbl, dataLbl)
  allowList = ["image/png"; "image/jpg"; "image/jpeg"]
  checkMimeType && (@assert imgEntry.mimeType in allowList "Should we allow image format DataBlobEntry.mimeType=$(imgEntry.mimeType), current allowList=$allowList")
  ImageMagick.readblob(imgBytes)
end
fetchDataImage(dfg::AbstractDFG,datastore::AbstractBlobStore,varLbl::Symbol,dataLbl::Symbol,checkMimeType::Bool=true) = fetchDataImage(dfg, varLbl, dataLbl, (g,vl,dl) -> getData(g,datastore,vl,dl) , checkMimeType)



#