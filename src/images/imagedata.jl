
@info "Loading Caesar tools related to ImageMagick."

export fetchDataImage

"""
    $SIGNATURES

`Data: Entry => Blob` helper function to load images stored in standard (png, jpg, jpeg) format from supported a DFG data blob store.

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
  checkMimeType && (@assert imgEntry.mimeType in ["image/png"; "image/jpg"; "image/jpeg"] "Unknown image format DataBlobEntry.mimeType=$(imgEntry.mimeType)")
  ImageMagick.readblob(imgBytes)
end
fetchDataImage(dfg::AbstractDFG,datastore::AbstractBlobStore,varLbl::Symbol,dataLbl::Symbol,checkMimeType::Bool=true) = fetchDataImage(dfg, varLbl, dataLbl, (g,vl,dl) -> getData(g,datastore,vl,dl) , checkMimeType)



#