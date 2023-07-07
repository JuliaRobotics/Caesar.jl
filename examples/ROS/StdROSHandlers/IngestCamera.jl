
"""
$SIGNATURES

Message callback for camera images.
"""
function handleCameraCompressed!(
    msg::sensor_msgs.msg.CompressedImage, 
    dfg, 
    state::SystemState, 
    options=Dict()
)

    lbl = "IMG_CENTER_$(state.cur_variable)_$(msg.header.seq)"
    @info "handleCameraCompressed!" lbl maxlog=10

    entries = Tuple{String, String}[]

    prvLbl = state.prv_variable === "" ? "x0" : state.prv_variable
    if state.pushBlobs
        res = addData(dfg, lbl, msg.data)
        # not super well synched, but good enough for basic demonstration
        addBlobEntry!(dfg, prvLbl, res, lbl, "image/jpeg")
        #   description="ImageMagick.readblob(imgBytes)"
        push!(entries, (lbl, res))
    end

    # add the data entries to the local factor graph too
    if state.pushBlobs && state.pushLocal
        vari = Caesar.getVariable(state.localfg, Symbol(prvLbl))
        timestamp = now(localzone())
        for (lb,id) in entries
            fde = Caesar.BlobStoreEntry(
                Symbol(lb),
                UUID(id),
                :nva,
                "",
                "", # $(userId)|$(robotId)|$(sessionId)|$(prvLbl)",
                "",
                "image/jpeg",
                now(localzone())
            )
            de = addBlobEntry!(vari, fde)
        end
    end

    nothing
end
