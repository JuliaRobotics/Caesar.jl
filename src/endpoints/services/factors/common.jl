function _evalType(pt::String)::Type
    try
        getfield(Main, Symbol(pt))
    catch ex
        io = IOBuffer()
        showerror(io, ex, catch_backtrace())
        err = String(take!(io))
        error("_evalType: Unable to locate factor type '$pt' in main context. Please check that this factor type is loaded into main. Stack trace = $err")
    end
end
