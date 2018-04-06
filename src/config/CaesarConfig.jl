export CaesarConfig, VisualizationConfig

struct VisualizationConfig
    drawDepth::Bool
    cleanSlamInDb::Bool
    drawEdges::Bool
end

struct CaesarConfig
    numParticles::Int64
    multiSession::Vector{AbstractString}
    visualizationConfig::VisualizationConfig
end
