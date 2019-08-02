using Amphitheatre
using Test

@testset "Amphitheatre.jl" begin

    @testset "Examples" begin
        @info "Testing if examples run"
        include("../example/extendingAmphi.jl")
    end
end
