
## test 
using Test

using Caesar
import Caesar: FolderStore



@testset "Basic functional tests of FolderDict" begin
##

fd = FolderDict{Symbol, Int}(;cache_size=2)

@show fd.wdir

fd[:a] = 1

@test 1 == length(fd.keydict)
@test fd.keydict[:a] isa UUID
@test 1 == length(fd.pqueue)
@test 1 == length(fd.cache)
@test 1 == fd.cache[:a]
@test 1 == fd[:a] # all up test for getindex when key in cache



##
end