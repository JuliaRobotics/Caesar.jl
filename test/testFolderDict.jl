
## test 
using Test
using UUIDs
using Caesar
import Caesar: FolderDict

##

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

fd[:b] = 2

@test 2 == length(fd.keydict)
@test fd.keydict[:a] != fd.keydict[:b]
@test 2 == length(fd.pqueue)
@test 2 == length(fd.cache)
@test 2 == fd.cache[:b]
@test 2 == fd[:b] # all up test for getindex when key in cache

fd[:c] = 3

@test 3 == length(fd.keydict)
@test fd.keydict[:a] != fd.keydict[:c]
@test 2 == length(fd.pqueue)
@test 2 == length(fd.cache)
@test 3 == fd.cache[:c]
@test 3 == fd[:c] # all up test for getindex when key in cache


delete!(fd, :b)

@test 2 == length(fd.keydict)
@test fd.keydict[:a] != fd.keydict[:c]
@test 2 == length(fd.pqueue)
@test 2 == length(fd.cache)
@test 3 == fd.cache[:c]
@test 3 == fd[:c] # all up test for getindex when key in cache

@test_throws KeyError fd[:b]


@test 2 == length(intersect([:a; :c], collect(keys(fd))))

##
end