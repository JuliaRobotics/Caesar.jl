
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

@test haskey(fd, :a)
@test 1 == length(fd.keydict)
@test fd.keydict[:a] isa UUID
@test 1 == length(fd.pqueue)
@test 1 == length(fd.cache)
@test 1 == fd.cache[:a]
@test 1 == fd[:a] # all up test for getindex when key in cache

fd[:b] = 2

@test haskey(fd, :b)
@test 2 == length(fd.keydict)
@test fd.keydict[:a] != fd.keydict[:b]
@test 2 == length(fd.pqueue)
@test 2 == length(fd.cache)
@test 2 == fd.cache[:b]
@test 2 == fd[:b] # all up test for getindex when key in cache

fd[:c] = 3

@test haskey(fd, :c)
@test 3 == length(fd.keydict)
@test fd.keydict[:a] != fd.keydict[:c]
@test 2 == length(fd.pqueue)
@test 2 == length(fd.cache)
@test 3 == fd.cache[:c]
@test 3 == fd[:c] # all up test for getindex when key in cache

# make sure folder recovery works by fetching from all three keys, with cache_size set to 2
@test fd[:a] != fd[:b]
@test fd[:b] != fd[:c]

@show fd;

delete!(fd, :b)

# TODO check that the actual folder stored was deleted from permanent storage after `delete!( ,:b)`

@test 2 == length(fd.keydict)
@test fd.keydict[:a] != fd.keydict[:c]
@test 2 == length(fd.pqueue)
@test 2 == length(fd.cache)
@test 3 == fd.cache[:c]
@test 3 == fd[:c] # all up test for getindex when key in cache

@test_throws KeyError fd[:b]

@test 2 == length(intersect([:a; :c], collect(keys(fd))))
@test !haskey(fd, :b)
@test haskey(fd, :a)
@test haskey(fd, :c)


fd_copy = deepcopy(fd)

@show fd_copy;

@test !haskey(fd_copy, :b)
@test haskey(fd_copy, :a)
@test haskey(fd_copy, :c)

# make sure folder recovery works by fetching from all three keys, with cache_size set to 2
@test fd_copy[:a] != fd_copy[:c]

# make sure folder recovery works by fetching from all three keys, with cache_size set to 2
@test fd[:a] == fd_copy[:a]
@test fd[:c] == fd_copy[:c]

##
end