using MetaGraphsNext
using Graphs
using Clustering
using StaticArrays
using TensorCast
using Distances

# References:
# [SZ 2003]: Sivic and Zisserman, 2003, October. Video Google: A text retrieval approach to object matching in videos. In Proceedings ninth IEEE international conference on computer vision (pp. 1470-1477). IEEE.
# [Wang 2011] Wang, X., Yang, M., Cour, T., Zhu, S., Yu, K., & Han, T. X. (2011, November). Contextual weighting for vocabulary tree based image retrieval. In 2011 International conference on computer vision (pp. 209-216). IEEE.
# [Gálvez-López, 2012] Gálvez-López, D., & Tardos, J. D. (2012). Bags of binary words for fast place recognition in image sequences. IEEE Transactions on robotics, 28(5), 1188-1197.
# [Nister, 2006] Nister, David, and Henrik Stewenius. "Scalable recognition with a vocabulary tree." 2006 IEEE Computer Society Conference on Computer Vision and Pattern Recognition (CVPR'06). Vol. 2. Ieee, 2006.

## =============================================================================
## Vocabulary
## =============================================================================
const Wordnode = @NamedTuple{
    leaveId::Int, word::SVector{128, Float32}, count::Int, level::Int, weight::Float64}
const Word = @NamedTuple{leaveId::Int, weight::Float64}
function Base.zero(::Type{Wordnode})
    (leaveId = 0, word = zeros(SVector{128, Float32}), count = 0, level = 0, weight = 0.0)
end

function add_voc_children!(tree, descriptors::Matrix, parent = 1, level = 0; progress)
    children = Clustering.kmeans(descriptors, tree.graph_data[:n_children])
    level += 1
    for (i, (centre, cnt)) in enumerate(zip(eachcol(children.centers), children.counts))
        next!(progress)
        idx = nv(tree) + 1

        # only crete children if count enough for n_children or not last level
        isLeaveNode = cnt < tree.graph_data[:n_children] ||
                      level >= tree.graph_data[:n_levels]

        if isLeaveNode
            tree.graph_data[:n_leaves] += 1
            leaveId = tree.graph_data[:n_leaves]
        else
            leaveId = 0
        end

        tree[idx] = (
            leaveId = leaveId, word = centre, count = cnt, level = level, weight = 0.0)
        add_edge!(tree, parent, idx)

        if isLeaveNode
            # push!(tree.graph_data[:leaveIds], idx)
            continue
        else
            next_idxs = findall(children.assignments .== i)
            add_voc_children!(tree, descriptors[:, next_idxs], idx, level; progress)
        end
    end

    return nothing
end

function countOccurance(voctree, image_descriptors)
    image_word_count = [Set() for _ in 1:voctree.graph_data[:n_leaves]]
    # FIXME make thread save if needed for speed, got undef # Threads.@threads 
    for (idx, image_desc) in enumerate(image_descriptors)
        img_words = getWords(voctree, image_desc)
        for img_word in img_words
            push!(image_word_count[img_word.leaveId], idx)
        end
    end
    return length.(image_word_count)
end

# set inverse document frequency idf weights for the vocabulary [SZ 2003]
function setVocabularyWeigths!(voctree, all_desc)
    N = length(all_desc) # how many images used to train the vocabulary
    occs = countOccurance(voctree, all_desc)
    _weights = map(occs) do n_i
        log(N / n_i) # [SZ 2003, Sec 4]
    end
    @showprogress for l in labels(voctree)
        node = voctree[l]
        if node.leaveId > 0
            voctree[l] = (node..., weight = _weights[node.leaveId])
        end
    end
end

"""
    buildVocabulary

Build a vocabulary tree from a set of descriptors.
image_descriptors is a vector of vectors of descriptors, each vector of descriptors is from one image.
"""
function buildVocabulary(image_descriptors::Vector, n_children = 9, n_levels = 5)
    descriptors = reduce(vcat, image_descriptors)

    # tree and root vertex
    voctree = MetaGraph(
        DiGraph(),
        Int,
        Wordnode,
        Nothing,
        Dict(
            :n_children => n_children,
            :n_levels => n_levels,
            :n_leaves => 0
        )
    )

    voctree[1] = (leaveId = 0, word = mean(descriptors),
        count = length(descriptors), level = 0, weight = 0.0)
    @cast desc_mat[j, i] := descriptors[i][j]

    n_nodes = sum(n_children .^ collect(0:n_levels))
    progress = Progress(n_nodes; dt = 1.0)

    add_voc_children!(voctree, desc_mat; progress)

    setVocabularyWeigths!(voctree, image_descriptors)

    finish!(progress)
    return voctree
end

## =============================================================================
## Vocabulary Lookup
## =============================================================================
"""
    getWord(tree, lookmeup, nodeIdx=1, level=0, MAX_LEVEL=tree.graph_data[:n_levels]; dist=Distances.Euclidean())

Recursively traverse the vocabulary tree structure to find the closest word to `lookmeup` using a specified distance metric.

# Arguments
- `tree`: The vocabulary tree structure containing nodes with words.
- `lookmeup`: The features to look up in the tree.
- `nodeIdx`: The current node index (default is 1).
- `level`: The current level in the tree (default is 0).
- `MAX_LEVEL`: The maximum level to traverse in the tree (default is `tree.graph_data[:n_levels]`).
- `dist`: The distance metric to use for comparison (default is `Distances.Euclidean()`).

# Returns
- A named tuple containing:
  - `leaveId`: The ID of the leaf node.
  - `weight`: The weight associated with the leaf node.
"""
function getWord(tree, lookmeup, nodeIdx = 1, level = 0, MAX_LEVEL = tree.graph_data[:n_levels]; 
        dist = Distances.Euclidean()
)
    level += 1
    children = outneighbors(tree, nodeIdx)
    # @debug level nodeIdx children
    if level <= MAX_LEVEL && !isempty(children)
        dists = map(children) do i
            dist(tree[i].word, lookmeup)
        end
        getWord(tree, lookmeup, children[argmin(dists)], level, MAX_LEVEL; dist)
    else
        (; leaveId, weight) = tree[nodeIdx]
        return (leaveId = leaveId, weight = weight)
    end
end

"""
    getWords(tree, lookupvec, nodeIdx=1, level=0, MAX_LEVEL=tree.graph_data[:n_levels]; dist=Distances.Euclidean())

Given a vocabulary tree and a vector of lookup features, this function computes the corresponding words for each value in the lookup vector.

# Arguments
- `tree`: The tree structure containing the vocabulary.
- `lookupvec`: A vector of values for which words need to be found.
- `dist`: The distance metric to use (default is `Distances.Euclidean()`).

# Returns
- `words`: A vector of words corresponding to each value in the lookup vector.
"""
function getWords(tree,
    lookupvec,
    nodeIdx = 1,
    level = 0,
    MAX_LEVEL = tree.graph_data[:n_levels];
    dist = Distances.Euclidean()
)
    # words = Vector{Wordnode}(undef, length(lookupvec))
    # words = zeros(Wordnode, length(lookupvec))
    words = Vector{Word}(undef, length(lookupvec))
    Threads.@threads for idx in eachindex(lookupvec)
        words[idx] = getWord(tree, lookupvec[idx], nodeIdx, level, MAX_LEVEL; dist)
    end
    return words
end

function getWords(tree::MetaGraph, lookupsift::Vector{SIFTDescriptor}, args...; kwargs...)
    return getWords(tree, getproperty.(lookupsift, :value))
end

"""
    getBowvector(voctree, image_words)

Compute the Bag of Words (BoW) vector for a given image.

# Arguments
- `voctree`: The vocabulary tree used to generate the words.
- `image_words`: The words extracted from the image.

# Returns
- A sparse vector representing the Term Frequency-Inverse Document Frequency (TF-IDF) of the image words.

"""
function getBowvector(voctree, image_words)
    tfvec = spzeros(voctree[][:n_leaves])
    idfvec = spzeros(voctree[][:n_leaves]) # IDF weights
    # TODO can improve, bit inefficient, but easy
    for (i, w) in image_words
        # calculate n_id  number of occurences of word i in image d
        tfvec[i] += 1
        idfvec[i] = w
    end
    n_d = length(image_words) #total number of words in image d
    # n_id/n_d*log(N/n_i), # [SZ 2003, Sec 4]
    return (tfvec / n_d) .* idfvec # TF_IDF bowvec
end

"""
    score_L1(v1, v2)

Compute the L1 score between two vectors `v1` and `v2`.

The L1 score is calculated as `1 - 0.5 * norm(v1 / norm(v1) .- v2 / norm(v2))`, which measures the similarity between the two vectors.

# Arguments
- `v1::AbstractVector`: The first input vector.
- `v2::AbstractVector`: The second input vector.

# Returns
- `Float64`: The L1 score between the two input vectors.

# References
- [Nister, 2006]
- [Gálvez-López, 2014]
"""
function score_L1(v1, v2)
    # [Gálvez-López, 2014] eq2 #TODO can optimize if needed with [Nister, 2006] eq 5
    return 1 - 0.5 * norm(v1 / norm(v1, 1) .- v2 / norm(v2, 1), 1)
end

function score_L2(v1, v2)
    # [Nister, 2006] eq 6
    # return 2.0 - sqrt(1.0 - dot(v1, v2))
    return 1 - 0.5 * norm(v1 / norm(v1) .- v2 / norm(v2))
end

function score_norm(p=2)
    return (v1,v2) -> 1 - 0.5 * norm(v1 / norm(v1, p) .- v2 / norm(v2, p), p)
end
## =============================================================================
## Image DB
## =============================================================================
# 
"""
    createImageInverseIndex(voctree, image_descriptors)

Create an image inverse index using Term Frequency-Inverse Document Frequency (TF-IDF) weighting.

# Arguments
- `voctree`: A vocabulary tree structure containing the graph data and other relevant information.
- `image_descriptors`: A collection of image descriptors, where each descriptor is a pair consisting of an image identifier and its corresponding feature descriptors.

# Returns
- `image_index`: A sparse matrix where each column corresponds to an image and each row corresponds to a word in the vocabulary. The values are the TF-IDF weights.
- `image_ids`: A vector of image identifiers corresponding to the columns of the `image_index`.
"""
function createImageInverseIndex(voctree, image_descriptors)
    #creation is a bit slower this way, but should be easier to create faster lookups
    image_index = spzeros(voctree.graph_data[:n_leaves], length(image_descriptors))
    @showprogress for (l, image_desc) in enumerate(image_descriptors)
        img_words = getWords(voctree, image_desc.second)
        bow_vec = getBowvector(voctree, img_words)
        for (i, bv) in zip(findnz(bow_vec)...)
            image_index[i,l] = bv
        end
    end
    return image_index, first.(image_descriptors)
end

function createImageInverseIndex_idf(voctree, image_descriptors)
    image_index = [Tuple{Symbol, Float64}[] for _ in 1:voctree.graph_data[:n_leaves]]
    word_index = Dict{Symbol, Vector{Int64}}()
    for image_desc in image_descriptors
        img_words = getWords(voctree, getproperty.(image_desc.second, :value))
        for img_word in img_words
            push!(image_index[img_word.leaveId], (image_desc.first, img_word.weight))
            push!(get!(word_index, image_desc.first, Int[]), img_word.leaveId)
        end
    end
    return image_index, word_index
end


## =============================================================================
## Lookup
## =============================================================================

"""
    findkImages_BF_binary(image_inverse_index, image_word; k=10)

Finds the top `k` images that match the given `image_word` using a brute-force search using binary weights.

# Returns
- `Array`: An array of pairs where each pair consists of an image identifier and its corresponding score, sorted by score in descending order.
"""
function findkImages_BF_binary(image_inverse_index, image_word; k=10)
    worddict = OrderedDict{Symbol, Float64}()
    # Threads.@threads 
    for words in image_word
        poses = image_inverse_index[words[1]]
        foreach(poses) do p
            get!(worddict, p[1], 0) 
            worddict[p[1]] += 1
        end
    end
    sort!(worddict; byvalue=true, rev=true)
    return collect(pairs(worddict))[1:k]
end

"""
    findkImages_BF(image_index, image_labels, bowvec, score=score_L1; k=10)

Finds the top `k` images that best match the given bag-of-words vector (`bowvec`) using a brute-force approach.

# Arguments
- `image_index::Matrix{Float64}`: A matrix where each column represents the bag-of-words vector of an image in the DB.
- `image_labels::Vector{Symbol}`: A vector containing the labels of the images in the DB.
- `bowvec::Vector{Float64}`: The bag-of-words vector of the query image.
- `score::Function`: A function to compute the similarity score between two bag-of-words vectors. Defaults to `score_L1`.
- `k::Int`: The number of top matches to return. Defaults to 10.

# Returns
- `Vector{Pair{Symbol, Float64}}`: A vector of pairs where each pair consists of an image label and its corresponding similarity score, sorted in descending order of similarity. Only the top `k` matches are returned.
"""
function findkImages_BF(image_index, image_labels, bowvec, score=score_L1; k=10)
    matches = Vector{Pair{Symbol, Float64}}(undef, size(image_index,2))
    Threads.@threads for i in eachindex(matches) 
        vdb = image_index[:,i]
        matches[i] = image_labels[i]=>score(vdb, bowvec)
    end
    sort!(matches, by=last, rev=true)
    return matches[1:k]
end
