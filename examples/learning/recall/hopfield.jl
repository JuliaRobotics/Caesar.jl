
function train(X)
  l = size(X,1)
  p = size(X,2)
  W = zeros(l,l)
  for i in 1:p
    W += X[:,i]*X[:,i]'
  end
  W -= p*eye(l,l)
  return W
end

iterHopfield(W::Array{Float64,2}, x::Vector{Float64}) = sign(W*x)

hamming(a,b) = sum(abs(a-b)/2)

function findmatch(Ps::Array{Float64,2}, v::Vector{Float64})
  l = size(Ps,2)
  min = 1
  minidx = -1
  for i in 1:l
    d = hamming(Ps[:,i], v)
    if min == 0 && d == 0
      warn("find match -- double match")
      return -1
    end
    if d < min
      min = d
      minidx = i
    end
  end
  return minidx
end

function searchHopfield(Ps::Array{Float64,2}, v0::Vector{Float64}; W::Array{Float64,2}=zeros(1,0), maxiter::Int=4)
  W2 = Union{}
  if size(W,2) == 0
    W2 = train(Ps)
  else
    W2 = W
  end
  v = deepcopy(v0)
  for i in 1:maxiter
    v1 = iterHopfield(W2,v)
    hamming(v,v1) > 0 ? v = v1 : return findmatch(Ps, v1)
  end
  return -1
end


function basicTestNet()
  N = 20
  a = sign(randn(N)-1)
  b = sign(randn(N)-1)
  c = sign(randn(N))
  Ps = [a';b';c']'
  a1 = deepcopy(a)
  a1[1] = -1*a[1]
  a1[6] = -1*a[6]
  @show searchHopfield(Ps, a1)
  b1 = deepcopy(b)
  b1[6] = -1*b[6]
  @show searchHopfield(Ps, b1)
  return true
end

basicTestNet()
