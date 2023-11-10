## Good to know

# Conditional Multivariate Normals

```julia
using Distributions
using LinearAlgebra

##

# P(A|B)

Σab = 0.2*randn(3,3)
Σab += Σab'
Σab += diagm([1.0;1.0;1.0])

μ_ab = [10.0;0.0;-1.0]
μ_1 = μ_ab[1:1]
μ_2 = μ_ab[2:3]

Σ_11 = Σab[1:1,1:1]
Σ_12 = Σab[1:1,2:3]
Σ_21 = Σab[2:3,1:1]
Σ_22 = Σab[2:3,2:3]

##

# P(A|B) = P(A,B) / P(B)
P_AB = MvNormal(μ_ab, Σab)                       # likelihood
P_B = MvNormal([-0.5;0.75], [0.75 0.3; 0.3 2.0]) # evidence

# Schur compliment
μ_(b) = μ_1 + Σ_12*Σ_22^(-1)*(b-μ_2)
Σ_ = Σ_11 + Σ_12*Σ_22^(-1)*Σ_21

P_AB_B(a,b) = pdf(P_AB, [a;b]) / pdf(P_B, b)
P_A_B(a,b; mv = MvNormal(μ_(b), Σ_)) = pdf(mv, a) 

##

# probability density: p(a) = P(A=a | B=b)
@show P_A_B([1.;],[0.;0.])
@show P_AB_B([1.;],[0.;0.])

P(A|B=B(.))
```

# Various Internal Function Docs

```@docs
_solveCCWNumeric!
```
