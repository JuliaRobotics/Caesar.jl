
using Flux

# throttle, steering, bVx, bVy
Chain(
  Dense(25,8,relu),
  MaxPool(8,4),
  # flatten
  Dense(4,8,relu),
  Dense(8,2)
)
