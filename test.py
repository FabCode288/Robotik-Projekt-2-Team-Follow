# abs pitch > abs roll    pitch runter ist gut
#       linear = 0
#       roll + -> links lenken
#            angular = v            
#       roll - -> rechts lenken
#            angular = -v
#
#else                            (abs pitch< abs roll)
#   sin roll * omega = angular
#   cos abs roll * v = linear
g = [5,5,5,5,5,5,5,5,5,5,3,3,3,3,3,2,2,2,2,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0]
x_ = 0
sig = 0

for i in range g.len-1:
    x_ += g[i]

print(x_)

for j in range g.len-1:
    sig += (x_-g[j])^2/(x_-1)