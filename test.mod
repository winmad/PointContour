# parameters
set Point = 1..3;
param BN;
param CN {0..BN};
param init_p {i in 0..BN, 0..CN[i], Point};

# variables
var p {i in 0..BN, j in 0..CN[i], t in Point} := init_p[i, j, t];

# intermediate variables

# objective
minimize total_cost:
sum {i in 0..BN, j in 0..CN[i], t in Point} (p[i, j, t] - init_p[i, j, t]) ^ 2
;

subject to lineOrtho1: sum {t in Point} (p[0,1,t]-p[0,0,t])*(p[1,1,t]-p[1,0,t])=0;
