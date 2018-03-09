%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
v = [1 2 3]
w = [4 5 6]

plotv(v)

% index into vector (starts with 1 of course...)
v(2)

% norm aka. length
norm(v)
sqrt(v(1) ^ 2 + v(2) ^ 2 + v(3) ^ 2)

% unit vector aka. vector with length 1
u = v / norm(v)
norm(u) == 1

% dot aka scalar product
dot(v, w)
v(1) * w(1) + v(2) * w(2) + v(3) * w(3)

acosd ( dot(v, w) / (norm(v) * norm(w)))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% matrices
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = [1 2 3; 4 5 6; 7 8 9]

% transpose matrix (spiegeln an hauptache)
M'

% inverse
M * inv(M)
