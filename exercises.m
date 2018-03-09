%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (1) Let v = (8,6) . Compute the length of v. 
v = [8;6];

norm(v)
sqrt(v(1)^2 + v(2)^2)

% Give the coordinates of a unit vector u normal to v.
unorm = [-v(2); v(1)]
u = unorm / norm(unorm)
% floating point error, this should give 0
dot(u,v)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (2) For two points P = (1,2) and Q = (4,8), compute the intermediate points that are 1/3, 1/2 and 3/4 the way between P and Q.
P = [1;2];
Q = [4;8];
v = (Q - P)
P + v * 0.5
P + v * 1/3
P + v * 3/4


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (3) Let v = (8,6) , w = (0,5) , and θ = the angle between v and w.
% Using the dot and perp products, compute cos( θ ) and sin( θ ) .
v = [8;6];
w = [0;5];

% dot
acosd( dot(v, w) / (norm(v) * norm(w)) )

% perp
asind( dot( [-v(2); v(1)], w) / (norm(v) * norm(w)) )


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (4) Let v = (v1 , v2 , v3 ) and w = (w1 , w2 , w3 ) be any two 3D vectors.
% Prove that the equations: (v × w) · v = 0 and (v × w) · w = 0 are always true,
% and thus that v × w is always perpendicular to both v and w .
% v1 = sym('v1'); v2 = sym('v2'); v3 = sym('v3'); w1 = sym('w1'); w2 = sym('w2'); w3 = sym('w3'); 

syms v1 v2 v3 w1 w2 w3;
cross = [v2 * w3 - v3 * w2; v3 * w1 - v1 * w3; v1 * w2 - v2 * w1]
dot = cross(1) * v1 + cross(2) * v2 + cross(3) * v3
simplify(dot)

dot = cross(1) * w1 + cross(2) * w2 + cross(3) * w3
simplify(dot)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (5) Let u = (u1 , u2 , u3 ), v = (v1 , v2 , v3 ) and w = (w1 , w2 , w3 ) be any three 3D vectors. Prove that the
% equation: u · (v × w) = (u × v) · w is always true. Show, using geometric reasoning, that it is equal to
% the volume of the 3D parallelepiped defined by edge vectors u, v, and w starting at the origin.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (6) Let a 2D (infinite) line pass through two points P 0 and P 1 . Given any arbitrary point P in the plane,
% show that the perp product: (P – P 0 ) ⊥ (P 1 – P 0 ) will be positive for points P on one side of the line,
% and negative for points on the other side of the line.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (7) Using the previous exercise, develop a test for whether a finite segment between points Q 0 and Q 1
% crosses (i.e. intersects) the (infinite) line through P 0 and P 1 . Use this to develop another test for
% whether the finite segment Q 0 Q 1 intersects with the finite segment P 0 P 1 without actually computing
% the point of intersection

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (8) Let a 3D (infinite) line L be defined by two points P 0 = (x 0 , y 0 , z 0 ) and P 1 = (x 1 , y 1 , z 1 ) on it,
% and let P = (x, y, z) be an arbitrary point. Put ∆x = (x 1 – x 0 ), ∆y = (y 1 – y 0 ), ∆z = (z 1 – z 0 );
% and ∆r = (x – x 0 ), ∆s = (y – y 0 ), ∆t = (z – z 0 ).
% Show that the distance d(P, L) from P to the line L is given by the formula:
% d ( P , L ) = ( ∆ z ∆ s − ∆ y ∆ t ) 2 + ( ∆ x ∆ t − ∆ z ∆ r ) 2 + ( ∆ y ∆ r − ∆ x ∆ s ) 2
                                          % ( ∆ x 2 + ∆ y 2 + ∆ z 2 )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (9) In the 2D plane, let a and b be two fixed perpendicular vectors, and let v be any arbitrary vector.
% Show that v can be decomposed into the sum of two vectors in the directions of a and b. That is,
% show that the linear equation v = αa + βb can be solved by finding explicit formulas for the
% coefficients α and β in terms of a , b , and v. [Hint: use the dot product.]
