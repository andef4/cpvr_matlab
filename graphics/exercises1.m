clear all; close all; clc; %clear matrices, close figures & clear cmd wnd.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (1) Let v = (8,6) . Compute the length of v. 
v = [8;6];

fprintf('Extercise 1\n');
norm(v)
sqrt(v(1)^2 + v(2)^2)

% Give the coordinates of a unit vector u normal to v.
unorm = [-v(2); v(1)];
u = unorm / norm(unorm);
% floating point error, this should give 0
dot(u,v)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (2) For two points P = (1,2) and Q = (4,8), compute the intermediate points that are 1/3, 1/2 and 3/4 the way between P and Q.
fprintf('Extercise 2\n');
P = [1;2];
Q = [4;8];
v = (Q - P);
P + v * 0.5
P + v * 1/3
P + v * 3/4


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (3) Let v = (8,6) , w = (0,5) , and θ = the angle between v and w.
% Using the dot and perp products, compute cos( θ ) and sin( θ ) .
fprintf('Extercise 3\n');
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
fprintf('Extercise 4\n');

syms v1 v2 v3 w1 w2 w3;
cross = [v2 * w3 - v3 * w2; v3 * w1 - v1 * w3; v1 * w2 - v2 * w1];
dot = cross(1) * v1 + cross(2) * v2 + cross(3) * v3;
simplify(dot)

dot = cross(1) * w1 + cross(2) * w2 + cross(3) * w3;
simplify(dot)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (5) Let u = (u1 , u2 , u3 ), v = (v1 , v2 , v3 ) and w = (w1 , w2 , w3 ) be any three 3D vectors. Prove that the
% equation: u · (v × w) = (u × v) · w is always true. Show, using geometric reasoning, that it is equal to
% the volume of the 3D parallelepiped defined by edge vectors u, v, and w starting at the origin.
fprintf('Extercise 5\n');

syms u1 u2 u3 v1 v2 v3 w1 w2 w3;
cross1 = [v2 * w3 - v3 * w2; v3 * w1 - v1 * w3; v1 * w2 - v2 * w1];
cross2 = [u2 * v3 - u3 * v2; u3 * v1 - u1 * v3; u1 * v2 - u2 * v1];
dot1 = cross1(1) * u1 + cross1(2) * u2 + cross1(3) * u3;
dot2 = cross2(1) * w1 + cross2(2) * w2 + cross2(3) * w3;
simplify(dot1 - dot2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (6) Let a 2D (infinite) line pass through two points P0 and P1. Given any arbitrary point P in the plane,
% show that the perp product: (P – P0) ⊥ (P1 – P0) will be positive for points P on one side of the line,
% and negative for points on the other side of the line.
fprintf('Extercise 6\n');

P = sym('P_%d', [1 2]);
P0 = sym('P0_%d', [1 2]);
P1 = sym('P1_%d', [1 2]);

v1 = P - P0;
v2 = P1 - P0;

perp = v1(1) * v2(2) - v1(2) * v2(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (7) Using the previous exercise, develop a test for whether a finite segment between points Q 0 and Q 1
% crosses (i.e. intersects) the (infinite) line through P0 and P1.
fprintf('Extercise 7\n');

% idea:
% if (perp(Q0 - P0, P0 - P1) > 0 && perp(Q1 - P0, P0 - P1) < 0 or
%   perp(Q0 - P0, P0 - P1) < 0 && perp(Q1 - P0, P0 - P1) > 0) {
%   return true
% } else {
%   return false
% }

% Use this to develop another test for
% whether the finite segment Q 0 Q 1 intersects with the finite segment P 0 P 1 without actually computing
% the point of intersection

% idea: inverse the test, check if P intersects the infinite segment of Q


+%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
+% (12) Paralellism stays
+
+% Let be two segment σ 1 = (x 11 ; y 11 ; 1) − (x 12 ; y 12 ; 1) and
+% σ 2 = (x 21 ; y 21 ; 1) − (x 22 ; y 22 ; 1). The two
+% segments are defined as parallel. If one apply a scaling transformations to these two segments,
+% their images σ ∗ 1 and σ ∗ 2 remain parallel. Prove this affirmation.
+
+
+syms x11  x12 y11 y12 x21 x22 y21 y22 a b;
+
+% paralism defined as the two vectors have the same slope
+% (x11 - x12) / (y11 - y12) = (x21 - x22) / (y21 - y22)
+
+% scaling = [a 0
+%            0 b]
+
+% every x values is scaled by a, every y value scaled by b:
+% (a*x11 - a*x12) / (b*y11 - b*y12) = (a*x21 - a*x22) / (b*y21 - b*y22)
+
+% by dividing by a und multiplying by a on both sides, we get the original
+% formula:
+simplify((a*x11 - a*x12) / (b*y11 - b*y12) / a * b)
+simplify((a*x21 - a*x22) / (b*y21 - b*y22) / a * b)


