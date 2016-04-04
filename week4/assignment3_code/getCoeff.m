function [coeff, A, b] = getCoeff(waypoints)
% waypoints assume to be a N,1 vector of one of the coordinates (we call
% this function 3 times, for x,y,z)

n = size(waypoints, 1)-1; %n = size(waypoints, 2)-1; % number of segments P1..n
A = zeros(8*n, 8*n);
b = zeros(1, 8*n); %b = zeros(3, 8*n);

% Here, fill the b vector with Wi and Wi+1 waypoints based on the 1st and
% second constraints. All the rest of constraints are zero.

% your code here
for i = 1:n
    b(1, i) = waypoints(i, 1); %b(:, i) = waypoints(:, i); %
    b(1, i + n) = waypoints(i+1, 1); %b(:, i+n) = waypoints(:, i+1); %
end

row = 1;

% Constraint 1) Pi(0) = Wi for all i = 1..n

% Here you code to loop through the first n constraints, for each you
% should call the utility function polyT(8, 0, 0) to get the vector of the
% coefficients. For example:

for i = 1:n
   A(row, (i-1)*8+1: i*8) = polyT(8, 0, 0);
   row = row + 1;
end

%A(row, 1:8) = polyT(8, 0, 0); % for i = 1

%row = row + 1;

%A(row+1, 9:16) = polyT(8, 0, 0); % for i = 2

% Constraint 2) Pi(1) = Wi + 1 for all i = 1..n

% your code here
for i = 1:n
   A(row, (i-1)*8+1: i*8) = polyT(8, 0, 1);
   row = row + 1;
end

% Constraint 3) P1(k)(0) = 0 for all 1 <= k <= 3

% your code here
for k = 1:3
    A(row, (1-1)*8+1: 1*8) = polyT(8, k, 0);
    row = row + 1;
end

% Constraint 4) Pn(k)(1) = 0 for all 1 <= k <= 3

% your code here
for k = 1:3
   A(row, (n-1)*8+1: n*8) = polyT(8, k, 1);
   row = row + 1;
end

% Constraint 5) Pi-1(k)(1) = Pi(k)(0) for all i = 2..n and for all k =1..6

% your code here
for i = 2:n
   for k = 1:6
       A(row, (i-2)*8+1: i*8) = [polyT(8, k, 1) -polyT(8, k, 0)];
       row = row + 1;
   end
end

% Example:
%A(row, 1:16) = [ polyT(8,1,1) - polyT(8,1,0)] % this is for i=2 and k=1

% notice that we construct a vector of 1*16 for the coefficients of
% a11..a18 a21..a28. The reason we use a minus before the second call to
% polyT is because the coefficients of a21..a28 are all with minus:
% Pi-1(k)(1) - Pi(k)(0) = 0

% your code here.

% we can now return the coefficient vector. Notice that if inv fails, it is
% most likely that you did not code the above correctly. You can try pinv,
% but you'd better fix your code instead.

coeff = A\b'; %inv(A)*b;
end

%https://www.coursera.org/learn/robotics-flight/module/XoPE0/discussions/hcXrPs9yEeWwoQrbIHhKaQ



















