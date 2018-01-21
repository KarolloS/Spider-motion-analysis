function [ a ] = angle( A, B, C )
% zwarac k�t o wierzcho�ku w punkcie B 
%warto�� zwracana w stopniach 

BA = [A(1)-B(1), A(2)-B(2), A(3)-B(3)];
BC = [C(1)-B(1), C(2)-B(2), C(3)-B(3)];

ca = dot(BA, BC)/(norm(BA)*norm(BC));
a = radtodeg(acos(ca));

end

