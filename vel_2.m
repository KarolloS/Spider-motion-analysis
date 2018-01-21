function [ V, a ] = vel_2( k, o, h, F)
 
V = zeros (k,1);
for k=(o+1):(k-1)
    V(k) = (F(k+1)-F(k-1))/(2*h);
end

a = 1;

end

