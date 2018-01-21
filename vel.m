function [ V, a ] = vel( k, o, h, F )
% metoda Styrlinga

D = zeros (k,3);

for i=1:(k-1)
    D(i,1) = F(i+1) - F(i);
end

for i=1:(k-2)
    D(i,2) = D(i+1,1) - D(i,1);
end

for i=1:(k-3)
    D(i,3) = D(i+1,2) - D(i,2);
end

V = zeros (k,1);
for k=(o+2):(k-2)
    V(k) = (((D(k,1)+D(k-1,1))/2)-(D(k-1,3)+D(k-2,3))/12)/h;
end

a = 2;

end

