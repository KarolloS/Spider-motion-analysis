function [h] = draw(k, R1, R2, R3, R4, L1, L2, L3, L4)
h = zeros(64);

%% 
%g³owotó³w
h(1) = myline(R1(k,:,4), R2(k,:,4));
h(2) = myline(R2(k,:,4), R3(k,:,4));
h(3) = myline(R3(k,:,4), R4(k,:,4));
h(4) = myline(R1(k,:,4), L1(k,:,4));
h(5) = myline(L1(k,:,4), L2(k,:,4));
h(6) = myline(L2(k,:,4), L3(k,:,4));
h(7) = myline(L3(k,:,4), L4(k,:,4));
h(8) = myline(L4(k,:,4), R4(k,:,4));

%R1
h(9) = myline(R1(k,:,4), R1(k,:,3));
h(10) = myline(R1(k,:,3), R1(k,:,2));
h(11) = myline(R1(k,:,2), R1(k,:,1));

%R2
h(12) = myline(R2(k,:,4), R2(k,:,3));
h(13) = myline(R2(k,:,3), R2(k,:,2));
h(14) = myline(R2(k,:,2), R2(k,:,1));

%R3
h(15) = myline(R3(k,:,4), R3(k,:,3));
h(16) = myline(R3(k,:,3), R3(k,:,2));
h(17) = myline(R3(k,:,2), R3(k,:,1));

%R4
h(18) = myline(R4(k,:,4), R4(k,:,3));
h(19) = myline(R4(k,:,3), R4(k,:,2));
h(20) = myline(R4(k,:,2), R4(k,:,1));

%L1
h(21) = myline(L1(k,:,4), L1(k,:,3));
h(22) = myline(L1(k,:,3), L1(k,:,2));
h(23) = myline(L1(k,:,2), L1(k,:,1));

%L2
h(24) = myline(L2(k,:,4), L2(k,:,3));
h(25) = myline(L2(k,:,3), L2(k,:,2));
h(26) = myline(L2(k,:,2), L2(k,:,1));

%L3
h(27) = myline(L3(k,:,4), L3(k,:,3));
h(28) = myline(L3(k,:,3), L3(k,:,2));
h(29) = myline(L3(k,:,2), L3(k,:,1));

%L4
h(30) = myline(L4(k,:,4), L4(k,:,3));
h(31) = myline(L4(k,:,3), L4(k,:,2));
h(32) = myline(L4(k,:,2), L4(k,:,1));

%% 
%punkty
[x,y,z] = sphere;
a=0.5;
for i=1:4
    h(32+i) = surf(a*x+R1(k,1,i),a*y+R1(k,2,i),a*z+R1(k,3,i));
    h(36+i) = surf(a*x+R2(k,1,i),a*y+R2(k,2,i),a*z+R2(k,3,i));
    h(40+i) = surf(a*x+R3(k,1,i),a*y+R3(k,2,i),a*z+R3(k,3,i));
    h(44+i) = surf(a*x+R4(k,1,i),a*y+R4(k,2,i),a*z+R4(k,3,i));
    h(48+i) = surf(a*x+L1(k,1,i),a*y+L1(k,2,i),a*z+L1(k,3,i));
    h(52+i) = surf(a*x+L2(k,1,i),a*y+L2(k,2,i),a*z+L2(k,3,i));
    h(56+i) = surf(a*x+L3(k,1,i),a*y+L3(k,2,i),a*z+L3(k,3,i));
    h(60+i) = surf(a*x+L4(k,1,i),a*y+L4(k,2,i),a*z+L4(k,3,i));
end
colormap([1 0 0])

axis([-70 70 0 180 0 50])
xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]') 
hold off
grid on

end

