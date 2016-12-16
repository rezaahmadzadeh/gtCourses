
% load test_transformation.mat;




stepc = 10;
figure;
hold on
quiver3(x(1:stepc:end),y(1:stepc:end),z(1:stepc:end),T(1,1:stepc:end),T(2,1:stepc:end),T(3,1:stepc:end)); %z
quiver3(x(1:stepc:end),y(1:stepc:end),z(1:stepc:end),N(1,1:stepc:end),N(2,1:stepc:end),N(3,1:stepc:end),'r'); %x
quiver3(x(1:stepc:end),y(1:stepc:end),z(1:stepc:end),B(1,1:stepc:end),B(2,1:stepc:end),B(3,1:stepc:end),'b'); %y
axis equal
box on;
grid on;
view([45, 47]);


plot3(initPoint(1,1),initPoint(1,2),initPoint(1,3),'or');
index=1;
iter=1;
color=[1 0 0];
% P = [0.075 0 0];
P = [0.0167    0.1084    0.2762];


while index<=size(x,2)
    O = [x(index) y(index) z(index)];
    UVW = [N(:,index) B(:,index) T(:,index)];
    refNumber=iter;
    P2=O'+UVW*P.';  
    plot3(P2(1), P2(2), P2(3),'bo'); 
    index = index+stepc;
end

