% load test_transformation.mat;
clc,clear,close all
load test_not_working.mat;

stepc = 2;
figure;
hold on
quiver3(x(1:stepc:end),y(1:stepc:end),z(1:stepc:end),T(1,1:stepc:end),T(2,1:stepc:end),T(3,1:stepc:end)); %z
quiver3(x(1:stepc:end),y(1:stepc:end),z(1:stepc:end),N(1,1:stepc:end),N(2,1:stepc:end),N(3,1:stepc:end),'y'); %x
quiver3(x(1:stepc:end),y(1:stepc:end),z(1:stepc:end),B(1,1:stepc:end),B(2,1:stepc:end),B(3,1:stepc:end),'b'); %y
axis equal
box on;
grid on;
view([45, 47]);

index=1;
iter=1;
color=[1 0 0];

UVW1 = [N(:,index) B(:,index) T(:,index)];
O1 = [x(index) y(index) z(index)];
initPoint = [x(index) y(index) z(index)]+0.1*N(:,index)' + 0.0*B(:,index)' % in world frame
P = [1 0 0];
%P = (inv(UVW1)*(initPoint'-O1'))'
P1 = O1'+UVW1*P.'

plot3(initPoint(1,1),initPoint(1,2),initPoint(1,3),'or');
xlabel('x');
ylabel('y');
zlabel('z');

scatter3(P1(1), P1(2), P1(3),'filled','r')
%P = [0.075 0.075 0.075];
%P = [0.0167    0.1084    0.2762];
%P = [0.0167    0.1084 0];

while index<=size(x,2)
    O = [x(index) y(index) z(index)];
    UVW = [N(:,index) B(:,index) T(:,index)];
    P2=O'+UVW*P.';
    scatter3(P2(1), P2(2), P2(3),'filled','g')
    scatter3(O(1), O(2), O(3),'filled','r')
    %plot3(P2(1), P2(2), P2(3),'go'); 
    index = index+stepc;
end
axis equal
