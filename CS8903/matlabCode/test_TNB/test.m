clc,clear,close all
load test_transformation.mat;
stepc = 10;
figure;
hold on
quiver3(x(1:stepc:end),y(1:stepc:end),z(1:stepc:end),T(1,1:stepc:end),T(2,1:stepc:end),T(3,1:stepc:end));
quiver3(x(1:stepc:end),y(1:stepc:end),z(1:stepc:end),N(1,1:stepc:end),N(2,1:stepc:end),N(3,1:stepc:end));
quiver3(x(1:stepc:end),y(1:stepc:end),z(1:stepc:end),B(1,1:stepc:end),B(2,1:stepc:end),B(3,1:stepc:end));
axis equal
box on;
grid on;
view([45, 47]);


