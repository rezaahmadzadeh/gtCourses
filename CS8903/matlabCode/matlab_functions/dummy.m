clc,clear,close all

fig=gcf;

P = [0 1 0]';

O0=[0 0 0]; %origin
UVW0 = [1 0 0;
    0 1 0;
    0 0 1];
refNumber0=0;
color0=[0 1 0];
plotCS2(O0,UVW0,color0,refNumber0,fig);
P0 = P;
scatter3(P0(1), P0(2), P0(3), 'filled','g');


O1=[1 2 1]; %origin
theta1 = pi/4;
UVW1 =  [1 0 0; 0 cos(theta1) -sin(theta1) ; 0 sin(theta1) cos(theta1)]
refNumber1=1;
color1=[1 0 0];
plotCS2(O1,UVW1',color1,refNumber1,fig);
P1= O1' + P(1)*UVW1(:,1) + P(2)*UVW1(:,2) + P(3)*UVW1(:,3);
scatter3(P1(1), P1(2), P1(3), 'filled','r');


O2=[0 0 0]; %origin
theta2 = pi/3;
UVW2 =  [cos(theta2) -sin(theta2) 0;sin(theta2) cos(theta2) 0; 0 0 1];
refNumber2=2;
color2=[0 0 1];
plotCS2(O2,UVW2',color2,refNumber2,fig);
P2= O2' + P(1)*UVW2(:,1) + P(2)*UVW2(:,2) + P(3)*UVW2(:,3)
scatter3(P2(1), P2(2), P2(3), 'filled','b');


xlabel('x');
ylabel('y');
zlabel('z');

%{

dcm = getDCM(UVW0,UVW1)

[r1, r2, r3] = dcm2angle(dcm,'ZYX');
rotx(r1*pi/180)


q = dcm2quat(dcm);
%}


