function plotCS2(O,UVW,color,refNumber,h)
%----------------------------------------------------------
% This function plots a coordinate system in 3D
%
% Inputs: 
%       O: the origin [ox oy oz]
%       UVW: the unit vectors of the CS [U;V;W]
%       color: the color to plot the lines
%       refNumber: the number to add to the axis labels
%       h: the handle to the figure the CS will be added to
%
% Output:
%        []: the CS will be plotted on figure(h)
%
%----------------------------------------------------------
% Reza Ahmadzadeh - IRIM 2016 (reza.ahmadzadeh@gatech.edu)
%----------------------------------------------------------

if nargin < 5
    figure;
else
    figure(h);hold on
end
if nargin < 4
    refNumber = 1;
end
if nargin < 3
    color = 'k';
end
plot3([O(1,1) O(1,1)+UVW(1,1)], [O(1,2) O(1,2)+UVW(1,2)],[O(1,3) O(1,3)+UVW(1,3)],'color',color);
plot3([O(1,1) O(1,1)+UVW(2,1)], [O(1,2) O(1,2)+UVW(2,2)],[O(1,3) O(1,3)+UVW(2,3)],'color',color);
plot3([O(1,1) O(1,1)+UVW(3,1)], [O(1,2) O(1,2)+UVW(3,2)],[O(1,3) O(1,3)+UVW(3,3)],'color',color);
text(O(1,1)+UVW(1,1),O(1,2)+UVW(1,2),O(1,3)+UVW(1,3),['x_' num2str(refNumber)]);
text(O(1,1)+UVW(2,1),O(1,2)+UVW(2,2),O(1,3)+UVW(2,3),['y_' num2str(refNumber)]);
text(O(1,1)+UVW(3,1),O(1,2)+UVW(3,2),O(1,3)+UVW(3,3),['z_' num2str(refNumber)]);
end
