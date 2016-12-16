%MDL_JACO2 Create model of Kinova Jaco2 manipulator
%
% MDL_JACO2 is a script that creates the workspace variable jaco which
% describes the kinematic characteristics of a Kinova Jaco2 manipulator
% using standard DH conventions.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%
% Reference::
% - Jaco2 6DOF Advanced specification guide May 08, 2015
%
% Notes::
% - SI units of metres are used.
% - The DH model is specified in degrees and converted to radians for the need of matlab.
% - Unlike most other mdl_xxx scripts this one is actually a function that
%   behaves like a script and writes to the global workspace.
%
% See also SerialLink, mdl_mico, mdl_puma560.

% MODEL: Kinova, Jaco2, 6DOF, standard_DH


% Copyright (C) 1993-2015, by Peter I. Corke 
% modified by Assia Benbihi to get Jaco2
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function r = mdl_jaco2()
    
    deg = pi/180; % to bring everything back to radians
    
    % robot length values (metres)
    D1 = 0.2755;
    D2 = 0.4100;
    D3 = 0.2073;
    D4 = 0.0741;
    D5 = 0.0741;
    D6 = 0.1600;
    e2 = 0.0098;
    
    % alternate parameters
    aa = 30*deg;
    ca = cos(aa);
    sa = sin(aa);
    c2a = cos(2*aa);
    s2a = sin(2*aa);
    d4b = D3 + sa/s2a*D4;
    d5b = sa/s2a*D4 + sa/s2a*D5;
    d6b = sa/s2a*D5 + D6;
    
    % and build a serial link manipulator
    
    % offsets from the table on page 4, "Mico" angles are the passed joint
    % angles.  "DH Algo" are the result after adding the joint angle offset.
	% DONE:
	% 1st link Revolute('alpha', pi/2,  'a', 0,  'd', D1,   'flip')
	% Base translation: robot.base = transl(-0.16,-0.085,0);
	%Revolute('alpha', pi/2,  'a', 0,  'd', D1,   'offset', -pi/2)
	
    robot = SerialLink([
    	Revolute('alpha', pi/2,  'a', 0,  'd', D1,   'flip')
        Revolute('alpha', pi,    'a', D2, 'd', 0,    'offset', -pi/2)
        Revolute('alpha', pi/2,  'a', 0,  'd', -e2,  'offset', pi/2)
        Revolute('alpha', 2*aa,  'a', 0,  'd', -d4b)
        Revolute('alpha', 2*aa,  'a', 0,  'd', -d5b, 'offset', -pi)
        Revolute('alpha', pi,    'a', 0,  'd', -d6b, 'offset', 90*deg)
        ], ...
        'name', 'Jaco2', 'manufacturer', 'Kinova');

	%robot.base = transl(0.175,0,0);
 	%robot.base = transl(0,0.03,0);
    
    % place the variables into the global workspace
    if nargin == 1
        r = robot;
    elseif nargin == 0
        assignin('base', 'jaco2', robot);
        assignin('base', 'qz', [0 0 0 0 0 0]); % zero angles
        assignin('base', 'qr', [270 180 180 0 0 0]*deg); % vertical pose as per Fig 2
    end
end
