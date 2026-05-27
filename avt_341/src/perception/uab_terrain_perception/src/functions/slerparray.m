function quatsOUT = slerparray(X,quats,XQ)
%   This function is a combination of SLERP interpolation, with
%   a lookup table. It interpolates the quaternions of the
%   underlying function QUATS = F(X) at the query points XQ.
%   Quaternions maybe be entered as a quaternion array or a
%   Nx4 double matrix. X and XQ must be Nx1 matrices.

% Example:
% time = [10;30;50];
% q1 = quaternion([-30,0,-10],'eulerd','ZYX','frame');
% q2 = quaternion([-40,-30,35],'eulerd','ZYX','frame');
% q3 = quaternion([-80,10,30],'eulerd','ZYX','frame');
% quatsIN = [q1;q2;q3];
% timeq = (11:49)';
% quatsOUT = slerparray(time,quatsIN,timeq);
% pts = rotatepoint(quatsIN,[1,0,0]);
% newpts = rotatepoint(quatsOUT,[1,0,0]);
% figure
% [X,Y,Z] = sphere;
% surf(X,Y,Z,'FaceColor',[0.57 0.57 0.57])
% hold on;
% scatter3(newpts(:,1),newpts(:,2),newpts(:,3),'filled','b')
% scatter3(pts(:,1),pts(:,2),pts(:,3),'filled','r')
% view(30,10)
% axis equal

% Made by:
% Andrés Morales


% Check sample points are sorted
if ~issortedrows(X)
    error('Sample points must be sorted')
end
% Check for Extrapolation
if XQ(1)<X(1) || XQ(end)>X(end)
    error('No extrapolation available. Check query points.')
end
% Check Input for Quaternion or Nx4
if ~(isa(quats,"quaternion") || (isa(quats,"double") && size(quats,2)==4))
    error('Input must be Quaternion or double Nx4 Matrix')
end
% Convert Double to Quaternion
if isa(quats,"double")
    quats = quaternion(quats);
end

iafter = arrayfun(@(x) find(X>x,1),XQ);
ibefore = iafter-1;
xbefore = X(ibefore);
xafter = X(iafter);
quatbefore = quats(ibefore);
quatafter = quats(iafter);
interpolationCoefficients = (XQ-xbefore)./(xafter-xbefore);

quatsOUT = slerp(quatbefore,quatafter,interpolationCoefficients);
end