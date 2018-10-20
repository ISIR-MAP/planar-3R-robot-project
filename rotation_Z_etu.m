function [R] = rotation_Z_etu(angle)
%  ROTATION_Z retourne la matrice de rotation 3D
% correspondant à une rotation d'angle 'angle' autour de x 
%  R = rotation_Z(angle)

R = eye(3);
R = [cos(angle), -sin(angle), 0 ; sin(angle), cos(angle), 0 ; 0,0,1];

end
