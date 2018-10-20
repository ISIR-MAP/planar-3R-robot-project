function [a] = inv_rotation_Z_etu(R)
% INV_ROTATION_Z retourne l'angle de rotation autour de l'axe Z à partir
% d'une matrice de rotation passée comme argument (et dont on suppose 
% qu'elle représente bien une rotation d'axe Z).
%   [a] = inv_rotation_Z(R)

a = atan2(R(2,1), R(1,1));

end