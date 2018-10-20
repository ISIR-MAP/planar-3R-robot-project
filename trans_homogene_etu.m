function [H] = trans_homogene_etu(R,T)
% TRANS_HOMOGENE retourne la matrice de transformation homogene 3D
% correspondant à une rotation R et à une translation T successives. 

H = eye(4);
H(1:3,1:3) = R;
H(1:3,4) = T;

end