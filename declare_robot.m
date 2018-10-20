function robot = declare_robot(n,Trans,q)
% DECLARE_ROBOT crée, initialise et un retourne une varibale structurée
% contenant les attributs d'un robot sériel nR
%   rob = declare_robot(n,Trans,q)
%   où :
%   - 'n' est un entier positif non nul représentant le nombre de ddl du
%   robot
%   - 'Trans' est un vecteur à n éléments contenant la longueur l_i de chacun
%   des corps du robot
%   - 'q' est un vecteur colonne de 'n' réels contenant la position angulaire de chaque axe
%
%   Cette fonction ne fait aucune vérification de type ou de dimension

robot.n = n;
robot.T = Trans;
robot.q = q;

end
