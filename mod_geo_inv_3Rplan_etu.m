function [q1,q2] = mod_geo_inv_3Rplan_etu(robot,x,y,theta)
% MOD_GEO_INV_3RPLAN retourne les configurations du robot correspondant
% Ã  une position cartÃ©sienne 'x', 'y' et Ã  une orientation 'theta' dans le plan
%   [q1,q2] = mod_geo_inv_3Rplan(robot,x,y,theta)

q1 = zeros(3,1);
q2 = zeros(3,1);

l = robot.T;

%Coordonnées du centre de la dernière liaison

w1 = x - l(3)*cos(theta);
w2 = y - l(3)*sin(theta);

%Résolution pour q(2)

cq2 = (w1^2 + w2^2 - l(1)^2 - l(2)^2)/(2*l(1)*l(2));

%%Gestion des cibles non atteignables

if  abs(cq2) >  1
    cq2 = sign(cq2);    
end

%% Solution A

sqa2 = sqrt(1.0 - cq2^2);
qa(2) = atan2(sqa2,cq2);

%% Solution B

sqb2 = -sqa2;
qb(2)  = atan2(sqb2,cq2);


% Résoltution pour q(1), solution A

k1A = l(1) + l(2)*cos(qa(2));
k2A = l(2)*sin(qa(2));
cqa1 = (w1*k1A + w2*k2A)/(k1A^2 + k2A^2);
sqa1 = (-w1*k2A + w2*k1A)/(k1A^2 + k2A^2);
qa(1) = atan2(sqa1,cqa1);


% Résolution pour q(1), solution B

k1B = l(1) + l(2)*cos(qb(2));
k2B = l(2)*sin(qb(2));
cqb1 = (w1*k1B + w2*k2B)/(k1B^2 + k2B^2);
sqb1 = (-w1*k2B + w2*k1B)/(k1B^2 + k2B^2);
qb(1) = atan2(sqb1,cqb1);

% Résolution pour q(3), Solution A

qa(3) = theta - qa(2) - qa(1);

%% Solution B

qb(3) = theta - qb(2) - qb(1);

q1 = qa;
q2 = qb;


end