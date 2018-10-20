function p = angle2step(q,offset)
% ANGLE2STEP convertit la valeur angulaire Q (rad) en une valeur de pas
% moteur P.
% OFFSET permet de spécifier à quelle valeur de pas moteur correspond la
% valeur angulaire 0.
% 
% p = angle2step(q,offset);

% VOTRE CODE ICI
p = (q /((300/1024)*(2*pi/360)))+offset;
end
