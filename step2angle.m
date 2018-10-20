function q = step2angle(step,offset)
% STEP2ANGLE convertit la valeur de pas moteur STEP en une valeur angulaire
% Q exprim�e en rad.
% OFFSET permet de sp�cifier � quelle valeur de pas moteur correspond la
% valeur angulaire 0.
% 

% VOTRE CODE ICI
q = (step-offset)*(300/1024)*(2*pi/360);
end