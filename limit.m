function new_pos = limit(pos,val_min,val_max)
% LIMIT limite la valeur de position "pos" entre ses valeurs
% minimales et maximales autorisées par la géométrie du 
% robot
% 
% pos = limit(pos,val_min,val_max);

% limit en step

% VOTRE CODE ICI

if (pos < val_min)
    new_pos = val_min;
    disp('erreur de limit: valeur de q < valmin');
elseif (pos > val_max)
    new_pos = val_max;
    disp('erreur de limit: valeur de q > valmax');
else
    new_pos = pos;
end
    