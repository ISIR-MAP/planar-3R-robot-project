function v = val2speed(value)
% VAL2SPEED convertit la valeur de vitesse VALUE lue sur le moteur en une 
% valeur de vitesse V exprim�e en tour/min.
% 
% v = val2speed(value);
% 1 unit� de value est 0.111rpm

% VOTRE CODE ICI
v = 0.111*value;
end