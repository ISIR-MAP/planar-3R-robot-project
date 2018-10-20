function [] = projet()
%% Codé par Jieyeon WOO(3521100) et Zi-bo Zhang(3605370)

%% Sommaire du Code
% 1. TP1
% 2. TP2
% 3. TP3et4 - Projet
% 4. Tâche d'Ecriture de la Lettre M

%% TP 1
clear all;
close all;

% 2.1) 
% 1) Déclaration du robot r3 et r4 à l'aide de la fonction
% "declare_robot()"

% declare_robot(n,l,config)
%   n = numéro de type de plan
%   l = longueurs des segments
%   config = configuration initiale

r3 = declare_robot(3,[0.5 0.3 0.2],[0 0 0]');
r4 = declare_robot(4,[0.5 0.3 0.2 0.2],[pi/3 pi/4 pi/5 pi/6]');


% 2) Affichage du robot r3 et r4 à l'aide de la fonction
% "affichage_robot()" - 1ère Version (pas dans le dossier)

% affiche_robot(r,a,b,c)
%   r = nom du robot
%   a = affichage des repères
%   b = conserver l'affichage
%   c = nouvelle figure

affiche_robot(r3,0,0,0);
title('Robot r3');
affiche_robot(r4,0,0,1);
title('Robot r4');


% 3) Nouveau r3 avec [pi/2 0 0]'

r3_new = declare_robot(3,[0.5 0.3 0.2],[pi/2 0 0]');
affiche_robot(r3,0,1,1);
affiche_robot(r3_new,0,1,0);
title('Mouvement du Robot r3 de [pi/2 0 0]');

% 2.2)
% 1) Appel de la fonction de la matrice de rotation avec "rotation_Z_etu(angle)"

disp('#################################');
disp('# Question 2.2.1                #');

% Angle en radian
angle1 = 0;
R1 = rotation_Z_etu(angle1)
angle2 = pi/2;
R2 = rotation_Z_etu(angle2)
angle3 = pi/4;
R3 = rotation_Z_etu(angle3)
angle4 = -5*pi/4;
R4 = rotation_Z_etu(angle4)


disp('#                               #');
disp('#################################');

% 2) Vérification de la fonction "inv_rotation_z_etu(R)"
disp('#################################');
disp('# Question 2.2.2                #');

angle_deduit1 = inv_rotation_Z_etu(R1)
angle_deduit2 = inv_rotation_Z_etu(R2)
angle_deduit3 = inv_rotation_Z_etu(R3)
angle_deduit4 = inv_rotation_Z_etu(R4)


disp('#                               #');
disp('#################################');

% 3) Vérification de la fonction "trans_homogene_etu(R,T)"

disp('#################################');
disp('# Question 2.2.3                #');

T1 = [3,3,3]';
trans_homogene_etu(R1,T1)
T2 = [7,7,7]';
trans_homogene_etu(R2,T2)

disp('#                               #');
disp('#################################');

% 2.3)
% 1)
% 2)Vérification de la fonction "mod_geo_dir_etu(robot,segment)" qui nous
% envoient [TH1, TH1p]
%    TH1p = H prime avec rotation et translation
%    TH1 = H avec rotation et translation
%    segment = numéro de segment

disp('#################################');
disp('# Question 2.3.2                #');

[TH1_r3,TH1p_r3] = mod_geo_dir_etu(r3,3)
[TH1_r3_cor,TH1p_r3_cor] = mod_geo_dir(r3,3)
[TH1_r3_new,TH1p_r3_new] = mod_geo_dir_etu(r3_new,3)
[TH1_r3_new_cor,TH1p_r3_new_cor] = mod_geo_dir(r3_new,3)
[TH1_r4,TH1p_r4] = mod_geo_dir_etu(r4,3)
[TH1_r4_cor,TH1p_r4_cor] = mod_geo_dir(r4,3)

disp('#                               #');
disp('#################################');

% 3)Vérification de la fonction "mod_geo_inv_3Rplan_etu(robot,x,y,theta)" qui nous
% envoient [q1, q2]
disp('#################################');
disp('# Question 2.3.3                #');

[q1_r3_t,q2_r3_t] = mod_geo_inv_3Rplan_etu(r3,1,1,0)
[q1_r3_t_cor,q2_r3_t_cor] = mod_geo_inv_3Rplan(r3,1,1,0)
[q1_r3_r,q2_r3_r] = mod_geo_inv_3Rplan_etu(r3,0,0,pi/2)
[q1_r3_r_cor,q2_r3_r_cor] = mod_geo_inv_3Rplan(r3,0,0,pi/2)
[q1_r3_tr,q2_r3_tr] = mod_geo_inv_3Rplan_etu(r3,5,5,pi/6)
[q1_r3_tr_cor,q2_r3_tr_cor] = mod_geo_inv_3Rplan(r3,5,5,pi/6)

disp('#                               #');
disp('#################################');

% 4) Vérification de MGD et MGI pour un exemple trivial et un exemple non trivial 
disp('#################################');
disp('# Question 2.3.4                #');

% Exemple Trivial
r3_trivial = declare_robot(3,[0.5 0.3 0.2],[0 0 0]');
[TH1_r3_trivial,TH1p_r3_trivial] = mod_geo_dir_etu(r3_trivial,3)
[q1_r3_trivial,q2_r3_trivial] = mod_geo_inv_3Rplan_etu(r3_trivial,TH1_r3_trivial(1,4),TH1_r3_trivial(1,4),0)

% Exemple Non Trivial
r3_nontrivial = declare_robot(3,[0.5 0.3 0.2],[pi/5 pi/3 pi/2]');
[TH1_r3_nontrivial,TH1p_r3_nontrivial] = mod_geo_dir_etu(r3_nontrivial,3)
[q1_r3_nontrivial,q2_r3_nontrivial] = mod_geo_inv_3Rplan_etu(r3_nontrivial,1,1,0)

disp('#                               #');
disp('#################################');

%% TP 2
clear all;
close all;

%Initialization
COM = 31;          % COM number
BAUD = 1000000;   % COM baud rate

% Constructor call
robot = AX12(COM,BAUD);

% Quelques exemples :
% ID=6;
% robot.setPosition(ID,0);        % On place le moteur en 0
% robot.setSpeed(ID,300);         % Vitesse de déplacement = 33.3rmp
% isMoving=robot.getMoving(ID);   % Mouvement en cours ?
% pos=rrobot.setPosition(93,10)robot.setPosition(93,10)robot.setPosition(93,10)obot.getPosition(ID);      % Lecture de la position
% sp=robot.getCurrentSpeed(ID);   % Lecture de la vitesse

% Spécifier ici l'ID du moteur à contrôler
ID1 = 93; % Moteur 1
ID2 = 36; % Moteur 2
ID3 = 37; % Moteur 3

% 2.2.7 - 11)
disp('#################################');
disp('# Question 2.2.7 - 11           #');
offset = 512;
step = 512;
q = step2angle(step,offset);
angle2step(q,offset);

v = [300, 100, 400, 200, 50]; % v : vitesse
s = [1023, 0, 1023, 0, 1023]; % s : nombre de step qu'on veut faire

robot.setSpeed(ID3,100);
robot.setPosition(ID3,0);
pause(10);

tic
for i = 1:5
 robot.setSpeed(ID3,v(i));
 robot.setPosition(ID3,s(i));
 pos = [];
 time = [];
 tint = toc;
 pint = robot.getPosition(ID3);
 while (robot.getMoving(ID3))
  time = [time toc];
  pos = [pos robot.getPosition(ID3)];
  posq = pos*(300/360*2*pi)/1024;
  tint = toc;
  pint = robot.getPosition(ID3);
 end
 
 % Figures pour 2.2.11)
 figure(1);
 hold on;
 plot(time,pos);
 xlabel('temps(s)');
 ylabel('position en step');
 title('Relation de l evolution de la position(step) en fonction du temps');
 
 figure(2);
 hold on;
 plot(time,posq);
 xlabel('temps(s)');
 ylabel('position en radian');
 title('Relation de l evolution de la position(rad) en fonction du temps');
end

% Réinitialiser la position du robot
robot.setPosition(ID3,512);
pause(10);

disp('#                               #');
disp('#################################');

% 2.2.12 - 14)
disp('#################################');
disp('# Question 2.2.12 - 14          #');
offset = 512;
step = 512;
q = step2angle(step,offset);
angle2step(q,offset);

v = [200, 100, 400, 50, 300]; % v : vitesse
s = [800, 800, 800, 800, 800]; % s : nombre de step qu'on veut faire

% Vérification de l'accessibilité des positions
for j = 1:5
 limit(s(j),0,1024);
end

tic
for i = 1:5
 robot.setSpeed(ID2,v(i));
 robot.setPosition(ID2,200);
 pause(500/v(i));
 robot.setPosition(ID2,s(i));
 pos = [];
 time = [];
 vitc = [];
 vit = [];
 vitdiff = [];
 tint = toc;
 pint = robot.getPosition(ID2);
 while (robot.getMoving(ID2))
  time = [time toc];
  pos = [pos robot.getPosition(ID2)];
  vitc = [vitc val2speed(v(i))];
  vit = [vit val2speed(robot.getCurrentSpeed(ID2))];
  posq = pos*300/360*2*pi/1024;
  % Vitesse de rotation du moteur par différence finie
  vitdiff = [vitdiff val2speed(abs((robot.getPosition(ID2)-pint)/(2*(toc-tint))))];
  tint = toc;
  pint = robot.getPosition(ID2);
 end
 
 % Figures pour 2.2.11 et 18)
 figure(3);
 hold on;
 plot(time,pos);
 xlabel('temps(s)');
 ylabel('position en step');
 title('Relation de l evolution de la position(step) en fonction du temps');
 
 figure(4);
 hold on;
 plot(time,posq);
 xlabel('temps(s)');
 ylabel('position en radian');
 title('Relation de l evolution de la position(rad) en fonction du temps');
 
 % Figures pour 2.2.12 et 13)
 figure(5);
 hold on;
 plot(time,vitc);
 xlabel('temps(s)');
 ylabel('vitesse(rpm)');
 axis ([0,60,0,50]);
 title('Relation de l evolution de la vitesse de consigne en fonction du temps');
 
 figure(6);
 hold on;
 plot(time,vit);
 xlabel('temps(s)');
 ylabel('vitesse(rpm)');
 axis ([0,60,0,50]);
 title('Relation de l evolution de la vitesse recuperee en fonction du temps');
 
 % Figure pour 2.2.14)
 figure(7);
 hold on;
 plot(time,vitdiff);
 xlabel('temps(s)');
 ylabel('vitesse(rpm)');
 axis ([0,60,0,50]);
 title('Relation de l evolution de la vitesse par différence finie en fonction du temps');
 
 pause(500/v(i));

end

% Reinitialisation de la position du robot
 robot.setPosition(ID1,512);
 robot.setPosition(ID2,512);
 robot.setPosition(ID3,512);
 
disp('#                               #');
disp('#################################');

% 2.2.15)
disp('#################################');
disp('# Question 2.2.15               #');
qinit = 400;
qfinal = 600;

v = [50, 100, 150, 200, 250, 300]; % v : vitesse

errq_moy3 = [0 0 0 0 0 0];

for i = 1:6
 robot.setSpeed(ID1,v(i));
 robot.setSpeed(ID2,v(i));
 robot.setSpeed(ID3,v(i));
 errq_moy = [0 0 0];
 % On essaye 3 fois
 for j = 1:3
    robot.setPosition(ID1,qinit);
    robot.setPosition(ID2,qinit);
    robot.setPosition(ID3,qinit);
    
    pause(5);
    
    robot.setPosition(ID1,qfinal);
    robot.setPosition(ID2,qfinal);
    robot.setPosition(ID3,qfinal);
    
    % On récupére les variables articulaires réelle du robot
    qint = [robot.getPosition(ID1), robot.getPosition(ID2), robot.getPosition(ID3)]
    
    % On calcul l'erreur de chaque q : errq = qfinal - qint
    errq = abs(qint - (1024-qfinal))
    % On fait une moyenne des erreurs de q (vecteur -> scalaire)
    errq_moy(j) = (errq(1)+errq(2)+errq(3))/3
    pause(5);
 end
 % On fait une moyenne de l'erreur moyenne errq_moy pour des 3 essais
 errq_moy3(i) = (errq_moy(1)+errq_moy(2)+errq_moy(3))/3 
 %pause(3);
end
 
figure(8);
hold on;
plot(val2speed(v),errq_moy3,'o');
xlabel('vitesse(rpm)');
ylabel('erreur de la position en step');
title('Relation de l evolution de l erreur de la position(step) en fonction de la vitesse');

figure(9);
hold on;
plot(val2speed(v),step2angle(errq_moy3,0),'o');
xlabel('vitesse(rpm)');
ylabel('erreur de la position en radian');
title('Relation de l evolution de l erreur de la position(rad) en fonction de la vitesse');

figure(10);
hold on;
plot(val2speed(v),step2angle(errq_moy3,0)/(2*pi)*360,'o');
xlabel('vitesse(rpm)');
ylabel('erreur de la position en degre');
title('Relation de l evolution de l erreur de la position(deg) en fonction de la vitesse');

disp('#                               #');
disp('#################################');

% 2.2.18)
disp('#################################');
disp('# Question 2.2.18               #');
offset = 512;
step = 512;
q = step2angle(step,offset);
angle2step(q,offset);

v = [300, 100, 400, 200, 50]; % v : vitesse
s = [1023, 0, 1023, 0, 1023]; % s : nombre de step qu'on veut faire

robot.setSpeed(ID1,100);
robot.setPosition(ID1, limit(0,312,712));
pause(10);

% Vérification de l'accessibilité des positions
for j = 1:5
 s(j) = limit(s(j),312,712);
end

s

tic
for i = 1:5
 robot.setSpeed(ID1,v(i));
 robot.setPosition(ID1,s(i));
 pos = [];
 time = [];
 tint = toc;
 pint = robot.getPosition(ID1);
 while (robot.getMoving(ID1))
  time = [time toc];
  pos = [pos robot.getPosition(ID1)];
  posq = pos*(300/360*2*pi)/1024;
  tint = toc;
  pint = robot.getPosition(ID1);
 end
 
 % Figures pour 2.2.18)
 figure(11);
 hold on;
 plot(time,pos);
 xlabel('temps(s)');
 ylabel('position en step');
 title('Relation de l evolution de la position(step) en fonction du temps');
 
 figure(12);
 hold on;
 plot(time,posq);
 xlabel('temps(s)');
 ylabel('position en radian');
 title('Relation de l evolution de la position(rad) en fonction du temps');
end

% Réinitialiser la position du robot
robot.setPosition(ID1,512);
pause(10);

disp('#                               #');
disp('#################################');

%% TP 3 et 4 - Projet
clear all;
close all;

%  Initialization
COM = 31;          % COM number
BAUD = 1000000;   % COM baud rate

% Constructor call
robot = AX12(COM,BAUD);

% ID du moteur à contrôler
ID1 = 93; % Moteur 1
ID2 = 36; % Moteur 2
ID3 = 37; % Moteur 3

% 2.3) Déclaration et affichage du robot réel r3 à l'aide de la fonction
% "declare_robot()" et "affichage_robot()"

% "declare_robot(n,l,config)"
%   n = numéro de type de plan
%   l = longueurs des segments en mètre
%   config = configuration initiale

% "affiche_robot(r,a,b,c,d,e)" - 2ème Version (dans le dossier)
%   r = nom du robot
%   a = affichage des repères
%   b = conserver l'affichage
%   c = nouvelle figure
%   d = couleur en [r,g,b] entre 0 et 1
%   e = trajectoire : 
%       0: non et la liste de points "trajectoire" est effacée, 
%       1: oui et la liste de points "trajectoire" est tracée

% q de notre choix : q1=-pi/3, q2=pi/3 q3=pi/3
r3 = declare_robot(3,[0.68 0.68 0.68],[-pi/3 pi/3 pi/3]');
affiche_robot(r3,0,0,0,[1,0,0],0);
xlabel('position de x (dm)');
ylabel('position de y (dm)');
title('Robot r3');

% 2.4)
disp('#################################');
disp('# Question 2.4                  #');
% "mod_geo_dir_etu(robot,segment)" qui nous envoie [TH1, TH1p]
%    TH1p = H prime avec rotation et translation
%    TH1 = H avec rotation et translation
%    segment = numéro de segment

[TH,THp] = mod_geo_dir_etu(r3,3)
x = TH(1,4)
y = TH(2,4)
angle = inv_rotation_Z_etu(TH) % en radian
disp('#                               #');
disp('#################################');

% 2.4.a) On change nos valeurs de q en step
disp('#################################');
disp('# Question 2.4.a)               #');
offset = 512;
qa_d = angle2step(-r3.q(1),offset)
qb_d = angle2step(-r3.q(2),offset)
qc_d = angle2step(-r3.q(3),offset)
disp('#                               #');
disp('#################################');

% 2.4.b) Limite du débattement angulaire des actionneurs
%        (on fait une saturation)
%        en step
disp('#################################');
disp('# Question 2.4.b)               #');
steplimit = 60/300*1024;
val_min = steplimit;
val_max = 1024 - steplimit;

% Angles pour tester la limite 
% qa_d = angle2step(-r3.q(1),offset) + 1024;
% qb_d = angle2step(-r3.q(2),offset) + 512;
% qc_d = angle2step(-r3.q(3),offset) - 1000;

qa_d = limit(qa_d,val_min,val_max);
qb_d = limit(qb_d,val_min,val_max);
qc_d = limit(qc_d,val_min,val_max);
disp('#                               #');
disp('#################################');

% 2.4.c)
disp('#################################');
disp('# Question 2.4.c)               #');
% Vérification de la pose atteinte pour MGD
v = 100; % v : vitesse

% Mouvement de ID1 avec qa_d
robot.setSpeed(ID1,v);
%%robot.setPosition(ID1,qa_d);

% Mouvement de ID2 avec qb_d
robot.setSpeed(ID2,v);
%%robot.setPosition(ID2,qb_d);

% Mouvement de ID3 avec qc_d
robot.setSpeed(ID3,v);
%%robot.setPosition(ID3,qc_d);

% Lancer un mouvement simulané pour MGD
robot.setMultiplePositions([ID1,ID2,ID3],[qa_d,qb_d,qc_d])
pause();

affiche_robot(r3,0,0,0,[1,0,0],0);
xlabel('position de x (dm)');
ylabel('position de y (dm)');
title('Robot r3 MGD');
disp('#                               #');
disp('#################################');

% 2.6)
disp('#################################');
disp('# Question 2.6                  #');
% "mod_geo_inv_3Rplan_etu(robot,x,y,theta)" qui nous envoie [q1,q2]
% On a récupéré les valeurs de x,y,angle de MGD : x=1.36, y=0, angle=1.0472 
[q1,q2] = mod_geo_inv_3Rplan_etu(r3,1.36,0,1.0472)

% Etape 1 : utilisation de q1
r3.q = q1;
affiche_robot(r3,0,0,1,[1,0,0],0);
xlabel('position de x (dm)');
ylabel('position de y (dm)');
title('Robot r3 MGI pour q1');

% Avec la simulation, on a vu que les deux méthodes sont cohérentes, 
% car elles nous donnent la même forme.

% Vérification de la pose atteinte pour MGI
% On change nos valeurs de q1 en step
qa_i_1 = angle2step(-r3.q(1),offset)
qb_i_1 = angle2step(-r3.q(2),offset)
qc_i_1 = angle2step(-r3.q(3),offset)

qa_i_1 = limit(qa_i_1,val_min,val_max);
qb_i_1 = limit(qb_i_1,val_min,val_max);
qc_i_1 = limit(qc_i_1,val_min,val_max);

% Lancer un mouvement simulané pour MGI avec q1
robot.setMultiplePositions([ID1,ID2,ID3],[qa_i_1,qb_i_1,qc_i_1])
pause();

% Etape 2 : utilisation de q2
r3.q = q2;
affiche_robot(r3,0,0,1,[1,0,0],0);
xlabel('position de x (dm)');
ylabel('position de y (dm)');
title('Robot r3 MGI pour q2');

% Avec la simulation, on a vu que les deux méthodes ne sont pas cohérentes, 
% car elles nous ne donnent pas la même forme.

% Vérification de la pose atteinte pour MGI
% On change nos valeurs de q2 en step
qa_i_2 = angle2step(-r3.q(1),offset)
qb_i_2 = angle2step(-r3.q(2),offset)
qc_i_2 = angle2step(-r3.q(3),offset)

qa_i_2 = limit(qa_i_2,val_min,val_max);
qb_i_2 = limit(qb_i_2,val_min,val_max);
qc_i_2 = limit(qc_i_2,val_min,val_max);

% Lancer un mouvement simulané pour MGI avec q2
robot.setMultiplePositions([ID1,ID2,ID3],[qa_i_2,qb_i_2,qc_i_2])
pause();

% Avec q1 nous donne la configuration attendue,
% ceci a les mêmes trajectoires que celle de MGD, celle de MGD,
% par contre q2 nous donne une configuration inattendue
disp('#                               #');
disp('#################################');

% 3.
% 3.1) Trajectoire dans l'espace articulaire
disp('#################################');
disp('# Question 3.1 Esp. Articulaire #');

r3 = declare_robot(3,[0.68 0.68 0.78],[0 0 0]');
qinit = [0,0,0];

%qinit = [robot.getPosition(ID1),robot.getPosition(ID2),robot.getPosition(ID3)]

% Vérification de la pose atteinte pour MGD
v = 100; % v : vitesse
% Mouvement de ID1 avec qa_d
robot.setSpeed(ID1,v);
% Mouvement de ID2 avec qb_d
robot.setSpeed(ID2,v);
% Mouvement de ID3 avec qc_d
robot.setSpeed(ID3,v);

posfin = [0,2.14,pi/2];   
[q1fin,q2fin] = mod_geo_inv_3Rplan_etu(r3,posfin(1),posfin(2),posfin(3));

% On choisit qfin avec notre fonction erreurQ qui détermine le bonne
% qfin en fonction de notre critère personalisée (de faire des mouvements minimaux).
qfin = erreurQ(qinit,q1fin,q2fin);

nb_pas = 10;

pas = (qfin - qinit)/nb_pas;

qint = qinit;

offset = 512;
steplimit = 60/300*1024;
val_min1 = steplimit;
val_max1 = 1024 - steplimit;
val_min = 0;
val_max = 1024;

affiche_robot(r3,0,1,1,[1,0,0],0);

for i=1:nb_pas+1
    r3.q = qint;
    affiche_robot(r3,0,1,0,[1,0,0],0);
    xlabel('position de x (dm)');
    ylabel('position de y (dm)');
    title('Robot r3 qint');
    
    for j=1:3        
        % On change nos valeurs de q : radian en step
        qint(j) = angle2step(-qint(j),offset);
    end
    
    % Limite du débattement angulaire des actionneurs en step
    qint(1) = limit(qint(1),val_min1,val_max1);
    qint(2) = limit(qint(2),val_min,val_max);
    qint(3) = limit(qint(3),val_min,val_max);
    
    r3.q = qint;
    % Lancer un mouvement simulané
    robot.setMultiplePositions([ID1,ID2,ID3],[qint(1),qint(2),qint(3)])
 
    for k=1:3
        % On rechange nos valeurs de q : step en radian
        qint(k) = -step2angle(qint(k),offset);
        % Prochaine iteration
        qint(k) = qint(k) + pas(k);
    end

    pause();
end

r3.q = qinit;
affiche_robot(r3,0,0,1,[1,0,0],0);
xlabel('position de x (dm)');
ylabel('position de y (dm)');
title('Robot r3 qinit');

r3.q = qfin;
affiche_robot(r3,0,0,1,[1,0,0],0);
xlabel('position de x (dm)');
ylabel('position de y (dm)');
title('Robot r3 qfin');

% Retour
disp('#           Retour              #');

qinit = qfin;

posfin = [2.14,0,0];   
[q1fin,q2fin] = mod_geo_inv_3Rplan_etu(r3,posfin(1),posfin(2),posfin(3));

% On choisit qfin avec notre fonction erreurQ qui détermine le bonne
% qfin en fonction de notre critère personalisée (de faire des mouvements minimaux).
qfin = erreurQ(qinit,q1fin,q2fin);

nb_pas = 10;

pas = (qfin - qinit)/nb_pas;

qint = qinit;

affiche_robot(r3,0,0,1,[1,0,0],0);

for i=1:nb_pas+1
    r3.q = qint;
    affiche_robot(r3,0,1,0,[1,0,0],0);
    xlabel('position de x (dm)');
    ylabel('position de y (dm)');
    title('Robot r3 qint');
    
    for j=1:3        
        % On change nos valeurs de q : radian en step
        qint(j) = angle2step(-qint(j),offset);
    end
    
    % Limite du débattement angulaire des actionneurs en step
    qint(1) = limit(qint(1),val_min1,val_max1);
    qint(2) = limit(qint(2),val_min,val_max);
    qint(3) = limit(qint(3),val_min,val_max);
    
    r3.q = qint;
    % Lancer un mouvement simulané
    robot.setMultiplePositions([ID1,ID2,ID3],[qint(1),qint(2),qint(3)])
 
    for k=1:3
        % On rechange nos valeurs de q : step en radian
        qint(k) = -step2angle(qint(k),offset);
        % Prochaine iteration
        qint(k) = qint(k) + pas(k);
    end

    pause();
end

r3.q = qinit;
affiche_robot(r3,0,0,1,[1,0,0],0);
xlabel('position de x (dm)');
ylabel('position de y (dm)');
title('Robot r3 qinit');

r3.q = qint;
affiche_robot(r3,0,0,1,[1,0,0],0);
xlabel('position de x (dm)');
ylabel('position de y (dm)');
title('Robot r3 qfin');

disp('#                               #');
disp('#################################');

% 3.2) Trajectoire dans l’espace opérationnel
disp('#################################');
disp('# Question 3.2 Esp. Operationnel#');

r3 = declare_robot(3,[0.68 0.68 0.78],[0 0 0]');

% Vérification de la pose atteinte pour MGD
v = 100; % v : vitesse
% Mouvement de ID1 avec qa_d
robot.setSpeed(ID1,v);
% Mouvement de ID2 avec qb_d
robot.setSpeed(ID2,v);
% Mouvement de ID3 avec qc_d
robot.setSpeed(ID3,v);

% qinit = [robot.getPosition(ID1),robot.getPosition(ID2),robot.getPosition(ID3)];
% [TH,THp] = mod_geo_dir_etu(r3,3);
% x = TH(1,4);
% y = TH(2,4);
% angle = inv_rotation_Z_etu(TH); % en radian
% posinit = [x,y,angle];

% On choisit les bonnes q avec notre fonction erreurQ qui détermine 
% en fonction de notre critère personalisée (de faire des mouvements minimaux).

posinit = [2.14,0,0];
[q1init,q2init] = mod_geo_inv_3Rplan_etu(r3,posinit(1),posinit(2),posinit(3));
qinit = erreurQ([0,0,0],q1init,q2init);

r3.q = qinit;

posfin = [0,2.14,pi/2];
[q1fin,q2fin] = mod_geo_inv_3Rplan_etu(r3,posfin(1),posfin(2),posfin(3));
qfin = erreurQ(qinit,q1fin,q2fin);

nb_pas = 10;

paspos = (posfin - posinit)/nb_pas;

posint = posinit;
qint = qinit;

offset = 512;
steplimit = 60/300*1024;
val_min1 = steplimit;
val_max1 = 1024 - steplimit;
val_min = 0;
val_max = 1024;

for i=1:nb_pas+1
    r3.q = qint;
    affiche_robot(r3,0,1,0,[1,0,0],0);
    xlabel('position de x (dm)');
    ylabel('position de y (dm)');
    title('Robot r3 qint');
    
    for j=1:3
        % Prochaine iteration
        posint(j) = posint(j) + paspos(j);
    end
    [q1int,q2int] = mod_geo_inv_3Rplan_etu(r3,posint(1),posint(2),posint(3));
    qint = erreurQ(qint,q1int,q2int);

    for k=1:3        
        % On change nos valeurs de q : radian en step
        qint(k) = angle2step(-qint(k),offset);
    end

    % Limite du débattement angulaire des actionneurs en step
    qint(1) = limit(qint(1),val_min1,val_max1);
    qint(2) = limit(qint(2),val_min,val_max);
    qint(3) = limit(qint(3),val_min,val_max);
    
    r3.q = qint;
    
    % Lancer un mouvement simulané
    robot.setMultiplePositions([ID1,ID2,ID3],[qint(1),qint(2),qint(3)])

    for l=1:3
        % On rechange nos valeurs de q : step en radian
        qint(l) = -step2angle(qint(l),offset);
    end
    
    pause();
end

r3.q = qinit;
affiche_robot(r3,0,0,1,[1,0,0],0);
xlabel('position de x (dm)');
ylabel('position de y (dm)');
title('Robot r3 qinit');

r3.q = qfin;
affiche_robot(r3,0,0,1,[1,0,0],0);
xlabel('position de x (dm)');
ylabel('position de y (dm)');
title('Robot r3 qfin');

disp('#                               #');
disp('#################################');

%% Tâche d'Ecriture de la Lettre M - Projet
clear all;
close all;

%  Initialization
COM = 31;          % COM number
BAUD = 1000000;   % COM baud rate

% Constructor call
robot = AX12(COM,BAUD);

% ID du moteur à contrôler
ID1 = 93; % Moteur 1
ID2 = 36; % Moteur 2
ID3 = 37; % Moteur 3

% Vérification de la pose atteinte pour MGD
v = 300; % v : vitesse
% Mouvement de ID1 avec qa_d
robot.setSpeed(ID1,v);
% Mouvement de ID2 avec qb_d
robot.setSpeed(ID2,v);
% Mouvement de ID3 avec qc_d
robot.setSpeed(ID3,v);
nb_pas = 300;

offset = 512;
steplimit = 60/300*1024;

% Valeur min et max pour moteur 1
val_min1 = steplimit;
val_max1 = 1024 - steplimit;
% Valeur min et max pour moteur 2 et 3
val_min = 0;
val_max = 1024;

xpos = [ ];
ypos = [ ];

r3 = declare_robot(3,[0.68 0.68 0.78],[0 0 0]');

posA0 = [1.5,1,pi/3];
[q1A0,q2A0] = mod_geo_inv_3Rplan_etu(r3,posA0(1),posA0(2),posA0(3));
qA0 = erreurQ([0,0,0],q1A0,q2A0);
r3.q = qA0;
posA1 = [1.25,1.5,pi/3];
paspos = (posA1 - posA0)/nb_pas;
posint = posA0;
qint = qA0;

for i=1:nb_pas
    r3.q = qint;
    affiche_robot(r3,0,1,0,[1,0,0],1);
    xlabel('position de x (dm)');
    ylabel('position de y (dm)');
    title('Lettre M - Robot r3 qint');
        xpos = [xpos posint(1)];
        ypos = [ypos posint(2)];
    for j=1:3
        % Prochaine iteration
        posint(j) = posint(j) + paspos(j);
    end
    [q1int,q2int] = mod_geo_inv_3Rplan_etu(r3,posint(1),posint(2),posint(3));
    qint = erreurQ(qint,q1int,q2int);
    for k=1:3        
        % On change nos valeurs de q : radian en step
        qint(k) = angle2step(-qint(k),offset);
    end
    % Limite du débattement angulaire des actionneurs en step
    qint(1) = limit(qint(1),val_min1,val_max1);
    qint(2) = limit(qint(2),val_min,val_max);
    qint(3) = limit(qint(3),val_min,val_max);
    r3.q = qint;
    % Lancer un mouvement simulané
   robot.setMultiplePositions([ID1,ID2,ID3],[qint(1),qint(2),qint(3)]);
if i == 1
   pause();
end
    for l=1:3
        % On rechange nos valeurs de q : step en radian
        qint(l) = -step2angle(qint(l),offset);
    end
end

pause(2);
 
posA2 = [1,1,5*pi/8];
paspos = (posA2 - posint)/nb_pas; 

for i=1:nb_pas
    r3.q = qint;
    affiche_robot(r3,0,1,0,[1,1,0],1);
        xpos = [xpos posint(1)];
        ypos = [ypos posint(2)];
    for j=1:3
        % Prochaine iteration
        posint(j) = posint(j) + paspos(j);
    end
    [q1int,q2int] = mod_geo_inv_3Rplan_etu(r3,posint(1),posint(2),posint(3));
    qint = erreurQ(qint,q1int,q2int);
    for k=1:3        
        % On change nos valeurs de q : radian en step
        qint(k) = angle2step(-qint(k),offset);
    end
    % Limite du débattement angulaire des actionneurs en step
    qint(1) = limit(qint(1),val_min1,val_max1);
    qint(2) = limit(qint(2),val_min,val_max);
    qint(3) = limit(qint(3),val_min,val_max);
    r3.q = qint;
    % Lancer un mouvement simulané
   robot.setMultiplePositions([ID1,ID2,ID3],[qint(1),qint(2),qint(3)]);
    for l=1:3
        % On rechange nos valeurs de q : step en radian
        qint(l) = -step2angle(qint(l),offset);
    end
end

pause(2);

posA3 = [0.75,1.5,5*pi/8];
paspos = (posA3 - posint)/nb_pas;

for i=1:nb_pas
    r3.q = qint;
    affiche_robot(r3,0,1,0,[0,1,0],1);
        xpos = [xpos posint(1)];
        ypos = [ypos posint(2)];
    for j=1:3
        % Prochaine iteration
        posint(j) = posint(j) + paspos(j);
    end
    [q1int,q2int] = mod_geo_inv_3Rplan_etu(r3,posint(1),posint(2),posint(3));
    qint = erreurQ(qint,q1int,q2int);
    for k=1:3        
        % On change nos valeurs de q : radian en step
        qint(k) = angle2step(-qint(k),offset);
    end
    % Limite du débattement angulaire des actionneurs en step
    qint(1) = limit(qint(1),val_min1,val_max1);
    qint(2) = limit(qint(2),val_min,val_max);
    qint(3) = limit(qint(3),val_min,val_max);
    r3.q = qint;
    % Lancer un mouvement simulané
   robot.setMultiplePositions([ID1,ID2,ID3],[qint(1),qint(2),qint(3)]);
%   pause();
    for l=1:3
        % On rechange nos valeurs de q : step en radian
        qint(l) = -step2angle(qint(l),offset);
    end
end

pause(2);

posA4 = [0.5,1,5*pi/8];
paspos = (posA4 - posint)/nb_pas;

for i=1:nb_pas
    r3.q = qint;
    affiche_robot(r3,0,1,0,[0,0,1],1);
        xpos = [xpos posint(1)];
        ypos = [ypos posint(2)];
    for j=1:3
        % Prochaine iteration
        posint(j) = posint(j) + paspos(j);
    end
    [q1int,q2int] = mod_geo_inv_3Rplan_etu(r3,posint(1),posint(2),posint(3));
    qint = erreurQ(qint,q1int,q2int);
    for k=1:3        
        % On change nos valeurs de q : radian en step
        qint(k) = angle2step(-qint(k),offset);
    end
    % Limite du débattement angulaire des actionneurs en step
    qint(1) = limit(qint(1),val_min1,val_max1);
    qint(2) = limit(qint(2),val_min,val_max);
    qint(3) = limit(qint(3),val_min,val_max);
    r3.q = qint;
    % Lancer un mouvement simulané
   robot.setMultiplePositions([ID1,ID2,ID3],[qint(1),qint(2),qint(3)]);
%   pause();
    for l=1:3
        % On rechange nos valeurs de q : step en radian
        qint(l) = -step2angle(qint(l),offset);
    end
end

figure;
plot(xpos,ypos);
xlabel('position de x (dm)');
ylabel('position de y (dm)');
title('Lettre M');

end