function qfinal = erreurQ(qinitial,q1,q2)
% On initialies les sommes des écarts entre les vecteurs q et q initial
somme1 = 0;
somme2 = 0;

% On calcule les somme des écarts respectivement
for i = 1:size(qinitial)
somme1 = somme1 + abs(q1(i)-qinitial(i));
somme2 = somme2 + abs(q2(i)-qinitial(i));
end

% On compare les sommes obtenues correspondantes aux vecteurs q différents
% et puis on prend qfinal
if somme1 >= somme2
    qfinal = q2;
    
else
    qfinal = q1;
end

end
    