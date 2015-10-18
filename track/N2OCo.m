function N2OCo=N2OCo(P1,S)
% P1=[x1,y1,a1] Stand=[x0,y0,a0]
% return NP=[x1,y1,a1] in old coordinate 

NP(1)=P1(1)*cos(S(3))-P1(2)*sin(S(3));
NP(2)=P1(1)*sin(S(3))+P1(2)*cos(S(3));
NP(3)=P1(3)+S(3);

TP(1)=NP(1)+S(1);
TP(2)=NP(2)+S(2);
TP(3)=NP(3);

N2OCo=TP;
end