function O2NCo=O2NCo(P,S)
%%% P=[x,y,a] Stand=[x0,y0,a0]
%%% return NP=[x1,y1,a1] in new coordinate Stand=[0,0,0]

TP=[];
NP=[];

TP(1)=P(1)-S(1);
TP(2)=P(2)-S(2);
TP(3)=P(3);

NP(1)=TP(1)*cos(S(3))+TP(2)*sin(S(3));
NP(2)=-TP(1)*sin(S(3))+TP(2)*cos(S(3));
NP(3)=TP(3)-S(3);

O2NCo=NP;
end