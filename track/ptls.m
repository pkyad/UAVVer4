function ptls=ptls(UAV0,TAR0)
%% the direction of the moving target can change.
%% UAV0=[Ux,Uy,Ua]=loaction of UAV 
%% TAR0=[Tx,Ty,Ta]=location of targat, vt=v/R 
%% for example R=60, V=15..30, Vt=(0.4..0.8)*V
%% v<V 0<v<15 vt<0.25
R=1;
%V=0.27;
yE=1.651405412;
 
%% we only consider first 3 case, It means that k<k0

Ux=UAV0(1);
Uy=UAV0(2);
Ua=mod(UAV0(3),2*pi);
V=UAV0(4);
Tx=TAR0(1);
Ty=TAR0(2);
Ta=TAR0(3);
v=TAR0(4);

Tt=1;

k=V/v;

theta0=fzero(@(x) myfun(x,k),[0.00001 pi-0.00001]); %%%theta

%%%%% only consider k<k0=3.00, d has two cases largest value
if (theta0>0)&&(theta0<=pi/2)
    d=R*(1-cos(theta0));
else d=R*(1+cos(pi-theta0));
end
     
%%%%%basic graph%%%%%
t00=[-1*R:0.01:2*R];
x00=Tx+t00*cos(Ta);
y00=Ty+t00*sin(Ta);  %%%x-line


TTa=Ta+pi/2;
x01=d*cos(TTa)+Tx+t00*cos(Ta);
y01=d*sin(TTa)+Ty+t00*sin(Ta);  %%%upper basic line 
x02=-d*cos(TTa)+Tx+t00*cos(Ta);
y02=-d*sin(TTa)+Ty+t00*sin(Ta);  %%%lower basic line
OLx=Ux+R*cos(pi/2+Ua);
OLy=Uy+R*sin(pi/2+Ua);  %%%%center of left circle OL
ORx=Ux-R*cos(pi/2+Ua);
ORy=Uy-R*sin(pi/2+Ua);  %%%%center of right circle OR

% figure(1)
% plot(x00,y00,'-k');
% axis equal
% hold on
% 
% 
% figure(1)
% plot(Ux,Uy,'*r',Tx,Ty,'*r');
% axis equal
% hold on


t1=[-0.5:0.01:0.5];
Ux1=Ux+t1*cos(Ua);
Uy1=Uy+t1*sin(Ua);  %%%%%%%%%%alpha line

theta2=[0:0.01:2*pi];
x22=OLx+cos(theta2);
y22=OLy+sin(theta2);      %%%%circle OL
x21=ORx+cos(theta2);
y21=ORy+sin(theta2);      %%%%circle OR

% figure(1)
% plot(Ux,Uy,'*r',Tx,Ty,'*r')
% plot(Ux1,Uy1,'-k');
% plot(x01,y01,'-k',x02,y02,'-k');
% plot(OLx,OLy,'*b',ORx,ORy,'*b');
% %plot(OLx,OLy,'*b',ORx,ORy,'*b');
% plot(x21,y21,'-b',x22,y22,'-b');
% hold on



%%%%% change the coordinates into new one TAR0[0 0 0]
if (Tx==0)&&(Ty==0)&&(Ta==0)
    NU=[Ux Uy Ua];
    NT=[Tx Ty Ta];
else
    NU=O2NCo([Ux,Uy,Ua],[Tx,Ty,Ta]); % O2NCo: old to new coordinate
    NT=[0 0 0];
end
NUx=NU(1);NUy=NU(2);NUa=NU(3);
NTx=NT(1);NTy=NT(2);NTa=NT(3);
% NUx
% NUy
% NUa
% NUa+pi/2

NUx1=NUx+t1*cos(NUa);
NUy1=NUy+t1*sin(NUa);  %%%%%%%%%%alpha line

t01=[-2*R:0.01:4*R];
Nx00=NTx+t01;
Ny00=NTy+t01*tan(NTa);  %%%x-line
Nx01=NTx+t01;
Ny01=d+NTy;  %%%upper basic line 
Nx02=NTx+t01;
Ny02=-d+NTy;  %%%lower basic line
Ny01;
Ny02;

NOLx=NUx+R*cos(pi/2+NUa);
NOLy=NUy+R*sin(pi/2+NUa);  %%%%center of left circle OL
NORx=NUx-R*cos(pi/2+NUa);
NORy=NUy-R*sin(pi/2+NUa);  %%%%center of right circle OR

%  figure(2)
%  plot(NUx,NUy,'*r',NTx,NTy,'*r');
%  plot(NOLx,NOLy,'*b',NORx,NORy,'*b'); %circles center
%  plot(NUx1,NUy1,'-g'); %% alpha line
%  plot(Nx00,Ny00,'-b',Nx01,Ny01,'-b',Nx02,Ny02,'-b'); %%% x-line standard lines
%  axis equal
%  hold on
% % 

% figure(1),hold on
% 
% plot(x00,y00,'-k');
% plot(Ux,Uy,'*r',Tx,Ty,'*r');
% axis equal




%%%% [NUx NUy NUa] [NTx NTy v] [OLx OLy] [ORx ORy]
if NUy>=NTy %%%%% UAV in upper half space
    NOLx=NUx+R*cos(pi/2+NUa);
    NOLy=NUy+R*sin(pi/2+NUa);  %%%%center of left circle OL
    NORx=NUx-R*cos(pi/2+NUa);
    NORy=NUy-R*sin(pi/2+NUa);  %%%%center of right circle OR

    %%%%y-axis of the highest point
    NyLM=NOLy+R;
    NyRM=NORy+R;

    if (NUa>=0)&&(NUa<=pi/2)  %%%I
        Ny0=NyRM;
        if Ny0<=Ny01 %%%% LR
            NO3y=Ny01-R;
            NO3x=NOLx+sqrt(4*R^2-(NOLy-NO3y)^2);
            NA3x=(NOLx+NO3x)/2;NA3y=(NOLy+NO3y)/2;
            NA3a=mod(pi/2+atan((NO3y-NOLy)/(NO3x-NOLx)),2*pi);
            NA4x=NO3x;
            NA4y=Ny01;
            NA4a=0;
            Nu=[-1 1]; %%% -1 means turn left and 1 means turn right
            NUs(1)=R*abs(NUa+2*pi-NA3a);
            NUt(1)=NUs(1)/V;
            NUs(2)=R*abs(2*pi-NA3a);
            NUt(2)=NUs(2)/V;

        else %y0>y01   %%%%% RL
            NO3y=Ny01+R;
            NO3x=NORx+sqrt(4*R^2-(NORy-NO3y)^2);
            NA3x=(NORx+NO3x)/2;NA3y=(NORy+NO3y)/2;

            if NO3x==NORx&&NO3y>NORy
                temp3a=pi/2;
            elseif NO3x==NORx&&NO3y<NO3Ry
                temp3a=-pi/2;
            elseif NO3x>NORx;
                temp3a=atan((NO3y-NORy)/(NO3x-NORx));
            elseif NO3x<NORx
                temp3a=pi+atan((NO3y-NORy)/(NO3x-NORx));
            end
            NA3a=mod(-pi/2+temp3a,2*pi);
            NA4x=NO3x;
            NA4y=Ny01;
            NA4a=0;
            Nu=[1 -1];
            NUs(1)=R*abs(NA3a-NUa);
            NUt(1)=NUs(1)/V;
            NUs(2)=R*abs(NA3a);
            NUt(2)=NUs(2)/V;
        end
    elseif (NUa>=pi/2)&&(NUa<=pi)  %%%% II
        if NyRM<NTy+yE %%%turn right
            NO3x=NORx;NO3y=NORy;
            NA3x=NUx;NA3y=NUy;NA3a=NUa;
            NA4x=NORx;NA4y=NORy+R;
            NA4a=0;
            Nu=[1];
            NUs(1)=R*abs(NUa-NA4a);
            NUt(1)=NUs(1)/V;
        elseif NOLy>NTy  %%%%NyRM>=NTy+yE Left center higher than NTy, turn left
            NO3x=NOLx;NO3y=NOLy;
            NA3x=NUx;NA3y=NUy;NA3a=NUa;
            NA4y=NTy;
            NA4x=NOLx-sqrt(R^2-(NOLy-NTy)^2);
            if NOLx==NA4x&&NOLy>NA4y
                t0=pi/2;
            elseif NOLx==NA4x&&NOLy<NA4y
                t0=-pi/2;
            elseif NOLx>NA4x;
                t0=atan((NOLy-NA4y)/(NOLx-NA4x));
            elseif NOLx<NA4x
                t0=pi+atan((NOLy-NA4y)/(NOLx-NA4x));
            end
            NA4a=mod(-pi/2+t0,2*pi);
            Nu=[-1];
            NUs(1)=R*abs(NA4a-NUa);
            NUt(1)=NUs(1)/V;
        else  %NOLy<=NTy and NyRM>=NTy+yE Left center lower higher than NTy, turn left
            NO3x=NOLx;NO3y=NOLy; %%NO3=NOL critical circle=left circle
            NA3x=NUx;NA3y=NUy; NA3a=NUa; 
            NA4x=NOLx-R; NA4y=OLy;
            NA4a=3*pi/2;
            Nu=[-1];
            NUs(1)=R*abs(NA4a-NUa); %%total turn angle
            NUt(1)=NUs(1)/V; %% total time to turn such angle
        end
    elseif (NUa>=pi)&&(NUa<=3*pi/2)   %%III %%%OLy<=NTy turn left
        NO3x=NOLx;
        NO3y=NOLy; %%NO3=NOL critical circle=left circle
        NA3x=NUx;NA3y=NUy; NA3a=NUa;  %%% A3=NU
        NA4y=NTy; %%% point which circle and x-aixs intersect
        NA4x=NOLx-sqrt(R^2-(NOLy-NTy)^2);
        if NOLx==NA4x&&NOLy>NA4y%%%% A4-->OL
            t00=pi/2;
        elseif NOLx==NA4x&&NOLy<NA4y
            t00=-pi/2;
        elseif NOLx>NA4x;
            t00=atan((NOLy-NA4y)/(NOLx-NA4x));
        elseif NOLx<NA4x
            t00=pi+atan((NOLy-NA4y)/(NOLx-NA4x));
        end
        NA4a=mod(-pi/2+t00,2*pi);
        Nu=[-1]; %% turn left;
        NUs(1)=R*abs(NA4a-NUa); %total turn angle
        NUt(1)=NUs(1)/V; %% total time to turn such angle, may less than 1 second
        if NUt(1)>=Tt
        else
           NUt(1)=Tt;
           NUs(1)=V/R*NUt; %%% turn rate: trun angle in one second  
        end
        
        
    else  %(NUa>=3/2pi)&&(NUa<=2*pi)  %%%%IV
        Ny0=NyRM;
        if (Ny0>Ny01)  %y0>y01 %%%%% RL   the highest point of the right circle higher than the upper line
            NO3y0=Ny01+R;   %%%%new circle's center which is tangent the upper line, the circle is higher than the upper line
            NO3x0=NORx+sqrt(4*R^2-(NORy-NO3y0)^2); %%%%%%
            NA3x0=(NORx+NO3x0)/2;
            NA3y0=(NORy+NO3y0)/2;
            if NUy>=NA3y0 %%%% NUaV's location higher than the tangent point. NUaV can track the upper line
                NO3y=NO3y0;   %%%%upper line
                NO3x=NO3x0;
                NA3x=NA3x0;NA3y=NA3y0;
                if NO3x==NORx&&NO3y>NORy
                    temp3a=pi/2;
                elseif NO3x==NORx&&NO3y<NO3Ry
                    temp3a=-pi/2;
                elseif NO3x>NORx;
                    temp3a=atan((NO3y-NORy)/(NO3x-NORx));
                elseif NO3x<NORx
                    temp3a=pi+atan((NO3y-NORy)/(NO3x-NORx));
                end
                NA3a=mod(-pi/2+temp3a,2*pi);
                NA4x=NO3x;NA4y=Ny01;
                NA4a=0;
                Nu=[1 -1];
                NUs(1)=R*abs(NUa-NA3a);
                NUt(1)=NUs(1)/V;
                NUs(2)=R*abs(2*pi-NA3a);
                NUt(2)=NUs(2)/V;
            else %%%% NUaV's location lower than the tangent point. NUaV can track the lower line
                NO3y=Ny02+R; %%%%lower line
                NO3x=NORx+sqrt(4*R^2-(NORy-NO3y)^2);
                NA3x=(NORx+NO3x)/2;NA3y=(NORy+NO3y)/2;
                if NO3x==NORx&&NO3y>NORy
                    temp3a=pi/2;
                elseif NO3x==NORx&&NO3y<NO3Ry
                    temp3a=-pi/2;
                elseif NO3x>NORx;
                    temp3a=atan((NO3y-NORy)/(NO3x-NORx));
                else
                    temp3a=pi+atan((NO3y-NORy)/(NO3x-NORx));
                end
                NA3a=mod(-pi/2+temp3a,2*pi);
                NA4x=NO3x;NA4y=Ny02;
                NA4a=0;
                Nu=[1 -1];
                NUs(1)=R*abs(NUa-NA3a);
                NUt(1)=NUs(1)/V;
                NUs(2)=R*abs(2*pi-NA3a);
                NUt(2)=NUs(2)/V;
            end
        else %%%%% RL the highest point of the right circle lower than the upper line
            NO3y=Ny02+R; %%%% NUaV can only track lower line
            NO3x=NORx+sqrt(4*R^2-(NORy-NO3y)^2);
            NA3x=(NORx+NO3x)/2;NA3y=(NORy+NO3y)/2;
            if NO3x==NORx&&NO3y>NORy
                temp3a=pi/2;
            elseif NO3x==NORx&&NO3y<NO3Ry
                temp3a=-pi/2;
            elseif NO3x>NORx;
                temp3a=atan((NO3y-NORy)/(NO3x-NORx));
            else
                temp3a=pi+atan((NO3y-NORy)/(NO3x-NORx));
            end
            NA3a=mod(-pi/2+temp3a,2*pi);
            NA4x=NO3x;NA4y=Ny02;
            NA4a=0;
            Nu=[1 -1];
            NUs(1)=R*abs(NUa-NA3a);
            NUt(1)=NUs(1)/V;
            NUs(2)=R*abs(2*pi-NA3a);
            NUt(2)=NUs(2)/V;
        end
    end
    %%%% end NUy>=NTy
else %%NUy<NTy %% need turn up, UAV is in the lower half plane
    %%%% turn up
    NUy=2*NTy-NUy;
    NUa=mod(-NUa,2*pi);
    %%%% end turn up
    %%%% up
    NOLx=NUx+R*cos(pi/2+NUa);
    NOLy=NUy+R*sin(pi/2+NUa);  %%%%center of left circle OL(not real one)
    NORx=NUx-R*cos(pi/2+NUa);
    NORy=NUy-R*sin(pi/2+NUa);  %%%%center of right circle OR(not real one)
    
%     figure(3)
% plot(NUx,NUy,'*r',NTx,NTy,'*r');
% plot(NOLx,NOLy,'*b');
% plot(NORx,NORy,'*b');
% plot(NUx1,NUy1,'-g');
% plot(Nx00,Ny00,'-b',Nx01,Ny01,'-b',Nx02,Ny02,'-b');
% axis equal
% hold on
    
    %%%%y-axis of the highest point
    NyLM=NOLy+R;
    NyRM=NORy+R;

    if (NUa>=0)&&(NUa<=pi/2)  %%%I
        Ny0=NyRM;
        if Ny0<=Ny01 %%%% LR
            NO3y=Ny01-R;
            NO3x=NOLx+sqrt(4*R^2-(NOLy-NO3y)^2);
            NA3x=(NOLx+NO3x)/2;NA3y=(NOLy+NO3y)/2;
            if NO3x==NOLx&&NO3y>NOLy
                temp3a=pi/2;
            elseif NO3x==NOLx&&NO3y<NO3Ly
                temp3a=-pi/2;
            elseif NO3x>NOLx;
                temp3a=atan((NO3y-NOLy)/(NO3x-NOLx));
            else
                temp3a=pi+atan((NO3y-NOLy)/(NO3x-NOLx));
            end
            NA3a=mod(pi/2+temp3a,2*pi);
            %NA3a=mod(pi/2+atan((NO3y-NOLy)/(NO3x-NOLx)),2*pi);
            NA4x=NO3x;NA4y=Ny01;
            NA4a=0;
            Nu=[-1 1];
            NUs(1)=R*abs(NUa+2*pi-NA3a);
            NUt(1)=NUs(1)/V;
            NUs(2)=R*abs(2*pi-NA3a);
            NUt(2)=NUs(2)/V;
        else %Ny0>Ny01   %%%%% RL
            NO3y=Ny01+R;
            NO3x=NORx+sqrt(4*R^2-(NORy-NO3y)^2);
            NA3x=(NORx+NO3x)/2;NA3y=(NORy+NO3y)/2;
            if NO3x==NORx&&NO3y>NORy
                temp3a=pi/2;
            elseif NO3x==NORx&&NO3y<NO3Ry
                temp3a=-pi/2;
            elseif NO3x>NORx;
                temp3a=atan((NO3y-NORy)/(NO3x-NORx));
            else
                temp3a=pi+atan((NO3y-NORy)/(NO3x-NORx));
            end
            NA3a=mod(-pi/2+temp3a,2*pi);
            NA4x=NO3x;NA4y=Ny01;
            NA4a=0;
            Nu=[1 -1];
            NUs(1)=R*abs(NA3a-NUa);
            NUt(1)=NUs(1)/V;
            NUs(2)=R*abs(NA3a);
            NUt(2)=NUs(2)/V;
        end
    elseif (NUa>=pi/2)&&(NUa<=pi)  %%%% II
        if NyRM<NTy+yE 
            %%%% NyRM: y-axis of the highest point in the right circle
            %%%% yE: critical value of the distance.
            %%%% UAV moves along right circle
            NO3x=NORx;NO3y=NORy;
            NA3x=NUx;NA3y=NUy;NA3a=NUa;
            NA4x=NORx;NA4y=NORy+R;
            NA4a=0;
            Nu=[1];
            NUs(1)=R*abs(NUa-NA4a);
            NUt(1)=NUs(1)/V;
        elseif NOLy>NTy  %%%%Left center higher than NTy (target's y aixs)
            %%%% UAV moves along left circle
            NO3x=NOLx;NO3y=NOLy;
            NA3x=NUx;NA3y=NUy;NA3a=NUa;
            %[NOLx NOLy]
            %[NTx NTy]
            NA4y=NTy;
            NA4x=NOLx-sqrt(R^2-(NOLy-NTy)^2); %%%% complex?? distance is large. why???
            %[NA4x NA4y] i=24 bug
            if NOLx==NA4x&&NOLy>NA4y
                t0=pi/2;
            elseif NOLx==NA4x&&NOLy<NA4y
                t0=-pi/2;
            elseif NOLx>NA4x;
                t0=atan((NOLy-NA4y)/(NOLx-NA4x));
            else
                t0=pi+atan((NOLy-NA4y)/(NOLx-NA4x));
            end
            NA4a=mod(-pi/2+t0,2*pi);
            %NA4a=mod(-pi/2+atan((NOLy-NA4y)/(NOLx-NA4x)),2*pi);
            Nu=[-1];
            NUs(1)=R*abs(NA4a-NUa);
            NUt(1)=NUs(1)/V;
        else  %OLy<=NTy Left center lower than NTy(target's y aixs)
            NO3x=NOLx;NO3y=NOLy;
            NA3x=NUx;NA3y=NUy; NA3a=NUa;
            NA4x=NOLx-R; NA4y=NOLy;
            NA4a=3*pi/2;
            Nu=[-1];
            NUs(1)=R*abs(NA4a-NUa);
            NUt(1)=NUs(1)/V;
        end
    elseif (NUa>=pi)&&(NUa<=3*pi/2)   %%III %%%OLy>NTy
        NO3x=NOLx;NO3y=NOLy;
        NA3x=NUx;NA3y=NUy; NA3a=NUa;
        NA4y=NTy;
        NA4x=NOLx-sqrt(R^2-(NOLy-NTy)^2);
        if NOLx==NA4x&&NOLy>NA4y
            t0=pi/2;
        elseif NOLx==NA4x&&NOLy<NA4y
            t0=-pi/2;
        elseif NOLx>NA4x;
            t0=atan((NOLy-NA4y)/(NOLx-NA4x));
        else
            t0=pi+atan((NOLy-NA4y)/(NOLx-NA4x));
        end
        NA4a=mod(-pi/2+t0,2*pi);
        Nu=[-1];
        NUs(1)=R*abs(NA4a-NUa);
        NUt(1)=NUs(1)/V;
        if NUt(1)>=Tt
        else
           NUt(1)=Tt;
           NUs(1)=V/R*NUt; %%% turn rate: trun angle in one second  
        end
        
    else  %(NUa>=3/2pi)&&(NUa<=2*pi)  %%%%IV
        Ny0=NyRM;
        if (Ny0>Ny01)  %Ny0>Ny01 %%%%% RL   the highest point of the right circle higher than the upper line
            NO3y0=Ny01+R;   %%%%new circle's center which is tangent the upper line, the circle is higher than the upper line
            NO3x0=NORx+sqrt(4*R^2-(NORy-NO3y0)^2); %%%%%%
            NA3x0=(NORx+NO3x0)/2;
            NA3y0=(NORy+NO3y0)/2;
            if NUy>=NA3y0 %%%% NUaV's location higher than the tangent point. NUaV can track the upper line
                NO3y=NO3y0;   %%%%upper line
                NO3x=NO3x0;
                NA3x=NA3x0;NA3y=NA3y0;
                if NO3x==NORx&&NO3y>NORy
                    temp3a=pi/2;
                elseif NO3x==NORx&&NO3y<NO3Ry
                    temp3a=-pi/2;
                elseif NO3x>NORx;
                    temp3a=atan((NO3y-NORy)/(NO3x-NORx));
                else
                    temp3a=pi+atan((NO3y-NORy)/(NO3x-NORx));
                end
                NA3a=mod(-pi/2+temp3a,2*pi);
                NA4x=NO3x;NA4y=Ny01;
                NA4a=0;
                Nu=[1 -1];
                NUs(1)=R*abs(NUa-NA3a);
                NUt(1)=NUs(1)/V;
                NUs(2)=R*abs(2*pi-NA3a);
                NUt(2)=NUs(2)/V;
            else %%%% NUaV's location lower than the tangent point. NUaV can track the lower line
                NO3y=Ny02+R; %%%%lower line
                NO3x=NORx+sqrt(4*R^2-(NORy-NO3y)^2);
                NA3x=(NORx+NO3x)/2;NA3y=(NORy+NO3y)/2;
                if NO3x==NORx&&NO3y>NORy
                    temp3a=pi/2;
                elseif NO3x==NORx&&NO3y<NO3Ry
                    temp3a=-pi/2;
                elseif NO3x>NORx;
                    temp3a=atan((NO3y-NORy)/(NO3x-NORx));
                else
                    temp3a=pi+atan((NO3y-NORy)/(NO3x-NORx));
                end
                NA3a=mod(-pi/2+temp3a,2*pi);
                NA4x=NO3x;NA4y=Ny02;
                NA4a=0;
                Nu=[1 -1];
                NUs(1)=R*abs(NUa-NA3a);
                NUt(1)=NUs(1)/V;
                NUs(2)=R*abs(2*pi-NA3a);
                NUt(2)=NUs(2)/V;
            end
        else %%%%% RL the highest point of the right circle lower than the upper line
            NO3y=Ny02+R; %%%% NUaV can only track lower line
            NO3x=NORx+sqrt(4*R^2-(NORy-NO3y)^2);
            NA3x=(NORx+NO3x)/2;NA3y=(NORy+NO3y)/2;
            if NO3x==NORx&&NO3y>NORy
                temp3a=pi/2;
            elseif NO3x==NORx&&NO3y<NO3Ry
                temp3a=-pi/2;
            elseif NO3x>NORx;
                temp3a=atan((NO3y-NORy)/(NO3x-NORx));
            else
                temp3a=pi+atan((NO3y-NORy)/(NO3x-NORx));
            end
            NA3a=mod(-pi/2+temp3a,2*pi);
            NA4x=NO3x;NA4y=Ny02;
            NA4a=0;
            Nu=[1 -1];
            NUs(1)=R*abs(NUa-NA3a);
            NUt(1)=NUs(1)/V;
            NUs(2)=R*abs(2*pi-NA3a);
            NUt(2)=NUs(2)/V;
        end
    end
    %%%% end up

    %%%% turn back
    NOLy=2*NTy-NOLy;
    NORy=2*NTy-NORy;
    Ntempx=NOLx; Ntempy=NOLy;
    NOLx=NORx; NOLy=NORy;
    NORx=Ntempx; NORy=Ntempy;
    %%%%
    Nu=-Nu;
    NUy=2*NTy-NUy;
    NO3y=2*NTy-NO3y;
    NA3y=2*NTy-NA3y;
    NA4y=2*NTy-NA4y;
    NUa=mod(-NUa,2*pi);
    NA3a=mod(-NA3a,2*pi);
    NA4a=mod(-NA4a,2*pi);
    %%%% end turn back
end


if NA4a==0
    NA4a=2*pi;
else
end



Nx22=NOLx+cos(theta2);
Ny22=NOLy+sin(theta2);      %%%%circle OL
Nx21=NORx+cos(theta2);
Ny21=NORy+sin(theta2);      %%%%circle OR
Nx31=NO3x+R*cos(theta2);
Ny31=NO3y+R*sin(theta2);      %%%%circle O3


[NUx NUy NUa];
[NTx NTy v];
[NA3x NA3y NA3a];
[NA4x NA4y NA4a];
Nu;




NA3x1=NA3x+t1*cos(NA3a);
NA3y1=NA3y+t1*sin(NA3a);  %%%%%%%%%%A3 alpha line
NA4x1=NA4x+t1*cos(NA4a);
NA4y1=NA4y+t1*sin(NA4a);  %%%%%%%%%%A4 alpha line


NSt0=v*Tt; %%% Tt is the step length
NNTx=NTx+NSt0;
NNTy=NTy;

% %%%other alpha line
% figure(1)
% plot(A3x1,A3y1,'-b',A4x1,A4y1,'-b');
% hold on

nNu=size(Nu,2); %%% do the turn
% Us;
% Ut;
NUtt=0;
NUt;
for i=1:nNu
    NUtt=NUt(i)+NUtt;
end
 % u;
 NUtt;
NSt1=V*Tt;
Narc1=NSt1/R;
if NUtt<Tt %%%%need to move part standard curve
    Ntempt=Tt-NUtt; %%% not enough
    Narc0=Ntempt*V/R;
    if NA4y>=0
        NO4x=NA4x;NO4y=NA4y-R;
        NNUx=NO4x+R*cos(pi/2-Narc0);
        NNUy=NO4y+R*sin(pi/2-Narc0);
        NNUa=mod(Narc0,2*pi);
    else %%%%A4y<0
        NO4x=NA4x;NO4y=NA4y+R;
        NNUx=NO4x+R*cos(+Narc0-pi/2);
        NNUy=NO4y+R*sin(+Narc0-pi/2);
        NNUa=mod(Narc0,2*pi);
    end
    Nx41=NO4x+cos(theta2);
    Ny41=NO4y+sin(theta2);
%     figure(1)
%     %plot(O4x,O4y,'*b'); %%%% new critical points
%     plot(x41,y41,'-b'); %%%%%%%%% O4 circle
%     hold on
elseif nNu==1
    if Nu(1)==1
        NNUa=mod(NUa-Narc1,2*pi);
        NtempOx=NORx; NtempOy=NORy;
        NNUx=NtempOx+R*cos(NNUa+pi/2);
        NNUy=NtempOy+R*sin(NNUa+pi/2);
    else
        NNUa=mod(NUa+Narc1,2*pi);
        NtempOx=NOLx; NtempOy=NOLy;
        NNUx=NtempOx+R*cos(NNUa-pi/2);
        NNUy=NtempOy+R*sin(NNUa-pi/2);
    end
else %%%nNu==2
    if NUt(1)>=Tt
        if Nu(1)==1
            Nu(1);
            NNUa=mod(NUa-Narc1,2*pi);
            NtempOx=NORx; NtempOy=NORy;
            NNUx=NtempOx+R*cos(NNUa+pi/2);
            NNUy=NtempOy+R*sin(NNUa+pi/2);
        else
            NNUa=mod(NUa+Narc1,2*pi);
            NtempOx=NOLx; NtempOy=NOLy;
            NNUx=NtempOx+R*cos(NNUa-pi/2);
            NNUy=NtempOy+R*sin(NNUa-pi/2);
        end
    else %%% NUt(1)<Tt
        Ntempt=Tt-NUt(1);
        if Nu(2)==1
            NNUa=mod(NA3a-V/R*Ntempt,2*pi);
            NtempOx=NO3x; NtempOy=NO3y;
            NNUx=NtempOx+R*cos(NNUa+pi/2);
            NNUy=NtempOy+R*sin(NNUa+pi/2);
        else %%%Nu(2)==-1
            NNUa=mod(NA3a+V/R*Ntempt,2*pi);
            NtempOx=NO3x; NtempOy=NO3y;
            NNUx=NtempOx+R*cos(NNUa-pi/2);
            NNUy=NtempOy+R*sin(NNUa-pi/2);
        end
    end
end

[NNUx,NNUy];
[NNTx,NNTy];

%%% new points 
%%% [NOLx,NOLy] [NORx,NORy] [NO3x,NO3y] [NO4x,NO4y] 
%%% [NA3x,NA3y,NA3a] [NA4x,NA4y,NA4a] [NNUx NNUy NNUa]

%%%%%In new coordination
% figure(2)
% plot(NA3x,NA3y,'*g',NA4x,NA4y,'*g'); %%%% new critical points
% %plot(Nx01,Ny01,'-b',Nx02,Ny02,'-b'); %%%%%standard lines
% %plot(NA3x1,NA3y1,'-b',NA4x1,NA4y1,'-b'); %%%%%alpha lines
% %plot(Nx22,Ny22,'-b',Nx21,Ny21,'-b',Nx31,Ny31,'-b'); %%%%%%%%% R L circle
% %position of the old locations of UAV and Target
% %plot(NUx,NUy,'*r',NTx,NTy,'*r');  
% %position of new locations of UAV and Target
% %plot(NNTx,NNTy,'*b',NNUx,NNUy,'*b'); 
% hold on


%%% change back the old coordinates
A3=N2OCo([NA3x,NA3y,NA3a],[Tx,Ty,Ta]); %%%%A3
A4=N2OCo([NA4x,NA4y,NA4a],[Tx,Ty,Ta]); %%%%A4
 %%%%
O3=N2OCo([NO3x,NO3y,0],[Tx,Ty,Ta]);  %%%%center O3
x31=O3(1)+R*cos(theta2);
y31=O3(2)+R*sin(theta2);     %%%%circle O3

%new position of the UAV in original coordinate
NOU=N2OCo([NNUx NNUy NNUa],[Tx,Ty,Ta]);

%new position of the target in original coordinate
NOT=N2OCo([NNTx,NNTy,0],[Tx,Ty,Ta]); 


%plot(NOU(1),NOU(2),'*g',NOT(1),NOT(2),'*g'); %%%% new position of UAV and Target



Tt;
clear u

ptls=[NOU(1) NOU(2) NOU(3); NOT(1),NOT(2) Ta];

% figure(1)
% hold off




