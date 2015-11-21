%% figure plots


figure(1) , hold on

for l = 1:1:num_of_UAVs
    X = [UAVs(l).vehical_log.x];
    Y = [UAVs(l).vehical_log.y];
    Z = [UAVs(l).vehical_log.h];
    if l == 1
        plot3(X , Y , Z)
    elseif l == 2
        plot3(X , Y , Z , 'g')
    elseif l == 3
        plot3(X , Y , Z , 'r')
    elseif l == 4
        plot3(X , Y , Z ,'c')
    elseif l ==5
        plot3(X , Y , Z ,'k')
    elseif l ==6
        plot3(X , Y , Z ,'y')
    elseif l ==7
        plot3(X , Y , Z ,'b')
    elseif l ==8
        plot3(X , Y , Z ,'m')
    else
        plot3(X , Y , Z ,'m')
    end
    
end

map = sample_arena.road_map;

road_map1 = [map.p1;map.p2;map.p6;...
    map.p7;map.p8;map.p14];


road_map2 = [map.p1;map.p5;map.p13;map.p12;...
    map.p9;map.p10;map.p11;map.p14];

road_map3 = [map.p1;map.p3;map.p7;map.p4;map.p10;map.p8];

road_map4 = [map.p5;map.p9;map.p4;map.p1];

road_map5 = [map.p2;map.p7];

road_map6 = [map.p11;map.p12];

road_map7 = [map.p6;map.p14];

road_map8 = [map.p6;map.p8];

road_map9 = [map.p11;2000 -500];

road_map10 = [map.p13;4250 -500];

plot(road_map1(:,1), road_map1(:,2),'--b')
plot(road_map2(:,1), road_map2(:,2),'--r')
plot(road_map3(:,1), road_map3(:,2),'g')
plot(road_map4(:,1), road_map4(:,2),'g')
plot(road_map5(:,1), road_map5(:,2),'g')
plot(road_map6(:,1), road_map6(:,2),'g')
plot(road_map7(:,1), road_map7(:,2),'g')
plot(road_map8(:,1), road_map8(:,2),'g')
plot(road_map9(:,1), road_map9(:,2),'g')
plot(road_map10(:,1), road_map9(:,2),'g')

rectangle('Position',[2000,-3000,2000,2000],'LineWidth',3,'LineStyle','--')
axis equal , grid on,hold off


% other results
for i = 1:1:num_of_UAVs
    
    speed(:,i) = [UAVs(i).vehical_log.v];
    heading(:,i) = [UAVs(i).vehical_log.psi];
    bank(:,i) = [UAVs(i).vehical_log.phi];
    height(:,i) = [UAVs(i).vehical_log.h];
    
end

figure(2)
subplot(4,1,1)
plot(speed)
xlabel('time (s)','fontsize',7)
ylabel('air spd (m/s)','fontsize',7)

subplot(4,1,2)
plot(heading)
xlabel('time (s)','fontsize',7)
ylabel('hdg (deg)','fontsize',7)


subplot(4,1,3)
plot(bank)
xlabel('time (s); solid - actual; dotted - cmd','fontsize',7)
ylabel('bank (deg)','fontsize',7)
subplot(4,1,4)
plot(height)
xlabel('time (s)','fontsize',7)
ylabel('ht (m)','fontsize',7)





