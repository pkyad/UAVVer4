
%clear
close all

%load('3.0.4.mat');
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

road_map8 = [map.p11;2000 -500];

road_map9 = [map.p13;4250 -500];

flag = 0;
keep_going =0;
fig = figure(1);
num_of_UAVs = numel(UAVs);
set (fig, 'Units', 'normalized', 'Position', [0,0,1,1]);
pause(2)
for i = 2000:5:numel(UAVs(1).vehical_log)
    cla , hold on
    for l = 1:1:numel(UAVs)
        X = [UAVs(l).vehical_log(1:5:i).x];
        Y = [UAVs(l).vehical_log(1:5:i).y];
        Z = [UAVs(l).vehical_log(1:5:i).h];
        
        x = [UAVs(l).vehical_log(i).x];
        y = [UAVs(l).vehical_log(i).y];
        z = [UAVs(l).vehical_log(i).h];
        sumy = 0;
        sumx = 0;
        for k = 1:1:num_of_UAVs
            
            sumy = sumy + UAVs(k).vehical_log(i).y;
            sumx = sumx + UAVs(k).vehical_log(i).x;
            
        end
        
        
        if  (sumx > num_of_UAVs*1700 &&  -600*num_of_UAVs > sumy ) || keep_going ==1
            if flag == 0
               tgt_t_0 = i;
               flag = 1; 
               keep_going = 1;
            end
            for kk = 1:1:numel(sample_arena.targets)
                if sample_arena.targets(kk).group == 1
                    plot(sample_arena.targets(kk).vehical_log(i-tgt_t_0 +350 ).x , sample_arena.targets(kk).vehical_log(i-tgt_t_0 +350 ).y , 'bs')
                elseif sample_arena.targets(kk).group == 2
                    plot(sample_arena.targets(kk).vehical_log(i-tgt_t_0 +350 ).x , sample_arena.targets(kk).vehical_log(i-tgt_t_0 +350 ).y , 'rs')
                elseif sample_arena.targets(kk).group == 3
                    plot(sample_arena.targets(kk).vehical_log(i-tgt_t_0 +350 ).x , sample_arena.targets(kk).vehical_log(i-tgt_t_0 +350 ).y , 'ks')
                end
                
            end
            
        end

%         
%         if l == 1
%             plot3(X , Y , Z , '')
%         elseif l == 2
%             
%             plot3(X , Y , Z , 'g')
%             
%         elseif l == 3
%             plot3(X , Y , Z , 'r')
%         elseif l == 4
%             plot3(X , Y , Z ,'c')
%         elseif l ==5
%             plot3(X , Y , Z ,'k')
%         elseif l ==6
%             plot3(X , Y , Z ,'m')
%         end
              
        if l == 1
            plot3(x , y , z , '*')
        elseif l == 2
            
            plot3(x , y , z ,  '*g')
            
        elseif l == 3
            plot3(x , y , z ,  '*r')
        elseif l == 4
            plot3(x , y , z , '*c')
        elseif l ==5
            plot3(x , y , z , '*k')
        elseif l ==6
            plot3(x , y , z , '*m')
        elseif l ==7
            plot3(x , y , z , '*m')
        else
            plot3(x , y , z , '*m')
        end
        
    end
    
    
    for j = 1%:10:numel(sample_arena.targets(6).vehical_log)-3000
        plot(road_map1(:,1), road_map1(:,2),'--b')
        plot(road_map2(:,1), road_map2(:,2),'--r')
        plot(road_map3(:,1), road_map3(:,2),'--g')
        plot(road_map4(:,1), road_map4(:,2),'--g')
        plot(road_map5(:,1), road_map5(:,2),'--g')
        plot(road_map6(:,1), road_map6(:,2),'--g')
        plot(road_map7(:,1), road_map7(:,2),'--g')
        plot(road_map8(:,1), road_map8(:,2),'--g')
        plot(road_map9(:,1), road_map9(:,2),'--g')
        
        hold off
    end
    rectangle('Position',[2000,-3000,2000,2000],'LineWidth',3,'LineStyle','--')
    axis([100 4100 -3100 -900 0 1000]), view(2)
    drawnow;
    hold off
    
 
end


