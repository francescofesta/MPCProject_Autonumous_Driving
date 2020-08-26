%% CONFIGURAZIONE
mapWidth = 100;
mapLength = 100;
costVal = 0.15;
cellSize = 0.2;
vehicleDim=vehicleDimensions(egoVehicle.Length,egoVehicle.Width);
ccConfig = inflationCollisionChecker(vehicleDim,3);
ccConfig.InflationRadius=1.7;

costmap = vehicleCostmap(mapWidth,mapLength,costVal,'CellSize',cellSize,'CollisionChecker',ccConfig);
costmap.CollisionChecker=ccConfig;
occupiedVal= 0.7;



rB=roadBoundaries(scenario);
%% MATRICI
rb_mat_ext= rB{:,1};
rb_mat_int= rB{:,2};
rb_no_terz_ext=rb_mat_ext(:,1:2);
rb_no_terz_int=rb_mat_int(:,1:2);
%% COSTMAP
for i=1:1:84
setCosts(costmap,rb_no_terz_int(i,:),occupiedVal);
setCosts(costmap,rb_no_terz_ext(i,:),occupiedVal);
end

%% ostacoli
% for i=1:1:5
% setCosts(costmap,scenario.Actors(1,i).Position(1,1:2),occupiedVal);
% end
plot(costmap)
%setCosts(costmap,[17.9,16.1],occupiedVal);
%plot(costmap)
hold on

% Scelta delle pose iniziali e finali per il planning

 h=msgbox('Scelta della start pose');
    uiwait(h,5);
         if ishandle(h) == 1
         delete(h);
         end
                  
        xlabel('Usa il click sx del mouse per scegliere la posizione iniziale','Color','red');
        but=0;
%         axis([0 map_size_x 0 map_size_y])
        grid on;
        hold on;
        while (but ~= 1) %Ripete finché non premi click sx
        [xval,yval,but]=ginput(1);
        end
        dims = [1 50];
prompt = {'teta start pose','teta goal pose'};
dlgtitle = 'Inserire angoli';               
angoli = inputdlg(prompt,dlgtitle,dims); % funzione prompt comandi
teta_start=str2num(angoli{1});
teta_goal=str2num(angoli{2});
        
    startPose=[xval yval teta_start];
    
 h=msgbox('Scelta della goal pose');
    uiwait(h,5);
         if ishandle(h) == 1
         delete(h);
         end
                  
        xlabel('Usa il click sx del mouse per scegliere la posizione finale','Color','blue');
        but=0;
%         axis([0 map_size_x 0 map_size_y])
        grid on;
        hold on;
        while (but ~= 1) %Ripete finché non premi click sx
        [xval,yval,but]=ginput(1);
        end
        
    goalPose=[xval yval teta_goal]; 
    
    plot(startPose(:,1),startPose(:,2),'*');
    hold on
    plot(goalPose(:,1),goalPose(:,2),'o');
    close

