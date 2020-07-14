%% CONFIGURAZIONE
mapWidth = 100;
mapLength = 100;
costVal = 0.15;
cellSize = 0.2;

costmap = vehicleCostmap(mapWidth,mapLength,costVal,'CellSize',cellSize,'InflationRadius',1.5);

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

