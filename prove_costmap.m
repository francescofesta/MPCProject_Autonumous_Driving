mapWidth = 25;
mapLength = 70;
costVal = 0.5;
cellSize = 0.5;

costmap = vehicleCostmap(mapWidth,mapLength,costVal,'CellSize',cellSize);

occupiedVal = 0.9;

rb_mat = rB{:,1};
rb_no_terz=rb_mat(:,1:2);
for i=1:1:53
setCosts(costmap,rb_no_terz(i,:),occupiedVal);
plot(costmap)

end

setCosts(costmap,[17.9,16.1],occupiedVal);
plot(costmap)

