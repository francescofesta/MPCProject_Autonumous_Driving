function params = ObstaclePosition(scenario)
n=size(scenario.Actors,2)-1;
for i=1:1:n
obstacles.pos(i,:)=scenario.Actors(1,i).Position(1,1:2);
obstacles.length(i)=scenario.Actors(1,i).Length;
obstacles.width(i)=scenario.Actors(1,i).Width;

 

end
params=[obstacles];
end