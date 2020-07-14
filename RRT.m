%% RRT
planner=pathPlannerRRT(costmap,'ConnectionDistance',15,'MinTurningRadius',2);
startPose=scenario.Actors(1,6).Position(1,:);
goalPose=[22.9,27.8,pi];
[refPath,tree] = plan(planner,startPose,goalPose);
plot(planner)

poses = interpolate(refPath);
trajectory(egoVehicle,poses(:,1:2),30);

plot(scenario,'Waypoints','on','RoadCenters','on')
while advance(scenario)
    pause(0.1)
end