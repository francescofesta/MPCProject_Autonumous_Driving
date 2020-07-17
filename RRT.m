 %% RRT
% planner=pathPlannerRRT(costmap,'ConnectionDistance',15,'MinTurningRadius',2);
planner=pathPlannerRRT(costmap,'ConnectionDistance',15);
startPose=scenario.Actors(1,6).Position(1,:);
goalPose=[22.9,27.8,pi];
[refPath,tree] = plan(planner,startPose,goalPose);
plot(planner)

% poses = interpolate(refPath);
[refPoses,refDirections] = interpolate(refPath);
approxSeparation = 0.05; % meters
numSmoothPoses = round(refPath.Length / approxSeparation);
minSeparation=1;
[poses,directions] = smoothPathSpline(refPoses,refDirections,numSmoothPoses,minSeparation);
% plot(poses_filtrate(:,1),poses_filtrate(:,2),'LineWidth',2,'DisplayName','Smooth path')
% hold on
% plot(poses(:,1),poses(:,2))
% plot(traiettoria_mat(:,2),traiettoria_mat(:,3))


trajectory(egoVehicle,poses(:,1:2),30);

plot(scenario,'Waypoints','on','RoadCenters','on')
while advance(scenario)
    pause(0.1)
end