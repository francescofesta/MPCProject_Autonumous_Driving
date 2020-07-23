clear all
clc
scenario_finale;
Costmap;
RRT;
pose_record=record(scenario);
%% traiettoria
sim_time={pose_record.SimulationTime};

sim_time=cell2mat(sim_time');

for i=1:1:size(pose_record,2)
    
    poses_new(i,1:2)=pose_record(i).ActorPoses(6).Position(1:2);
    velocity_new(i,1)=sqrt((pose_record(i).ActorPoses(6).Velocity(1))^2+(pose_record(i).ActorPoses(6).Velocity(2))^2);
    angle_new(i,1)=pose_record(i).ActorPoses(6).Yaw;
end

traiettoria_mat=[sim_time,poses_new,angle_new,velocity_new ]; 


%% ostacoli
ost=scenario.Actors(1,1:5);

for i=1:5
    ost_pos(i,:)=ost(1,i).Position;
    
    ost_dim(i,:)=ost(1,i).Length;
end
ost_pos(:,3)=[];


%% CONTROLLORE MPC
tic
NLmpc;
toc

%% PLOT SCENARIO PUNTI DI VIA REALI

% waypoints=[xHistory];
% waypoints_diff_prima=waypoints(:,1);
% waypoints_diff_seconda=waypoints(:,2);
% waypoints_diff_seconda=[waypoints_diff_seconda;0];
% waypoints_diff_prima=unique(waypoints_diff_prima);
% 
% waypoints_diff_seconda=unique(waypoints_diff_seconda);
% waypoints_diff=[waypoints_diff_prima waypoints_diff_seconda];
% speed=abs(uHistory(:,1));
speed=10;

% % v=vehicle(scenario);
trajectory(egoVehicle,xHistory,speed);
% 
chasePlot(egoVehicle);
plot(scenario,'Waypoints','on','RoadCenters','on')
while advance(scenario)
    pause(0.1)
end







