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
% restart(scenario);
% waypoints=[xHistory(:,1:2)];
% trajectory(egoVehicle,waypoints);
% chasePlot(egoVehicle);
% plot(scenario,'Waypoints','on','RoadCenters','on')
% while advance(scenario)
%     pause(0.1)
% end







