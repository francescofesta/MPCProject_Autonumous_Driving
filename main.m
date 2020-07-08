clear all;
clc;
scenario_finale;
Costmap;
RRT;
pose_record=record(scenario);

sim_time={pose_record.SimulationTime};
sim_time=cell2mat(sim_time');
for i=1:1:size(pose_record,2)
    
    poses_new(i,1:2)=pose_record(i).ActorPoses(6).Position(1:2);
    velocity_new(i,1)=sqrt((pose_record(i).ActorPoses(6).Velocity(1))^2+(pose_record(i).ActorPoses(6).Velocity(2))^2);
    angle_new(i,1)=pose_record(i).ActorPoses(6).Yaw;
end

traiettoria_mat=[sim_time,poses_new,angle_new,velocity_new ]; 

%% GENERAZIONE PLANT
AdaptiveModelGen;
%% CONTROLORE MPC
mpc_designer;




