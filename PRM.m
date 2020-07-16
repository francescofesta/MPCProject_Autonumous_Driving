map=binaryOccupancyMap(100,100,0.4);
posizione_ostacoli=[rb_mat_ext(:,1) rb_mat_ext(:,2);rb_mat_int(:,1) rb_mat_int(:,2)];
inflate(map,5);
setOccupancy(map,posizione_ostacoli,10);
show(map);
planner_PRM = mobileRobotPRM(map);
path = findpath(planner_PRM, startPose(1,1:2), goalPose(1,1:2));

while isempty(path)
    % No feasible path found yet, increase the number of nodes
    planner_PRM.NumNodes = planner_PRM.NumNodes + 10;
    
    % Use the |update| function to re-create the PRM roadmap with the changed
    % attribute
    update(planner_PRM);
    
    % Search for a feasible path with the updated PRM
    path = findpath(planner_PRM, startPose(1,1:2), goalPose(1,1:2));
end