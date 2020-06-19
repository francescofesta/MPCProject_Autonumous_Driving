function cineq = CollisionAvoidanceFcn(X,U,e,data,params)
     p = data.PredictionHorizon;
     rb_mat_ext=params.Lane_rb_mat_ext;
     rb_mat_int=params.Lane_rb_mat_int;
     rb_mat_int_flip=flip(rb_mat_int);
     for i=1:1:size(rb_mat_int_flip,1)
         %dist_track_int(i)=norm(X(2:p+1,1:2)-rb_mat_int_flip(1,1:2))
         dist_track_ext(i)=norm(X(2:p+1,1:2)-rb_mat_ext(i,1:2));
     end
         %dist_track_int=norm(X(2:p+1,1:2)-rb_mat_int_flip(1,1:2))
     treshold_lane=0.2+(params.Vehicle_Length/2)*cos(X(2:p+1,3));
     treshold_obstacle=(params.length/2)+(params.Vehicle_Length/2)*cos(X(2:p+1,3));
     for i=1:1:size(params.pos,1)
        dist_obstacle(i)=norm(X(2:p+1,1:2)-params.pos(i,:)); 
     end
     cineq=[
         -(dist_track_int-treshold_lane)';
         -(dist_track_ext-treshold_lane)';
         -(dist_obstacle-treshold_obstacle)';
            ];
%     r_safe = 2*params.Obstacle.length(1);
%     
% 
%     X1 = X(2:p+1,1:2); 
%     X2 = X(2:p+1,2:3);
%     
%     dist = vecnorm(X1' - X2');
%     
%     Lanenorm_ext=vecnorm((rb_mat_ext(2:p+1,1))'-(rb_mat_ext(2:p+1,2))');
%     Lanenorm_int=vecnorm((rb_mat_int(2:p+1,1))'-(rb_mat_int(2:p+1,2))');
% 
%     cineq = [-(dist - r_safe);
%              dist-Lanenorm_ext;
%              Lanenorm_int-dist];
end