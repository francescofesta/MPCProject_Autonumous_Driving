function cineq = CollisionAvoidanceFcn(X,U,e,data,params)
     p = data.PredictionHorizon;
     rb_mat_ext=params.Lane_rb_mat_ext;
     rb_mat_int=params.Lane_rb_mat_int;
     rb_mat_int_flip=flip(rb_mat_int);
     treshold_lane=0.2+(params.Vehicle_Length/2)*cos(X(2:p+1,3));
     for i=2:1:p
         for j=1:1:84
             diff_int_x(i,j)=X(i,1)-rb_mat_int_flip(j,1);
             diff_int_y(i,j)=X(i,2)-rb_mat_int_flip(j,2);

             
             diff_ext_x(i,j)=X(i,1)-rb_mat_ext(j,1);
             diff_ext_y(i,j)=X(i,2)-rb_mat_ext(j,2);
         end
     end

     diff_int_x=reshape(diff_int_x,[],1);
     diff_int_x=diff_int_x';
     diff_int_y=reshape(diff_int_y,[],1);
     diff_int_y=diff_int_y';
     diff_int=[diff_int_x;diff_int_y];
     dist_track_int=vecnorm(diff_int);
     diff_ext_x=reshape(diff_ext_x,[],1);
     diff_ext_x=diff_ext_x';
     diff_ext_y=reshape(diff_ext_y,[],1);
     diff_ext_y=diff_ext_y';
     diff_ext=[diff_ext_x;diff_ext_y];
     dist_track_ext=vecnorm(diff_ext);
     
      for i=1:1:size(dist_track_int,2)
         vincolo_int(:,i)=dist_track_int(:,i)'-treshold_lane;
         vincolo_est(:,i)=dist_track_ext(:,i)'-treshold_lane;
      end
         
    
     
     
     
     %FEDERICO
%      for k=1:1:p
%      diff_int(k,:)=[diff_int_x(k,:);diff_int_y(k,:)];
%      diff_int_norm()=vecnorm(diff_int(k,:));
%      end
%      diff_int_colonne=reshape(diff_int,[],1);
%      diff_ext_colonne=reshape(diff_ext,[],1);
     
     
%       for i=1:1:size(rb_mat_int_flip,1)
%          diff_int=X(2:p+1,1:2)-rb_mat_int_flip(i,1:2);
%          dist_track_int(:,i)=vecnorm(diff_int',2,1);
%          dist_track_ext(:,i)=vecnorm((X(2:p+1,1:2)-rb_mat_ext(i,1:2))',2,1);
%          vincolo_int(:,i)=dist_track_int(:,i)-treshold_lane;
%          vincolo_est(:,i)=dist_track_ext(:,i)-treshold_lane;
%        
%       end
      
      vincolo_int=reshape(vincolo_int,[],1);
      vincolo_est=reshape(vincolo_est,[],1);
      vincolo_int=vincolo_int(1:6:8400);
      vincolo_est=vincolo_est(1:6:8400);
%       vincolo_int_new(1)=vincolo_int(1);
%       vincolo_est_new(1)=vincolo_est(1);
%       
%       for i=3:3:size(vincolo_int,1)
%           vincolo_int_new()=vincolo_int(i);
%           vincolo_est_new(i-2)=vincolo_est(i);
%       end
      
         %dist_track_int=norm(X(2:p+1,1:2)-rb_mat_int_flip(1,1:2))
     treshold_obstacle=(params.length(1,1)/2)+(params.Vehicle_Length/2)*cos(X(2:p+1,3));
      for i=1:1:size(params.pos,1)
        dist_obstacle(:,i)=vecnorm(X(2:p+1,1:2)-params.pos(i,:),2,2); 
        vincolo_ost(:,i)=dist_obstacle(:,i)-treshold_obstacle;
      end
      
      vincolo_ost=reshape(vincolo_ost,[],1);
     cineq=[
         -(vincolo_int);
         -(vincolo_est);
         -(vincolo_ost);
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