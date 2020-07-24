function cineq = ObstacleAvoidanceFcn(X,U,e,data,params)
 
p=data.PredictionHorizon;
treshold_obstacle=3;
    for i=1:1:(size(params.pos,1))
        argomento_vecnorm=[X(2:p+1,1)-params.pos(i,1)  X(2:p+1,2)-params.pos(i,2)];
        out_vec(i,:)=vecnorm(argomento_vecnorm');
%       dist_obstacle(i)=out_vec(i,1)+out_vec(i,2);
        vincolo_ost(i)=out_vec(i)-treshold_obstacle;
        dist_ost=vecnorm([X(1,1)-params.pos(i,1)  X(1,2)-params.pos(i,2)]);
        if dist_ost<10
            disp('sono entrato nell if');
            if params.pos(i,1)>X(1,1) && params.pos(i,2)<X(1,2)
                vincolo_carr(:,i)=-(X(2:p+1,2)-params.pos(i,2));

            end
            if params.pos(i,1)>X(1,1) && params.pos(i,2)>X(1,2)
                vincolo_carr(:,i)=X(2:p+1,1)-params.pos(i,1);
            end
            if params.pos(i,1)<X(1,1) && params.pos(i,2)>X(1,2)
                vincolo_carr(:,i)=X(2:p+1,2)-params.pos(i,2);
            end
            if params.pos(i,1)<X(1,1) && params.pos(i,2)<X(1,2)
                vincolo_carr(:,i)=-(X(2:p+1,1)-params.pos(i,1));
            end
        
        else vincolo_carr=zeros(p,1);
        end
    
    end   
    
  vincolo_carr=reshape(vincolo_carr,[],1);
  cineq= [-vincolo_ost';vincolo_carr];

end