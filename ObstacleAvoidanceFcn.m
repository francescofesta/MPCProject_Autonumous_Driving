function cineq = ObstacleAvoidanceFcn(X,U,e,data,params)
 
p=data.PredictionHorizon;
xref=data.References(:,1);
yref=data.References(:,2);

treshold_obstacle=3;
    for i=1:1:(size(params.pos,1))
        argomento_vecnorm=[X(2:p+1,1)-params.pos(i,1)  X(2:p+1,2)-params.pos(i,2)];
        out_vec(i,:)=vecnorm(argomento_vecnorm');
%       dist_obstacle(i)=out_vec(i,1)+out_vec(i,2);
        vincolo_ost(i)=out_vec(i)-treshold_obstacle;
    end   
    
    tresholdRRT=2;
    
        arg=[X(2:p+1,1)-xref(:,1) X(2:p+1,2)-yref(:,1)];
        distRRT=vecnorm(arg');
        vincolo_RRT=distRRT-tresholdRRT;
   
%  vincolo_ost=reshape(vincolo_ost,[],1);
%      for i=1:1:5
        cineq= [-vincolo_ost'; -vincolo_RRT'];

end