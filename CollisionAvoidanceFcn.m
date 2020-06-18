function cineq = CollisionAvoidanceFcn(X,U,e,data,params)
    p = data.PredictionHorizon;
    r_safe = 2*params.Obstacle.length(1);
    rb_mat_ext=params.Lane.rb_mat_ext;
    rb_mat_int=params.Lane.rb_mat_int;

    X1 = X(2:p+1,1:2); 
    X2 = X(2:p+1,2:3);
    
    dist = vecnorm(X1' - X2');
    
    Lanenorm_ext=vecnorm((rb_mat_ext(2:p+1,1))'-(rb_mat_ext(2:p+1,2))');
    Lanenorm_int=vecnorm((rb_mat_int(2:p+1,1))'-(rb_mat_int(2:p+1,2))');

    cineq = [-(dist - r_safe);
             dist-Lanenorm_ext;
             Lanenorm_int-dist];
end