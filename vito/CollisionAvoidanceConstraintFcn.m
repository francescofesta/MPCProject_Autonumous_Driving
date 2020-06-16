function cineq = CollisionAvoidanceConstraintFcn(X,U,e,data,params)
    p = data.PredictionHorizon;
    r_safe = 2*params.dim;  

    X1 = X(2:p+1,1:2);
    X2 = X(2:p+1,2:3);
    dist = vecnorm(X1' - X2');

    cineq = -(dist - r_safe)';
end

