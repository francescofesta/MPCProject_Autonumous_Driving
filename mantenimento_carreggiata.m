function J = mantenimento_carreggiata (X,U,e,data,params)
k=1;
j=1;
p=data.PredictionHorizon;

rb_mat_ext=params.Lane_rb_mat_ext;
rb_mat_int=params.Lane_rb_mat_int;

J=diff_int(1,1);
    
  