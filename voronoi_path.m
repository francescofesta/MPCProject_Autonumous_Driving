%VORONOI
[vertici,celle]=voronoin([rb_mat_ext(:,1) rb_mat_ext(:,2);rb_mat_int(:,1) rb_mat_int(:,2)]);
voronoi([rb_mat_ext(:,1); rb_mat_int(:,1)],[rb_mat_ext(:,2); rb_mat_int(:,2)]);
plot(vertici(:,1),vertici(:,2),'bo');

