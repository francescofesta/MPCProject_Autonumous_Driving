function [poses_filtrate,directions] =smooth_path(refPath,poses,traiettoria_mat)
[refPoses,refDirections] = interpolate(refPath);
approxSeparation = 0.1; % meters
numSmoothPoses = round(refPath.Length / approxSeparation);
[poses_filtrate,directions] = smoothPathSpline(refPoses,refDirections,numSmoothPoses);
plot(poses_filtrate(:,1),poses_filtrate(:,2),'LineWidth',2,'DisplayName','Smooth path')
hold on
plot(poses(:,1),poses(:,2))
plot(traiettoria_mat(:,2),traiettoria_mat(:,3))
end
