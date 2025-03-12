ref_pos  = data_pos_Ref_enu;
comp_pos = data_pos_Obs_enu;
ref_vel  = [state_vel_Ref_enu_x state_vel_Ref_enu_y state_vel_Ref_enu_z];
% comp_vel = data_vel_Obs_enu;
% comp_vel = data_vel_Obs_enu;
comp_vel = analize_POE;

% comp_vel = ref_vel;

dt = const_dt_Gnss;

% plot_type = 'histogram';
plot_type = 'scatter';
% plot_type = 'compass'; %おかしいので使わない

visualizeVelocityComparison(ref_pos, ref_vel, comp_pos, comp_vel, dt, plot_type);