% 軌跡

KF_xyz(:,1) = KF_x(:,1); KF_xyz(:,2) = KF_y(:,1); KF_xyz(:,3) = KF_z(:,1);
for i = 1:length(KF_xyz)  
    KF_enu(i,:) = xyz2enu( KF_xyz(i,:) ,  ORG_XYZ );
    % obs_vel_enu(i,:) = xyz2enu_vel( velocity_GNSS(i,:) ,  ORG_XYZ );
end

%%

    name_thisdata = '20241217';
    
    % if k == 2
    %     mkdir('../Figure/20241217/all')
    %     mkdir('../Figure/20241217/all', 'traj')
    % end
    pathname_at  = '../Figure/20241217/all/traj';
    figure
    grid on
    hold on
    axis equal
    plot(data_pos_Ref_enu(:,1),data_pos_Ref_enu(:,2),'.r')
    plot(KF_x(:,1),KF_y(:,1),'.b')
    for i = 1:length(KF_x)-1
        quiver(KF_x(i), KF_y(i), state_vel_Ref_enu_x(i), state_vel_Ref_enu_y(i), 0)
    end
    % for i = 1:length(KF_x)-1
    %     quiver(KF_x(i), KF_y(i), KF_vec_x(i), KF_vec_y(i), 0)
    % end
    %
    % for i = 1:length(KF_x)-1
    %     quiver(KF_x(i), KF_y(i), element_Vx3(i), element_Vy3(i),...
    %         0,'Color','b')
    % end
    % for i = 1:length(KF_x)-1
    %     quiver(KF_x(i), KF_y(i), data_velocity(i,1), data_velocity(i,2),...
    %         0,'Color','r')
    % end
    xlabel('East [m]')
    ylabel('North [m]')
    title('trajectory')
    % title(['trajectory_' num2str(k)])
    legend({'reference' 'KF後'},'FontName','ＭＳ 明朝','fontsize',9)
    ax = gca;
    ax.FontName = 'Times New Roman';
    ax.FontSize = 16;
    ax.Box = 'on';
    axSTR = ax.Title.String;
    axSTR = replace(axSTR, ' ', '_');
    axSTR = replace(axSTR, '　', '_');
    axSTR = replace(axSTR, ':', '_');
    axSTR = replace(axSTR, '\n', '_');
    path = strcat(pathname_at, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    pathfig = strcat(pathname_at, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    saveas(gcf, path)
    saveas(gcf, pathfig)



    figure
    grid on
    hold on
    plot(1:time_SimEpoch,data_velocity(:,1),'.b')
    plot(1:time_SimEpoch,element_Vx3(:,1),'.r')
    % area0 = 947:960;
    % area1 = 940:970;
    % area2 = 1580:1760;
    % area3 = 1850:2000;
    % area4 = 2300:2550;
    % 
    % disp(num2str(k))
    % 
    % 
    % 
    % % area0
    % area = 'area0';
    % if k == 2
    %     mkdir('../Figure/20241217', area)
    %     mkdir(strcat('../Figure/20241217/', area), 'traj')
    %     mkdir(strcat('../Figure/20241217/', area), 'variance East')
    %     mkdir(strcat('../Figure/20241217/', area), 'variance North')
    %     mkdir(strcat('../Figure/20241217/', area), 'error')
    % end
    % pathname_t  = strcat('../Figure/20241217/', area, '/traj');
    % pathname_ve = strcat('../Figure/20241217/', area, '/variance East');
    % pathname_vn = strcat('../Figure/20241217/', area, '/variance North');
    % pathname_e = strcat('../Figure/20241217/', area, '/error');
    % 
    % figure
    % grid on
    % hold on
    % axis equal
    % plot(data_pos_Ref_enu(area0,1),data_pos_Ref_enu(area0,2),'.r','MarkerSize',12)
    % plot(KF_x(area0,1),KF_y(area0,1),'.','Color','#66cdaa','MarkerSize', 12)
    % % for i = 1:length(area0)-1
    % %     quiver(KF_x(area0(i)), KF_y(area0(i)), KF_vec_x(area0(i)), KF_vec_y(area0(i)), 0)
    % % end
    % xlabel('East [m]')
    % ylabel('North [m]')
    % % title('trajectory Fix&Float')
    % % title('trajectory Float')
    % % title('trajectory Float adaptive')
    % title(['trajectory Float adaptive win:' num2str(k)])
    % legend({'reference' 'KF後'},'FontName','ＭＳ 明朝','fontsize',9)
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_t, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_t, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % 
    % figure
    % grid on
    % hold on
    % % axis equal
    % plot(time_Obs(area0,1),status_dispersion_param_obs_noise_gnssX(area0,1),'.b','MarkerSize',12)
    % xlabel('time [s]')
    % ylabel('variance [m^2]')
    % title(['variance East' num2str(k)])
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_ve, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_ve, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % figure
    % grid on
    % hold on
    % % axis equal
    % plot(time_Obs(area0,1),status_dispersion_param_obs_noise_gnssY(area0,1),'.b','MarkerSize',12)
    % xlabel('time [s]')
    % ylabel('variance [m^2]')
    % title(['variance North' num2str(k)])
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_vn, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_vn, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % 
    % figure
    % grid on
    % hold on
    % % axis equal
    % plot(time_Obs(area0,1),analize_error_EKF3d(area0,1),'xr','MarkerSize',12)
    % xlabel('time [s]')
    % ylabel('error [m]')
    % title(['error 3d' num2str(k) '  mean:' num2str(mean(analize_error_EKF3d(area0,1)))])
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_e, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_e, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % 
    % % area1
    % area = 'area1';
    % if k == 2
    %     mkdir('../Figure/20241217', area)
    %     mkdir(strcat('../Figure/20241217/', area), 'traj')
    %     mkdir(strcat('../Figure/20241217/', area), 'variance East')
    %     mkdir(strcat('../Figure/20241217/', area), 'variance North')
    %     mkdir(strcat('../Figure/20241217/', area), 'error')
    % end
    % pathname_t  = strcat('../Figure/20241217/', area, '/traj');
    % pathname_ve = strcat('../Figure/20241217/', area, '/variance East');
    % pathname_vn = strcat('../Figure/20241217/', area, '/variance North');
    % pathname_e = strcat('../Figure/20241217/', area, '/error');
    % 
    % 
    % figure
    % grid on
    % hold on
    % axis equal
    % plot(data_pos_Ref_enu(area1,1),data_pos_Ref_enu(area1,2),'.r','MarkerSize',12)
    % plot(KF_x(area1,1),KF_y(area1,1),'.','Color','#66cdaa','MarkerSize', 12)
    % % for i = 1:length(area1)-1
    % %     quiver(KF_x(area1(i)), KF_y(area1(i)), KF_vec_x(area1(i)), KF_vec_y(area1(i)), 0)
    % % end
    % xlabel('East [m]')
    % ylabel('North [m]')
    % % title('trajectory Fix&Float')
    % % title('trajectory Float')
    % % title('trajectory Float adaptive')
    % title(['trajectory Float adaptive win:' num2str(k)])
    % legend({'reference' 'KF後'},'FontName','ＭＳ 明朝','fontsize',9)
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_t, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_t, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % figure
    % grid on
    % hold on
    % % axis equal
    % plot(time_Obs(area1,1),status_dispersion_param_obs_noise_gnssX(area1,1),'.b','MarkerSize',12)
    % xlabel('time [s]')
    % ylabel('variance [m^2]')
    % title(['variance East' num2str(k)])
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_ve, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_ve, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % figure
    % grid on
    % hold on
    % % axis equal
    % plot(time_Obs(area1,1),status_dispersion_param_obs_noise_gnssY(area1,1),'.b','MarkerSize',12)
    % xlabel('time [s]')
    % ylabel('variance [m^2]')
    % title(['variance North' num2str(k)])
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_vn, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_vn, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % 
    % figure
    % grid on
    % hold on
    % % axis equal
    % plot(time_Obs(area1,1),analize_error_EKF3d(area1,1),'xr','MarkerSize',12)
    % xlabel('time [s]')
    % ylabel('error [m]')
    % title(['error 3d' num2str(k) '  mean:' num2str(mean(analize_error_EKF3d(area1,1)))])
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_e, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_e, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % 
    % % area2
    % area = 'area2';
    % if k == 2
    %     mkdir('../Figure/20241217', area)
    %     mkdir(strcat('../Figure/20241217/', area), 'traj')
    %     mkdir(strcat('../Figure/20241217/', area), 'variance East')
    %     mkdir(strcat('../Figure/20241217/', area), 'variance North')
    %     mkdir(strcat('../Figure/20241217/', area), 'error')
    % end
    % pathname_t  = strcat('../Figure/20241217/', area, '/traj');
    % pathname_ve = strcat('../Figure/20241217/', area, '/variance East');
    % pathname_vn = strcat('../Figure/20241217/', area, '/variance North');
    % pathname_e = strcat('../Figure/20241217/', area, '/error');
    % 
    % 
    % figure
    % grid on
    % hold on
    % axis equal
    % plot(data_pos_Ref_enu(area2,1),data_pos_Ref_enu(area2,2),'.r','MarkerSize',12)
    % plot(KF_x(area2,1),KF_y(area2,1),'.','Color','#66cdaa','MarkerSize', 12)
    % % for i = 1:length(area2)-1
    % %     quiver(KF_x(area2(i)), KF_y(area2(i)), KF_vec_x(area2(i)), KF_vec_y(area2(i)), 0)
    % % end
    % xlabel('East [m]')
    % ylabel('North [m]')
    % % title('trajectory Fix&Float')
    % % title('trajectory Float')
    % % title('trajectory Float adaptive')
    % title(['trajectory Float adaptive win:' num2str(k)])
    % legend({'reference' 'KF後'},'FontName','ＭＳ 明朝','fontsize',9)
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_t, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_t, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % figure
    % grid on
    % hold on
    % % axis equal
    % plot(time_Obs(area2,1),status_dispersion_param_obs_noise_gnssX(area2,1),'.b','MarkerSize',12)
    % xlabel('time [s]')
    % ylabel('variance [m^2]')
    % title(['variance East' num2str(k)])
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_ve, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_ve, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % figure
    % grid on
    % hold on
    % % axis equal
    % plot(time_Obs(area2,1),status_dispersion_param_obs_noise_gnssY(area2,1),'.b','MarkerSize',12)
    % xlabel('time [s]')
    % ylabel('variance [m^2]')
    % title(['variance North' num2str(k)])
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_vn, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_vn, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % 
    % figure
    % grid on
    % hold on
    % % axis equal
    % plot(time_Obs(area2,1),analize_error_EKF3d(area2,1),'xr','MarkerSize',12)
    % xlabel('time [s]')
    % ylabel('error [m]')
    % title(['error 3d' num2str(k) '  mean:' num2str(mean(analize_error_EKF3d(area2,1)))])
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_e, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_e, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % 
    % 
    % % area3
    % area = 'area3';
    % if k == 2
    %     mkdir('../Figure/20241217', area)
    %     mkdir(strcat('../Figure/20241217/', area), 'traj')
    %     mkdir(strcat('../Figure/20241217/', area), 'variance East')
    %     mkdir(strcat('../Figure/20241217/', area), 'variance North')
    %     mkdir(strcat('../Figure/20241217/', area), 'error')
    % end
    % pathname_t  = strcat('../Figure/20241217/', area, '/traj');
    % pathname_ve = strcat('../Figure/20241217/', area, '/variance East');
    % pathname_vn = strcat('../Figure/20241217/', area, '/variance North');
    % pathname_e = strcat('../Figure/20241217/', area, '/error');
    % 
    % 
    % figure
    % grid on
    % hold on
    % axis equal
    % plot(data_pos_Ref_enu(area3,1),data_pos_Ref_enu(area3,2),'.r','MarkerSize',12)
    % plot(KF_x(area3,1),KF_y(area3,1),'.','Color','#66cdaa','MarkerSize', 12)
    % % for i = 1:length(area3)-1
    % %     quiver(KF_x(area3(i)), KF_y(area3(i)), KF_vec_x(area3(i)), KF_vec_y(area3(i)), 0)
    % % end
    % xlabel('East [m]')
    % ylabel('North [m]')
    % % title('trajectory Fix&Float')
    % % title('trajectory Float')
    % % title('trajectory Float adaptive')
    % title(['trajectory Float adaptive win:' num2str(k)])
    % legend({'reference' 'KF後'},'FontName','ＭＳ 明朝','fontsize',9)
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_t, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_t, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % figure
    % grid on
    % hold on
    % % axis equal
    % plot(time_Obs(area3,1),status_dispersion_param_obs_noise_gnssX(area3,1),'.b','MarkerSize',12)
    % xlabel('time [s]')
    % ylabel('variance [m^2]')
    % title(['variance East' num2str(k)])
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_ve, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_ve, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % figure
    % grid on
    % hold on
    % % axis equal
    % plot(time_Obs(area3,1),status_dispersion_param_obs_noise_gnssY(area3,1),'.b','MarkerSize',12)
    % xlabel('time [s]')
    % ylabel('variance [m^2]')
    % title(['variance North' num2str(k)])
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_vn, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_vn, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % 
    % figure
    % grid on
    % hold on
    % % axis equal
    % plot(time_Obs(area3,1),analize_error_EKF3d(area3,1),'xr','MarkerSize',12)
    % xlabel('time [s]')
    % ylabel('error [m]')
    % title(['error 3d' num2str(k) '  mean:' num2str(mean(analize_error_EKF3d(area3,1)))])
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_e, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_e, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % 
    % 
    % % area4
    % area = 'area4';
    % if k == 2
    %     mkdir('../Figure/20241217', area)
    %     mkdir(strcat('../Figure/20241217/', area), 'traj')
    %     mkdir(strcat('../Figure/20241217/', area), 'variance East')
    %     mkdir(strcat('../Figure/20241217/', area), 'variance North')
    %     mkdir(strcat('../Figure/20241217/', area), 'error')
    % end
    % pathname_t  = strcat('../Figure/20241217/', area, '/traj');
    % pathname_ve = strcat('../Figure/20241217/', area, '/variance East');
    % pathname_vn = strcat('../Figure/20241217/', area, '/variance North');
    % pathname_e = strcat('../Figure/20241217/', area, '/error');
    % 
    % 
    % figure
    % grid on
    % hold on
    % axis equal
    % plot(data_pos_Ref_enu(area4,1),data_pos_Ref_enu(area4,2),'.r','MarkerSize',12)
    % plot(KF_x(area4,1),KF_y(area4,1),'.','Color','#66cdaa','MarkerSize', 12)
    % % for i = 1:length(area4)-1
    % %     quiver(KF_x(area4(i)), KF_y(area4(i)), KF_vec_x(area4(i)), KF_vec_y(area4(i)), 0)
    % % end
    % xlabel('East [m]')
    % ylabel('North [m]')
    % % title('trajectory Fix&Float')
    % % title('trajectory Float')
    % % title('trajectory Float adaptive')
    % title(['trajectory Float adaptive win:' num2str(k)])
    % legend({'reference' 'KF後'},'FontName','ＭＳ 明朝','fontsize',9)
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_t, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_t, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % figure
    % grid on
    % hold on
    % % axis equal
    % plot(time_Obs(area4,1),status_dispersion_param_obs_noise_gnssX(area4,1),'.b','MarkerSize',12)
    % xlabel('time [s]')
    % ylabel('variance [m^2]')
    % title(['variance East' num2str(k)])
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_ve, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_ve, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % figure
    % grid on
    % hold on
    % % axis equal
    % plot(time_Obs(area4,1),status_dispersion_param_obs_noise_gnssY(area4,1),'.b','MarkerSize',12)
    % xlabel('time [s]')
    % ylabel('variance [m^2]')
    % title(['variance North' num2str(k)])
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_vn, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_vn, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)
    % 
    % 
    % figure
    % grid on
    % hold on
    % % axis equal
    % plot(time_Obs(area4,1),analize_error_EKF3d(area4,1),'xr','MarkerSize',12)
    % xlabel('time [s]')
    % ylabel('error [m]')
    % title(['error 3d' num2str(k) '  mean:' num2str(mean(analize_error_EKF3d(area4,1)))])
    % ax = gca;
    % ax.FontName = 'Times New Roman';
    % ax.FontSize = 16;
    % ax.Box = 'on';
    % axSTR = ax.Title.String;
    % axSTR = replace(axSTR, ' ', '_');
    % axSTR = replace(axSTR, '　', '_');
    % axSTR = replace(axSTR, ':', '_');
    % axSTR = replace(axSTR, '\n', '_');
    % path = strcat(pathname_e, '\', strcat(axSTR,  '_', name_thisdata), '.jpg');
    % pathfig = strcat(pathname_e, '\', strcat(axSTR,  '_', name_thisdata), '.fig');
    % saveas(gcf, path)
    % saveas(gcf, pathfig)

    % disp(num2str(k))
    % mean(KF_x(area1,1) - state_pos_Ref_enu(area1,1))
    % mean(KF_y(area1,1) - state_pos_Ref_enu(area1,2))
% end






%%
figure
grid on
hold on
axis equal
plot(ref_enu(:,1),ref_enu(:,2),'.r')
plot(gnss_enu(:,1),gnss_enu(:,2),'.g')
plot(KF_enu(:,1),KF_enu(:,2),'.b')
xlabel('East')
ylabel('North')
legend('ref','gnss','KF')

figure
grid on
hold on
axis equal
plot(ref_enu(:,1),ref_enu(:,2),'.r')
plot(gnss_enu(:,1),gnss_enu(:,2),'.g')
plot(KF_xyz(:,1),KF_xyz(:,2),'.b')
xlabel('East')
ylabel('North')
legend('ref','gnss','KF')


figure
grid on
hold on
axis equal
plot(trajectory(:,6),trajectory(:,7),'.r')
plot(rawdata_pos(:,3),rawdata_pos(:,4),'.g')
% plot(KF_xyz(:,1),KF_xyz(:,2),'.b')
xlabel('East')
ylabel('North')
legend('ref','gnss','KF')

figure
grid on
hold on
axis equal
plot(POSLV_Sync(:,6),POSLV_Sync(:,7),'.r')
plot(rawdata_pos(:,3),rawdata_pos(:,4),'.g')
% plot(KF_xyz(:,1),KF_xyz(:,2),'.b')
xlabel('East')
ylabel('North')
legend('ref','gnss','KF')


figure
grid on
hold on
axis equal
plot(ref_xyz(:,1),ref_xyz(:,2),'.r')
plot(gnss_xyz(:,1),gnss_xyz(:,2),'.g')
% plot(KF_xyz(:,1),KF_xyz(:,2),'.b')
xlabel('East')
ylabel('North')
legend('ref','gnss','KF')

figure
grid on
hold on
axis equal
plot(ref_xyz(:,1),ref_xyz(:,2),'.r')
plot(gnss(:,3),gnss(:,4),'.g')
% plot(KF_xyz(:,1),KF_xyz(:,2),'.b')
xlabel('East')
ylabel('North')
legend('ref','gnss','KF')



% figure
% grid on
% hold on
% axis equal
% plot(ref_xyz(:,1),ref_xyz(:,2),'.r')
% plot(gnss_xyz(:,1),gnss_xyz(:,2),'.g')
% % plot(EAGLEYE(:,81),EAGLEYE(:,82),'.k')
% % plot(KF_xyz(:,1),KF_xyz(:,2),'.b')
% xlabel('East')
% ylabel('North')
% % legend('ref','gnss','KF')

% figure
% grid on
% hold on
% % axis equal
% plot(1:length(trajectory),trajectory(:,24),'.r')
% plot(1:length(eagleye_log),eagleye_log(:,2),'.g')
% % plot(KF_xyz(:,1),KF_xyz(:,2),'.b')
% xlabel('epoc')
% ylabel('roll')
% 
% figure
% grid on
% hold on
% % axis equal
% plot(1:length(trajectory),trajectory(:,25),'.r')
% plot(1:length(eagleye_log),eagleye_log(:,3),'.g')
% % plot(KF_xyz(:,1),KF_xyz(:,2),'.b')
% xlabel('epoc')
% ylabel('pitch')
% 
figure
grid on
hold on
% axis equal
plot(1:length(trajectory),trajectory(:,26),'.r')
plot(1:length(eagleye_log),eagleye_log(:,4),'.g')
% plot(KF_xyz(:,1),KF_xyz(:,2),'.b')
xlabel('epoc')
ylabel('yaw')

figure
grid on
hold on
% axis equal
plot(1:length(ref),ref(:,26),'.r')
plot(1:length(EAGLEYE),EAGLEYE(:,4),'.g')
% plot(KF_xyz(:,1),KF_xyz(:,2),'.b')
xlabel('epoc')
ylabel('yaw')

figure
grid on
hold on
% axis equal
plot(1:length(eagleye_log),eagleye_log(:,4),'.g')
plot(1:length(POSLV_Sync),POSLV_Sync(:,26),'.r')
% plot(KF_xyz(:,1),KF_xyz(:,2),'.b')
xlabel('epoc')
ylabel('yaw')


%%
cd ..