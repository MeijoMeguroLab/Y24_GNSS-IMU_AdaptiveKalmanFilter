KF_enu = [KF_x(:,1),KF_y(:,1),KF_z(:,1)];
analize_error_EKF(:,1:3) = KF_enu(:,1:3)  - data_pos_Ref_enu(:,1:3);
analize_error_EKF3d(:,1) = sqrt(analize_error_EKF(:,1).^2+analize_error_EKF(:,2).^2+analize_error_EKF(:,3).^2);
analize_error_EKF2d(:,1) = sqrt(analize_error_EKF(:,1).^2+analize_error_EKF(:,2).^2);

area_s = [760
          947
          1330
          1640
          2300
          4140
          4500
          6330
          6796];
area_e = [800
          960
          1500
          1690
          2360
          4205
          4850
          6400
          6850];
for i = 1:9
    eval(['aria1 = ' num2str(area_s(i)) ':' num2str(area_e(i)) ';'])

% aria1 = 947:960;
% aria1
figure
grid on
hold on
axis equal
plot(data_pos_Ref_enu(aria1,1),data_pos_Ref_enu(aria1,2),'.r','MarkerSize',12)
plot(KF_x(aria1,1),KF_y(aria1,1),'.','Color','#66cdaa','MarkerSize', 12)
xlabel('East [m]')
ylabel('North [m]')
% title('trajectory Fix&Float')
% title('trajectory Float')
title(['trajectory ' title_str ' err|' num2str(mean(analize_error_EKF2d(aria1))) 'm'])
legend({'reference' 'KF後'},'FontName','ＭＳ 明朝','fontsize',9)


% figure
% grid on
% hold on
% plot(time_Obs(aria1,1),state_pos_Ref_enu(aria1,3),'.r','MarkerSize',12)
% plot(time_Obs(aria1,1),KF_z(aria1,1),'.b','MarkerSize',12)
% xlabel('time [s]')
% ylabel('Up [m]')
% title('Up down')

% figure
% grid on
% hold on
% plot(time_Obs(aria1,1),analize_error_EKF2d(aria1,1),'.b','MarkerSize',12)
% score = mean(EvalFunc_TGR(analize_error_EKF2d(aria1,1)));
% xlabel('time [s]')
% ylabel('error [m]')
% title(strcat('error ', title_str, 'score:', num2str(score)))
% xlim([time_Obs(aria1(1),1) time_Obs(aria1(end),1)])
% ylim([0 10])



disp(['area error|' num2str(mean(analize_error_EKF2d(aria1)))])

end


%% all
figure
grid on
hold on
axis equal
plot(data_pos_Ref_enu(:,1),data_pos_Ref_enu(:,2),'.r','MarkerSize',12)
plot(KF_x(:,1),KF_y(:,1),'.','Color','#66cdaa','MarkerSize', 12)
plot(state_pos_Eagleye_enu(:,1),state_pos_Eagleye_enu(:,2),'.b','MarkerSize', 10)
xlabel('East [m]')
ylabel('North [m]')
% title('trajectory Fix&Float')
% title('trajectory Float')
title(['trajectory ' title_str ' err|' num2str(mean(analize_error_EKF2d(:))) 'm'])
legend({'reference' 'KF後'},'FontName','ＭＳ 明朝','fontsize',9)
