clear status_PositionResidual
status_PositionResidual(:,1:3) = state_pos_Eagleye_enu(:,1:3) - data_pos_Ref_enu(:,1:3);

x = 10;
figure
grid on
hold on
subplot(2,x,1)
plot(state_pos_Eagleye_enu(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('EGLI enu')
subplot(2,x,2)
plot(state_vel_Obs_enu_x,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('GNSS enu')
subplot(2,x,3)
plot(state_vel_Ref_enu_x,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('PSLV enu')
subplot(2,x,4)
plot(data_velocity(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('velocity')
subplot(2,x,5)
plot(element_Vx,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('Velx')
subplot(2,x,6)
plot(element_Vx3,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('Velx3')
subplot(2,x,7)
plot(status_timedifference(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('timediff')
subplot(2,x,8)
plot(state_gnss_yaw,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('gnssyaw')
subplot(2,x,9)
plot(element_Y_yaw,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('yaw')
subplot(2,x,10)
plot(element_yr,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('yawrate')
subplot(2,x,11)
plot(element_R_roll,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('roll')
subplot(2,x,12)
plot(element_rr,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('rolrate')
subplot(2,x,13)
plot(element_P_pitch,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('pitch')
subplot(2,x,14)
plot(element_pr,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('pitchr')
subplot(2,x,15)
plot(element_Ax,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('Accx')
subplot(2,x,16)
plot(element_ax_EAGLEYE,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('EGLIacc')
subplot(2,x,17)
plot(data_Heading_Eagleye,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('HeadEGL')
subplot(2,x,18)
plot(data_difference,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('differ')
subplot(2,x,19)
plot(const_dt3,1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('dt3')
subplot(2,x,20)
plot(status_PositionResidual(:,1),1:time_SimEpoch,'.r')
ylim([1 time_SimEpoch])
title('PosRed')



figure
grid on
hold on
subplot(2,x,1)
plot(state_pos_Eagleye_enu(:,1)-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('EGLI enu')
subplot(2,x,2)
plot(state_vel_Obs_enu_x-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('GNSS enu')
subplot(2,x,3)
plot(state_vel_Ref_enu_x-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('PSLV enu')
subplot(2,x,4)
plot(data_velocity(:,1)-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('velocity')
subplot(2,x,5)
plot(element_Vx-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('Velx')
subplot(2,x,6)
plot(element_Vx3-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('Velx3')
subplot(2,x,7)
plot(status_timedifference(:,1)-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('timediff')
subplot(2,x,8)
plot(state_gnss_yaw-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('gnssyaw')
subplot(2,x,9)
plot(element_Y_yaw-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('yaw')
subplot(2,x,10)
plot(element_yr-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('yawrate')
subplot(2,x,11)
plot(element_R_roll-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('roll')
subplot(2,x,12)
plot(element_rr-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('rolrate')
subplot(2,x,13)
plot(element_P_pitch-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('pitch')
subplot(2,x,14)
plot(element_pr-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('pitchr')
subplot(2,x,15)
plot(element_Ax-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('Accx')
subplot(2,x,16)
plot(element_ax_EAGLEYE-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('EGLIacc')
subplot(2,x,17)
plot(data_Heading_Eagleye-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('HeadEGL')
subplot(2,x,18)
plot(data_difference-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('differ')
subplot(2,x,19)
plot(const_dt3-status_PositionResidual(:,1),1:time_SimEpoch,'.b')
ylim([1 time_SimEpoch])
title('dt3')

figure
grid on
hold on
plot(1:time_SimEpoch,state_vel_Obs_enu_x-status_PositionResidual(:,1),'o')
plot(1:time_SimEpoch,data_velocity(:,1)-status_PositionResidual(:,1),'o')
plot(1:time_SimEpoch,element_Vx3-status_PositionResidual(:,1),'o')
plot(1:time_SimEpoch,status_timedifference(:,1)-status_PositionResidual(:,1),'o')
plot(1:time_SimEpoch,element_R_roll-status_PositionResidual(:,1),'.')
plot(1:time_SimEpoch,element_ax_EAGLEYE-status_PositionResidual(:,1),'.')
plot(1:time_SimEpoch,data_Heading_Eagleye-status_PositionResidual(:,1),'.')
plot(1:time_SimEpoch,data_difference-status_PositionResidual(:,1),'.')
xlim([1 time_SimEpoch])
legend({'velobs' 'velocity' 'Vx3' 'timediff' 'roll' 'ax' 'head' 'difference'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)

figure
grid on
hold on
plot(1:time_SimEpoch,data_velocity(:,1)-status_PositionResidual(:,1),'o')
plot(1:time_SimEpoch,element_Vx3-status_PositionResidual(:,1),'o')
plot(1:time_SimEpoch,status_timedifference(:,1)-status_PositionResidual(:,1),'o')
plot(1:time_SimEpoch,element_R_roll-status_PositionResidual(:,1),'.')
plot(1:time_SimEpoch,element_ax_EAGLEYE-status_PositionResidual(:,1),'.')
plot(1:time_SimEpoch,data_Heading_Eagleye-status_PositionResidual(:,1),'.')
plot(1:time_SimEpoch,data_difference-status_PositionResidual(:,1),'.')
xlim([1 time_SimEpoch])
legend({'velocity' 'Vx3' 'timediff' 'roll' 'ax' 'head' 'difference'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)



figure
grid on
hold on
plot(1:time_SimEpoch,data_Heading_Eagleye-status_PositionResidual(:,1),'.')
plot(1:time_SimEpoch,data_difference-status_PositionResidual(:,1),'.')
xlim([1 time_SimEpoch])
legend({'head' 'difference'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)



%

figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,state_vel_Obs_enu_x,'.')
plot(1:time_SimEpoch,data_velocity(:,1),'.')
plot(1:time_SimEpoch,element_Vx3,'.')
plot(1:time_SimEpoch,status_timedifference(:,1),'.')
plot(1:time_SimEpoch,element_R_roll,'.')
plot(1:time_SimEpoch,element_ax_EAGLEYE,'.')
plot(1:time_SimEpoch,data_Heading_Eagleye,'.')
plot(1:time_SimEpoch,data_difference,'.')
xlim([1 time_SimEpoch])
legend({'PR' 'velobs' 'velocity' 'Vx3' 'timediff' 'roll' 'ax' 'head' 'difference'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)


figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
xlim([1 time_SimEpoch])
ylim([-10 5])

figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,state_vel_Obs_enu_x,'.b')
xlim([1 time_SimEpoch])
ylim([-10 5])
legend({'PR' 'velobs'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,data_velocity(:,1),'.b')
xlim([1 time_SimEpoch])
ylim([-10 5])
legend({'PR' 'velocity'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,element_Vx3,'.b')
xlim([1 time_SimEpoch])
ylim([-10 5])
legend({'PR' 'Vx3'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,status_timedifference(:,1),'.b')
xlim([1 time_SimEpoch])
ylim([-10 5])
legend({'PR'  'timediff'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,element_R_roll,'.b')
xlim([1 time_SimEpoch])
ylim([-10 5])
legend({'PR' 'roll'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,element_ax_EAGLEYE,'.b')
xlim([1 time_SimEpoch])
ylim([-10 5])
legend({'PR' 'ax'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,data_Heading_Eagleye,'.b')
xlim([1 time_SimEpoch])
ylim([-10 5])
legend({'PR' 'head'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,data_difference,'.b')
xlim([1 time_SimEpoch])
ylim([-10 5])
legend({'PR' 'difference'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)

%
figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,status_timedifference(:,1),'.b')
plot(1:time_SimEpoch,state_vel_Obs_enu_x,'.y')
xlim([1 time_SimEpoch])
ylim([-10 5])
legend({'PR'  'vel_Obs'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,status_timedifference(:,1),'.b')
plot(1:time_SimEpoch,data_velocity(:,1),'.y')
xlim([1 time_SimEpoch])
ylim([-10 5])
legend({'PR'  'velocity'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,status_timedifference(:,1),'.b')
plot(1:time_SimEpoch,element_Vx3,'.y')
xlim([1 time_SimEpoch])
ylim([-10 5])
legend({'PR'  'Vx3'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,status_timedifference(:,1),'.b')
plot(1:time_SimEpoch,element_R_roll,'.y')
xlim([1 time_SimEpoch])
ylim([-10 5])
legend({'PR'  'roll'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,status_timedifference(:,1),'.b')
plot(1:time_SimEpoch,element_ax_EAGLEYE,'.y')
xlim([1 time_SimEpoch])
ylim([-10 5])
legend({'PR'  'ax_EAGLEYE'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,status_timedifference(:,1),'.b')
plot(1:time_SimEpoch,data_Heading_Eagleye,'.y')
xlim([1 time_SimEpoch])
ylim([-10 5])
legend({'PR'  'Heading'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,1),'or')
plot(1:time_SimEpoch,status_timedifference(:,1),'.b')
plot(1:time_SimEpoch,data_difference,'.y')
xlim([1 time_SimEpoch])
ylim([-10 5])
legend({'PR'  'difference'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)

%
%
figure
grid on
hold on
plot(1:time_SimEpoch,abs(status_PositionResidual(:,1)),'or')
plot(1:time_SimEpoch,abs(status_timedifference(:,1)),'.b')
plot(1:time_SimEpoch,abs(state_vel_Obs_enu_x),'.y')
xlim([1 time_SimEpoch])
ylim([0 5])
legend({'PR'  'vel_Obs'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,abs(status_PositionResidual(:,1)),'or')
plot(1:time_SimEpoch,abs(status_timedifference(:,1)),'.b')
plot(1:time_SimEpoch,abs(data_velocity(:,1).*const_dt_Gnss),'.y')
xlim([1 time_SimEpoch])
ylim([0 5])
legend({'PR'  'velocity'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,abs(status_PositionResidual(:,1)),'or')
plot(1:time_SimEpoch,abs(status_timedifference(:,1)),'.b')
plot(1:time_SimEpoch,abs(element_Vx3),'.y')
xlim([1 time_SimEpoch])
ylim([0 5])
legend({'PR'  'Vx3'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,abs(status_PositionResidual(:,1)),'or')
plot(1:time_SimEpoch,abs(status_timedifference(:,1)),'.b')
plot(1:time_SimEpoch,abs(element_R_roll),'.y')
xlim([1 time_SimEpoch])
ylim([0 5])
legend({'PR'  'roll'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,abs(status_PositionResidual(:,1)),'or')
plot(1:time_SimEpoch,abs(status_timedifference(:,1)),'.b')
plot(1:time_SimEpoch,abs(element_ax_EAGLEYE),'.y')
xlim([1 time_SimEpoch])
ylim([0 5])
legend({'PR'  'ax_EAGLEYE'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,abs(status_PositionResidual(:,1)),'or')
plot(1:time_SimEpoch,abs(status_timedifference(:,1)),'.b')
plot(1:time_SimEpoch,abs(data_Heading_Eagleye),'.y')
xlim([1 time_SimEpoch])
ylim([0 5])
legend({'PR'  'Heading'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
figure
grid on
hold on
plot(1:time_SimEpoch,abs(status_PositionResidual(:,1)),'or')
plot(1:time_SimEpoch,abs(status_timedifference(:,1)),'.b')
plot(1:time_SimEpoch,abs(data_difference),'.y')
xlim([1 time_SimEpoch])
ylim([0 5])
legend({'PR'  'difference'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)


%
figure
grid on
hold on
plot(1:time_SimEpoch,data_velocity(:,1),'or')
plot(1:time_SimEpoch,element_Vx3(:,1).*5,'.b')
xlim([1 time_SimEpoch])
% ylim([-10 5])
legend({'velocity'  'Vx3'}...
    ,'FontName','ＭＳ 明朝','fontsize',9)
%%

figure
grid on
hold on
axis equal
plot(data_pos_Ref_enu(:,1),data_pos_Ref_enu(:,2),'.r')
% plot(KF_x(:,1),KF_y(:,1),'.b')
plot(data_pos_Obs_enu(:,1),data_pos_Obs_enu(:,2),'.b')
% for i = 1:length(KF_x)-1
%     quiver(KF_x(i), KF_y(i), KF_vec_x(i), KF_vec_y(i), 0)
% end

for i = 1:length(KF_x)-1
    quiver(data_pos_Obs_enu(i,1), data_pos_Obs_enu(i,2), element_Vx3(i), element_Vy3(i),...
        0,'Color','b')
end
for i = 1:length(KF_x)-1
    quiver(data_pos_Obs_enu(i,1), data_pos_Obs_enu(i,2), data_velocity(i,1).*const_dt_Gnss, ...
        data_velocity(i,2).*const_dt_Gnss,0,'Color','r')
end
xlabel('East [m]')
ylabel('North [m]')
title('trajectory')
% title(['trajectory_' num2str(k)])
legend({'reference' 'KF後'},'FontName','ＭＳ 明朝','fontsize',9)

area0 = 947:960;
figure
grid on
hold on
axis equal
plot(data_pos_Ref_enu(area0,1),data_pos_Ref_enu(area0,2),'.r')
% plot(KF_x(area0,1),KF_y(area0,1),'.b')
plot(data_pos_Obs_enu(area0,1),data_pos_Obs_enu(area0,2),'.b')
% for i = 1:length(KF_x)-1
%     quiver(KF_x(i), KF_y(i), KF_vec_x(i), KF_vec_y(i), 0)
% end

for i = area0
    quiver(data_pos_Obs_enu(i,1), data_pos_Obs_enu(i,2), element_Vx3(i), element_Vy3(i),...
        0,'Color','b')
end
for i = area0
    quiver(data_pos_Obs_enu(i,1), data_pos_Obs_enu(i,2), data_velocity(i,1).*const_dt_Gnss, ...
        data_velocity(i,2).*const_dt_Gnss,0,'Color','r')
end
xlabel('East [m]')
ylabel('North [m]')
title('trajectory')
% title(['trajectory_' num2str(k)])
legend({'reference' 'KF後'},'FontName','ＭＳ 明朝','fontsize',9)


%%
% base plot
target = 0:0.1:10;
score = EvalFunc_TGR(target);
figure
grid on
hold on
plot(target,score,'.b')

figure
grid on
hold on
plot(target,score,'.b')
xscale log

%
PR3d = sqrt(status_PositionResidual(:,1).^2 ...
           +status_PositionResidual(:,2).^2 ...
           +status_PositionResidual(:,3).^2);
target = PR3d;
score = EvalFunc_TGR(target);
disp(num2str(mean(score)))
figure
grid on
hold on
plot(target,score,'.b')
xscale log
figure
grid on
hold on
plot(target,score,'.b')
xscale log
yscale log

%
PRKF(:,1:3) = KF_enu(:,1:3) - data_pos_Ref_enu(:,1:3);
PRKF3d = sqrt(PRKF(:,1).^2 ...
           +PRKF(:,2).^2 ...
           +PRKF(:,3).^2);
target = PRKF3d;
score = EvalFunc_TGR(target);
disp(num2str(mean(score)))
figure
grid on
hold on
plot(target,score,'.b')
xscale log
figure
grid on
hold on
plot(target,score,'.b')
xscale log
yscale log

area0 = 947:960;
target = PRKF3d(area0);
score = EvalFunc_TGR(target);
disp(num2str(mean(score)))
figure
grid on
hold on
plot(target,score,'.b')
xscale log
figure
grid on
hold on
plot(target,score,'.b')
xscale log
yscale log

clear PR3d PRKF3d

%%
clear status_PositionResidual
status_PositionResidual(:,1:3) = state_pos_Eagleye_enu(:,1:3) - data_pos_Ref_enu(:,1:3);

PR3d = sqrt(status_PositionResidual(:,1).^2 ...
           +status_PositionResidual(:,2).^2 ...
           +status_PositionResidual(:,3).^2);
PR2d = sqrt(status_PositionResidual(:,1).^2 ...
           +status_PositionResidual(:,2).^2);

figure
grid on
hold on
plot(1:time_SimEpoch,abs(status_PositionResidual(:,1)),'or')
plot(1:time_SimEpoch,status_pseudo_observation_error(:,1),'.b')
xlim([1 time_SimEpoch])


POE3d = sqrt(status_pseudo_observation_error(:,1).^2 ...
           +status_pseudo_observation_error(:,2).^2 ...
           +status_pseudo_observation_error(:,3).^2);
POE2d = sqrt(status_pseudo_observation_error(:,1).^2 ...
           +status_pseudo_observation_error(:,2).^2);
TD3d = sqrt(status_timedifference(:,1).^2 ...
           +status_timedifference(:,2).^2 ...
           +status_timedifference(:,3).^2);
TD2d = sqrt(status_timedifference(:,1).^2 ...
           +status_timedifference(:,2).^2);
figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionResidual(:,3),'or')
plot(1:time_SimEpoch,status_pseudo_observation_error(:,3),'.b')
plot(1:time_SimEpoch,status_timedifference(:,3),'.g')
xlim([1 time_SimEpoch])

figure
grid on
hold on
plot(1:time_SimEpoch,PR2d(:,1),'or')
plot(1:time_SimEpoch,POE2d(:,1),'.b')
plot(1:time_SimEpoch,TD2d(:,1),'.g')
xlim([1 time_SimEpoch])

figure
grid on
hold on
plot(1:time_SimEpoch,PR2d(:,1),'or')
plot(1:time_SimEpoch,POE2d(:,1),'.b')
plot(1:time_SimEpoch,TD2d(:,1),'.g')
xlim([1 time_SimEpoch])

area0 = 947:960;
figure
grid on
hold on
plot(time_Obs(area0),PR3d(area0,1),'or')
plot(time_Obs(area0),POE3d(area0,1),'.b')
% xlim([1 time_SimEpoch])
figure
grid on
hold on
plot(time_Obs(area0),abs(status_PositionResidual(area0,1)),'or')
plot(time_Obs(area0),status_pseudo_observation_error(area0,1),'.b')
% xlim([1 time_SimEpoch])
figure
grid on
hold on
plot(time_Obs(area0),abs(status_PositionResidual(area0,2)),'or')
plot(time_Obs(area0),status_pseudo_observation_error(area0,2),'.b')
% xlim([1 time_SimEpoch])

%%
figure
grid on
hold on
axis equal
plot(data_pos_Ref_enu(:,1),data_pos_Ref_enu(:,2),'.r','MarkerSize', 12)
plot(state_pos_Eagleye_enu(:,1),state_pos_Eagleye_enu(:,2),'.b','MarkerSize', 12)
plot(KF_enu(:,1),KF_enu(:,2),'.',...
    'Color','#66cdaa','MarkerSize', 12)

% ,'.','Color','#66cdaa','MarkerSize', 12

status_PositionError(:,1:3) = KF_enu(:,1:3) - data_pos_Ref_enu(:,1:3);


figure
grid on
hold on
plot(1:time_SimEpoch,abs(status_PositionError(:,2)),'.',...
    'Color','#66cdaa','MarkerSize', 12)
plot(1:time_SimEpoch,abs(status_PositionResidual(:,2)),'.b')
xlim([1 time_SimEpoch])


figure
grid on
hold on
% plot(1:time_SimEpoch,abs(status_PositionError(:,2)),'.',...
%     'Color','#66cdaa','MarkerSize', 12)
plot(1:time_SimEpoch,abs(status_PositionResidual(:,2)),'b')
xlim([1 time_SimEpoch])

area_x= 760:800;
figure
grid on
hold on
axis equal
plot(data_pos_Ref_enu(area_x,1),data_pos_Ref_enu(area_x,2),'.r','MarkerSize', 12)
plot(state_pos_Eagleye_enu(area_x,1),state_pos_Eagleye_enu(area_x,2),'.b','MarkerSize', 12)
plot(KF_enu(area_x,1),KF_enu(area_x,2),'.',...
    'Color','#66cdaa','MarkerSize', 12)


%%
% 760:800
% 1330:1500
% 1640:1690
% 2300:2360
% 4140:4205
% 4500:4850
% 6330:6400
% 6796:6850


%%

clear status_PositionResidual
status_PositionResidual(:,1:3) = state_pos_Eagleye_enu(:,1:3) - data_pos_Ref_enu(:,1:3);

PR3d = sqrt(status_PositionResidual(:,1).^2 ...
           +status_PositionResidual(:,2).^2 ...
           +status_PositionResidual(:,3).^2);
PR2d = sqrt(status_PositionResidual(:,1).^2 ...
           +status_PositionResidual(:,2).^2);
status_PositionError(:,1:3) = KF_enu(:,1:3) - data_pos_Ref_enu(:,1:3);
PE3d = sqrt(status_PositionError(:,1).^2 ...
           +status_PositionError(:,2).^2 ...
           +status_PositionError(:,3).^2);
PE2d = sqrt(status_PositionError(:,1).^2 ...
           +status_PositionError(:,2).^2);
analize_velocity(:,1) = data_velocity(:,1).*const_dt_Gnss+element_Vx3;
analize_velocity(:,2) = data_velocity(:,2).*const_dt_Gnss+element_Vy3;
analize_velocity(:,3) = data_velocity(:,3).*const_dt_Gnss+element_Vz3;
VL3d = sqrt(analize_velocity(:,1).^2 ...
           +analize_velocity(:,2).^2 ...
           +analize_velocity(:,3).^2);
VL2d = sqrt(analize_velocity(:,1).^2 ...
           +analize_velocity(:,2).^2);
figure
grid on
hold on
plot(1:time_SimEpoch,abs(PE3d(:,1)),'.',...
    'Color','#66cdaa','MarkerSize', 12)
plot(1:time_SimEpoch,abs(PR3d(:,1)),'.b')
plot(1:time_SimEpoch,VL3d,'.r')
xlim([1 time_SimEpoch])
figure
grid on
hold on
plot(1:time_SimEpoch,PE2d,'.',...
    'Color','#66cdaa','MarkerSize', 12)
plot(1:time_SimEpoch,PR2d,'.b')
plot(1:time_SimEpoch,(VL2d.^2)*10,'.r')
xlim([1 time_SimEpoch])

figure
grid on
hold on
plot(1:time_SimEpoch,status_PositionError(:,3),'.',...
    'Color','#66cdaa','MarkerSize', 12)
plot(1:time_SimEpoch,status_PositionResidual(:,3),'.b')
plot(1:time_SimEpoch,(VL2d.^2)*10,'.r')
xlim([1 time_SimEpoch])

%%
load('20241231_analize.mat')

status_PositionError_notAdp(:,1:3) = KF_enu_notAdaptive(:,1:3) - data_pos_Ref_enu(:,1:3);
PEnA3d = sqrt(status_PositionError_notAdp(:,1).^2 ...
           +status_PositionError_notAdp(:,2).^2 ...
           +status_PositionError_notAdp(:,3).^2);
PEnA2d = sqrt(status_PositionError_notAdp(:,1).^2 ...
           +status_PositionError_notAdp(:,2).^2);
figure
grid on
hold on
plot(1:time_SimEpoch,PEnA2d,'xk','MarkerSize', 9)
plot(1:time_SimEpoch,PE2d,'.',...
    'Color','#66cdaa','MarkerSize', 12)
plot(1:time_SimEpoch,PR2d,'.b')
plot(1:time_SimEpoch,(VL2d.^2)*10,'.r')
xlim([1 time_SimEpoch])


figure
grid on
hold on
plot(1:time_SimEpoch,PEnA2d,'xk','MarkerSize', 9)
plot(1:time_SimEpoch,PE2d,'.',...
    'Color','#66cdaa','MarkerSize', 12)
plot(1:time_SimEpoch,PR2d,'.b')
plot(1:time_SimEpoch,(VL2d.^2)*10,'.r')
plot(1:time_SimEpoch,POE2d,'.g')
xlim([1 time_SimEpoch])



%%
ref_vel  = [state_vel_Ref_enu_x state_vel_Ref_enu_y state_vel_Ref_enu_z];

figure
grid on
hold on
plot(ref_vel,analize_error_EKF2d,'.b','MarkerSize', 12)
xlabel('速度 [m/s]');
ylabel('推定値2d誤差 [m]');
title('速度vs2d誤差');


figure
grid on
hold on
plot(ref_vel,abs(analize_error_EKF2d),'.b','MarkerSize', 12)
xlabel('速度 [m/s]');
ylabel('推定値2d誤差 [m]');
title('速度vs2d誤差');
xscale log



%%
% cd ..