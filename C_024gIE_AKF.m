 disp('C_EKF Adaptive')

% load('20241202.mat')
% load('20241203.mat')

state_pos_Eagleye_enu = data_correction_Eagleye_enu;

init_x = data_pos_Ref_enu(1,1);
init_y = data_pos_Ref_enu(1,2);
init_z = data_pos_Ref_enu(1,3);

%%

status_variance_param_obs_noise_gnssX = zeros(time_SimEpoch,1);
status_variance_param_obs_noise_gnssY = zeros(time_SimEpoch,1);
status_variance_param_obs_noise_gnssZ = zeros(time_SimEpoch,1);
status_variance_param_obs_noise_imuV  = zeros(time_SimEpoch,1);
status_movemean_param_obs_noise_gnssX = zeros(time_SimEpoch,1);
status_movemean_param_obs_noise_gnssY = zeros(time_SimEpoch,1);
status_movemean_param_obs_noise_gnssZ = zeros(time_SimEpoch,1);
status_kalmanGain = zeros(1,1,time_SimEpoch);

status_variance_param_obs_noise_gnssvX = zeros(time_SimEpoch,1);
status_variance_param_obs_noise_gnssvY = zeros(time_SimEpoch,1);
status_variance_param_obs_noise_gnssvZ = zeros(time_SimEpoch,1);

status_std2_param_obs_noise_gnss3d = zeros(time_SimEpoch,1);


status_VelocityResidual(:,1) = state_velocity(:,1) - data_velocity_REF(:,1);
status_PositionResidual(:,1:3) = state_pos_Eagleye_enu(:,1:3) - data_pos_Ref_enu(:,1:3);


%
status_timedifference = zeros(time_SimEpoch-1,3);
analisis_velocity = zeros(time_SimEpoch-1,3);
status_pseudo_observation_error = zeros(time_SimEpoch-1,3);
for i = 2:time_SimEpoch
    status_timedifference(i,1:3) = state_pos_Eagleye_enu(i,1:3) - state_pos_Eagleye_enu(i-1,1:3);
end

weight_Eagleye = 1;
weight_Doppler = 1; % fix:2 float:1 tosuru yotei

for i = 2:time_SimEpoch
    analisis_velocity(i,1:3) = (weight_Doppler*data_velocity(i-1,1:3).*const_dt_Gnss.*const_dt_Gnss ...%xxx
        +  weight_Eagleye*state_velocity(i-1,1:3).*const_dt3(i-1))/(weight_Doppler+weight_Eagleye);
    status_pseudo_observation_error(i,1:3) = status_timedifference(i,1:3)...
                                             - analisis_velocity(i,1:3);
end

status_POE2d = sqrt(status_pseudo_observation_error(:,1).^2 ...
                   +status_pseudo_observation_error(:,2).^2);
% for i = 2:time_SimEpoch% 大きさを合わせるxxx
%     lengthratio2d_velocity(i,1) = abs()/abs();
% 
% end
status_pseudo_observation_error(:,1:3) = abs(status_pseudo_observation_error(:,1:3));
% pseudo_observation_error POE
% handle_POE = 0;
% handle_POEnum = 0;
switch handle_POE
    case 0 % non
        analize_POE = status_pseudo_observation_error;
    case 1 % 閾値
        analize_POE = zeros(length(status_pseudo_observation_error),3);
        for i2 = 1:3
            for i = 1:length(status_pseudo_observation_error)
                if status_pseudo_observation_error(i,i2)>=handle_POEnum
                    analize_POE(i,i2) = status_pseudo_observation_error(i,i2);
                else
                    analize_POE(i,i2) = handle_POEnum;
                end
            end
        end
    case 2 % 1次関数
        analize_POE(:,1:3) = handle_POEnum_A .* status_pseudo_observation_error + handle_POEnum_B;
    case 3 % 2次関数
        analize_POE(:,1:3) = handle_POEnum_A .* status_pseudo_observation_error.^2;
    case 4 % 指数関数
        handle_POEnum_B = 1;
        analize_POE(:,1:3) = handle_POEnum_B .* handle_POEnum_A .^ status_pseudo_observation_error;
end



status_POE2d = sqrt(status_pseudo_observation_error(:,1).^2 ...
                   +status_pseudo_observation_error(:,2).^2);
status_VEL2d = sqrt(analisis_velocity(:,1).^2 ...
                   +analisis_velocity(:,2).^2);

% figure
% grid on
% hold on
% plot(1:time_SimEpoch,status_POE2d,'.b')
% plot(1:time_SimEpoch,(status_VEL2d.^2)*100,'.r')
% plot(1:time_SimEpoch,status_POE2d.*status_VEL2d,'.k')
% 
% figure
% grid on
% hold on
% plot(1:time_SimEpoch,status_POE2d.*status_VEL2d,'.k')
% yscale log

handle_velocity = status_POE2d.*status_VEL2d;
%
status_POE2d_diff = zeros(time_SimEpoch,1);
for i = 2:time_SimEpoch
    status_POE2d_diff(i) = (status_POE2d(i) - status_POE2d(i-1))/const_dt_Eagleye;
end
% figure
% grid on
% hold on
% plot(1:time_SimEpoch,status_POE2d,'.r')
% plot(1:time_SimEpoch,abs(status_POE2d_diff),'.b')
for i = 6:time_SimEpoch
    status_POE2d_diff5(i) = (status_POE2d(i) - status_POE2d(i-5))/const_dt_Eagleye;
end
% figure
% grid on
% hold on
% plot(1:time_SimEpoch,status_POE2d,'.r')
% plot(1:time_SimEpoch,abs(status_POE2d_diff5),'.b')
for i = 6:time_SimEpoch
    status_POE2d_diff2(i) = (status_POE2d(i) - status_POE2d(i-2))/const_dt_Eagleye;
    status_POE2d_diff3(i) = (status_POE2d(i) - status_POE2d(i-3))/const_dt_Eagleye;
    status_POE2d_diff4(i) = (status_POE2d(i) - status_POE2d(i-4))/const_dt_Eagleye;
    status_POE2d_diff5(i) = (status_POE2d(i) - status_POE2d(i-5))/const_dt_Eagleye;
end
weight_diff = 1:5;
status_POE2d_diff_5(:) = status_POE2d_diff(:) .*weight_diff(1)...
                        +status_POE2d_diff2(:).*weight_diff(2)...
                        +status_POE2d_diff3(:).*weight_diff(3)...
                        +status_POE2d_diff4(:).*weight_diff(4)...
                        +status_POE2d_diff5(:).*weight_diff(5);
status_POE2d_diff_5(:) = status_POE2d_diff_5(:) ./ sum(weight_diff);
% figure
% grid on
% hold on
% plot(1:time_SimEpoch,status_POE2d,'.r')
% plot(1:time_SimEpoch,abs(status_POE2d_diff_5),'.b')

%
for i = 6:time_SimEpoch
    status_POE_diff(i,1:3)  = (status_pseudo_observation_error(i,1:3)...
                           - status_pseudo_observation_error(i-1,1:3))/const_dt_Eagleye;
    status_POE_diff2(i,1:3) = (status_pseudo_observation_error(i,1:3)...
                           - status_pseudo_observation_error(i-2,1:3))/const_dt_Eagleye;
    status_POE_diff3(i,1:3) = (status_pseudo_observation_error(i,1:3) ...
                           - status_pseudo_observation_error(i-3,1:3))/const_dt_Eagleye;
    status_POE_diff4(i,1:3) = (status_pseudo_observation_error(i,1:3)...
                           - status_pseudo_observation_error(i-4,1:3))/const_dt_Eagleye;
    status_POE_diff5(i,1:3) = (status_pseudo_observation_error(i,1:3)...
                           - status_pseudo_observation_error(i-5,1:3))/const_dt_Eagleye;
end
weight_diff = 1:5;
status_POE_diff_5(:,1:3) = status_POE_diff(:,1:3) .*weight_diff(1)...
                          +status_POE_diff2(:,1:3).*weight_diff(2)...
                          +status_POE_diff3(:,1:3).*weight_diff(3)...
                          +status_POE_diff4(:,1:3).*weight_diff(4)...
                          +status_POE_diff5(:,1:3).*weight_diff(5);
status_POE_diff_5(:,1:3) = status_POE_diff_5(:,1:3) ./ sum(weight_diff);
% figure
% grid on
% hold on
% plot(1:time_SimEpoch,status_pseudo_observation_error(:,1),'.r')
% plot(1:time_SimEpoch,abs(status_POE_diff_5(:,1)),'.b')
% 
% figure
% grid on
% hold on
% plot(1:time_SimEpoch,status_pseudo_observation_error(:,2),'.r')
% plot(1:time_SimEpoch,abs(status_POE_diff_5(:,2)),'.b')
% figure
% grid on
% hold on
% plot(1:time_SimEpoch,status_pseudo_observation_error(:,1),'.r')
% plot(1:time_SimEpoch,1./(1+abs(status_POE_diff_5(:,1))),'.b')
% 
% figure
% grid on
% hold on
% plot(1:time_SimEpoch,status_pseudo_observation_error(:,2),'.r')
% plot(1:time_SimEpoch,1./(1+abs(status_POE_diff_5(:,2))),'.b')

%
status_timedifference_ref = zeros(time_SimEpoch-1,3);
status_dist_refer_enu = zeros(time_SimEpoch-1,3);
status_dist_refer_3d = zeros(time_SimEpoch-1,3);
for i = 2:time_SimEpoch
    status_timedifference_ref(i,1:3) = data_pos_Ref_enu(i,1:3) - data_pos_Ref_enu(i-1,1:3);
    status_dist_refer_enu(i,1:3) = status_timedifference(i,1:3) - status_timedifference_ref(i,1:3);
    status_dist_refer_3d(i,1) = sqrt(status_dist_refer_enu(i,1).^2 + status_dist_refer_enu(i,2).^2 ...
        +status_dist_refer_enu(i,3).^2);
end
% figure
% grid on
% hold on
% plot(1:time_Simepoch,dist_refer_enu(:,1))
% figure
% grid on
% hold on
% plot(1:time_Simepoch,dist_refer_enu(:,2))
% figure
% grid on
% hold on
% plot(1:time_Simepoch,dist_refer_enu(:,3))
% figure
% grid on
% hold on
% plot(1:time_Simepoch,dist_refer_3d(:,1))

% figure
% grid on
% hold on
% plot(1:time_Simepoch,time_difference(:,1),'.b')
% plot(1:time_Simepoch,time_difference_ref(:,1),'.r')

% figure
% grid on
% hold on
% plot(1:time_Simepoch,time_difference(:,1),'.b')
% % plot(1:time_Simepoch,velocity(:,1),'.g')
% plot(1:time_Simepoch,PositionResidual(:,1),'.r')
% xlabel('time [s]')
% ylabel('diff [m]')
% title('時間差分｜誤差 East')
% legend({'時間差分' '誤差'},'FontName','ＭＳ 明朝','fontsize',9)
% 
% figure
% grid on
% hold on
% plot(1:time_Simepoch,pseudo_observation_error(:,1),'.b')
% % plot(1:time_Simepoch,velocity(:,1),'.g')
% plot(1:time_Simepoch,PositionResidual(:,1),'.r')
% xlabel('time [s]')
% ylabel('diff [m]')
% title('距離差分｜誤差 East')
% legend({'時間差分' '誤差'},'FontName','ＭＳ 明朝','fontsize',9)
% 
% time_diff_3d(:,1) = sqrt(time_difference(:,1).^2+time_difference(:,2).^2+time_difference(:,3).^2);
% PR_3d(:,1) = sqrt(PositionResidual(:,1).^2+PositionResidual(:,2).^2+PositionResidual(:,3).^2);
% figure
% grid on
% hold on
% plot(1:time_Simepoch,time_diff_3d(:,1),'.b')
% % plot(1:time_Simepoch,velocity(:,1),'.g')
% plot(1:time_Simepoch,PR_3d(:,1),'.r')
% xlabel('time [s]')
% ylabel('diff [m]')
% title('時間差分｜誤差 3d')
% legend({'時間差分' '誤差'},'FontName','ＭＳ 明朝','fontsize',9)




%
status_dist_refer_3d_loc = zeros(time_SimEpoch-1,1);
status_dist_refer_3d_ref = zeros(time_SimEpoch-1,1);
for i = 2:time_SimEpoch
    status_dist_refer_3d_loc(i,1) = sqrt(status_timedifference(i,1).^2 + status_timedifference(i,2).^2 ...
        +status_timedifference(i,3).^2);
    status_dist_refer_3d_ref(i,1) = sqrt(status_timedifference_ref(i,1).^2 + status_timedifference_ref(i,2).^2 ...
        +status_timedifference_ref(i,3).^2);
end
% figure
% grid on
% hold on
% plot(1:time_Simepoch,dist_refer_3d_loc(:,1))
% figure
% grid on
% hold on
% plot(1:time_Simepoch,dist_refer_3d_ref(:,1))
% figure
% grid on
% hold on
% plot(1:time_Simepoch,dist_refer_3d_loc(:,1)-dist_refer_3d_ref(:,1))
%
% figure
% grid on
% hold on
% plot(1:time_Simepoch,dist_refer_3d_loc(:,1),'.b')
% plot(1:time_Simepoch,dist_refer_3d_ref(:,1),'.r')
% ylim([0 3])
%
% dist_movmean = movmean(dist_refer_3d_loc(:,1),1);
% dist_error(:,1) = dist_movmean(:,1) - dist_refer_3d_ref(:,1);
% figure
% grid on
% hold on
% plot(1:time_Simepoch,dist_movmean(:,1),'.b')
% plot(1:time_Simepoch,dist_refer_3d_ref(:,1),'.r')
% plot(1:time_Simepoch,dist_error(:,1),'xg')
% text(7500,1,num2str(sum(abs(dist_error))))
% dist_movmean = movmean(dist_refer_3d_loc(:,1),10);
% dist_error(:,1) = dist_movmean(:,1) - dist_refer_3d_ref(:,1);
% figure
% grid on
% hold on
% plot(1:time_Simepoch,dist_movmean(:,1),'.b')
% plot(1:time_Simepoch,dist_refer_3d_ref(:,1),'.r')
% plot(1:time_Simepoch,dist_error(:,1),'xg')
% text(7500,1,num2str(sum(abs(dist_error))))
% dist_movmean = movmean(dist_refer_3d_loc(:,1),100);
% dist_error(:,1) = dist_movmean(:,1) - dist_refer_3d_ref(:,1);
% figure
% grid on
% hold on
% plot(1:time_Simepoch,dist_movmean(:,1),'.b')
% plot(1:time_Simepoch,dist_refer_3d_ref(:,1),'.r')
% plot(1:time_Simepoch,dist_error(:,1),'xg')
% text(7500,1,num2str(sum(abs(dist_error))))
%
% dist_movmean = movmean(dist_refer_3d_loc(:,1),100);
% dist_movmean10 = movmean(dist_refer_3d_loc(:,1),10);
% dist_error_movemean(:,1) = dist_movmean(:,1) - dist_movmean10(:,1);
% figure
% grid on
% hold on
% plot(1:time_Simepoch,dist_movmean(:,1),'.b')
% plot(1:time_Simepoch,dist_refer_3d_ref(:,1),'.r')
% plot(1:time_Simepoch,dist_error_movemean(:,1),'xg')
% text(7500,1,num2str(sum(abs(dist_error_movemean))))
% % clear dist_movmean dist_refer_3d_loc dist_refer_3d_ref

% 自作movemean 今の値に近いほど影響が大きい

%{
tic
figure
grid on
hold on
dist_error = zeros(time_Simepoch,1);
for i = 1:time_Simepoch
    dist_movmean = movmean(dist_refer_3d_loc(:,1),i);
    dist_error(:,1) = dist_movmean(:,1) - dist_refer_3d_ref(:,1);
    dist_error_mean = mean(dist_error);
    plot(i,dist_error_mean,'.')
end
toc
figure
grid on
hold on
plot(1:time_Simepoch,dist_error(:,1),'.b')

tic
figure
grid on
hold on
dist_error = zeros(time_Simepoch,1);
for i = 1:time_Simepoch
    dist_movmean = movmean(dist_refer_3d_loc(:,1),i);
    dist_error(:,1) = dist_movmean(:,1) - dist_refer_3d_ref(:,1);

    plot(1:time_Simepoch,dist_error,'.')
end
toc

tic
figure
grid on
hold on
dist_error = zeros(time_Simepoch,1);
for i = 1:time_Simepoch
    dist_movmean = movmean(dist_refer_3d_loc(:,1),i);
    dist_error(:,1) = dist_movmean(:,1) - dist_refer_3d_ref(:,1);
    dist_error_mean = mean(abs(dist_error));
    plot(i,dist_error_mean,'.')
end
toc

tic
figure
grid on
hold on
dist_error = zeros(time_Simepoch,1);
for i = 1:time_Simepoch
    dist_movmean = movmean(dist_refer_3d_loc(:,1),i);
    dist_error(:,1) = dist_movmean(:,1) - dist_refer_3d_ref(:,1);
    dist_error_sum = sum(abs(dist_error));
    plot(i,dist_error_sum,'.')
end
toc




% y = HPF(dist_refer_3d_loc);

%}

% velocity
%%
clear status_PositionResidual
% status_PositionResidual(:,1:3) = status_timedifference(:,1:3);

status_PositionResidual(:,1:3) = analize_POE(:,1:3);
status_VelocityResidual(:,1) = status_POE2d(:,1);

% figure
% grid on
% hold on
% plot(1:time_Simepoch,pseudo_observation_error(:,1),'.b')
% plot(1:time_Simepoch,PositionResidual(:,1),'.r')
%
%
% figure
% grid on
% hold on
% plot(1:time_Simepoch,time_difference(:,1).*pseudo_observation_error(:,1),'.b')
% plot(1:time_Simepoch,PositionResidual(:,1),'.r')


%%
% for k = 1:10:1000
%     windowSize = k;
% for k = [2 50 100 360 390]
%     windowSize = k;

    % % k 5 10 40


    %%
    windowSize = 30;
    status_velocity = analisis_velocity;

    %%
    for i = 1:time_SimEpoch
        if i == 1          %変数に初期値を代入
            KF_x(i) = init_x;
            KF_y(i) = init_y;
            KF_z(i) = init_z;
            KF_roll(i) = init_roll;
            KF_pitch(i) = init_pitch;
            KF_yaw(i) = init_yaw;
            KF_velocity(i) = init_velocity;
            KF_covariance(i,:) = setVecCovariance(init_covariance,7); %init_covarianceを1列目に入れている。（5×5の行列の25個の要素を1行に並べてる）
            P = init_covariance;
            continue
        end


        % Predict　予測 t=k-1までの状態からt=kの状態を予測
        Pre_KF_roll = KF_roll(i-1) + (data_struct_Eagleye.Roll_rate(i-1,1)+data_struct_Eagleye.Pitch_rate(i-1,1)*sin(KF_roll(i-1))*tan(KF_pitch(i-1))+data_yawrate(i-1,1)*cos(KF_roll(i-1))*tan(KF_pitch(i-1)))*const_dt3(i); %
        % Pre_KF_pitch = KF_pitch(i-1) + (Pre_KF_pitchrate*cos(KF_roll(i-1))-Pre_KF_yawrate*sin(KF_roll(i-1)))*const_dt3(i); %
        Pre_KF_pitch = KF_pitch(i-1) + (data_struct_Eagleye.Pitch_rate(i-1,1)*cos(KF_roll(i-1))-data_yawrate(i-1,1)*sin(KF_roll(i-1)))*const_dt3(i);
        Pre_KF_yaw = KF_yaw(i-1) + data_yawrate(i-1,1)*const_dt3(i); %PΘ(k)=Θ(k-1)+yawrate(k)*const_dt3
        Pre_KF_z = KF_z(i-1) + sin(KF_pitch(i-1))*status_velocity(i)*const_dt3(i);

        % option_calc_type = 0;
        if handle_velocity < 0.001
            gain_velocity = 0;
        else
            gain_velocity = 1;
        end

        if option_calc_type==0
            Pre_KF_x = KF_x(i-1) - KF_velocity(i-1)*(data_yawrate(i-1,1)^(-1))*(-cos(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))+cos(KF_yaw(i-1,1))).*gain_velocity;
            Pre_KF_y = KF_y(i-1) - KF_velocity(i-1)*(data_yawrate(i-1,1)^(-1))*(sin(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))-sin(KF_yaw(i-1,1))).*gain_velocity;
        end
        KF_vec_x(i-1,1) = -KF_velocity(i-1)*(data_yawrate(i-1,1)^(-1))...
            *(-cos(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))+cos(KF_yaw(i-1,1)));
        KF_vec_y(i-1,1) = -KF_velocity(i-1)*(data_yawrate(i-1,1)^(-1))...
            *(sin(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))-sin(KF_yaw(i-1,1)));


        Pre_KF_velocity(1,1) = KF_velocity(i-1) + data_acc_Eagleye_enu3d(i-1,1)*const_dt3(i);


        % A(i)=(Pre_KF_pitchrate*cos(KF_roll(i-1))-Pre_KF_yawrate*sin(KF_roll(i-1)))*const_dt3(i);


        %ヤコビアン

        jF = ...
            [1, 0, 0, 0, 0, status_velocity(i-1)*(data_yawrate(i-1,1)^(-1))*(sin(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))-sin(KF_yaw(i-1,1))), 0
            0, 1, 0, 0, 0, status_velocity(i-1)*(data_yawrate(i-1,1)^(-1))*(cos(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))-cos(KF_yaw(i-1,1))), 0
            0, 0, 1, 0, 0, 0, 0
            0, 0, 0, 1+(data_struct_Eagleye.Pitch_rate(i-1,1)*cos(KF_roll(i-1))*tan(KF_pitch(i-1))-data_yawrate(i-1,1)*sin(KF_roll(i-1))*tan(KF_pitch(i-1)))*const_dt3(i), (data_struct_Eagleye.Pitch_rate(i-1,1)*sin(KF_roll(i-1))/(cos(KF_pitch(i-1))*cos(KF_pitch(i-1)))+data_yawrate(i-1,1)*cos(KF_roll(i-1))/(cos(KF_pitch(i-1))*cos(KF_pitch(i-1))))*const_dt3(i), 0, 0
            0, 0, 0, (-data_struct_Eagleye.Pitch_rate(i-1,1)*sin(KF_roll(i-1))-data_yawrate(i-1,1)*cos(KF_roll(i-1)))*const_dt3(i), 1, 0, 0
            0, 0, 0, 0, 0, 1, 0
            0, 0, 0, 0, 0, 0, 1];


        %予測値の行列に値を代入
        Pre_KF_status = [Pre_KF_x, Pre_KF_y, Pre_KF_z, Pre_KF_roll, Pre_KF_pitch, Pre_KF_yaw, Pre_KF_velocity]';
        %予測値の共分散を計算
        Pre_KF_covariance = jF*P*jF'+Q; %P(k|k-1)=F(k)P(k-1|k-1)F^T(k)+Q

        %%
        %観測値の方位角をカルマンフィルタの推定値に近づける
        handle_Heading_GNSS_control(i,1)=Pre_KF_yaw-handle_Heading_GNSS(i,1);
        handle_Heading_GNSS(i,1)=handle_Heading_GNSS(i,1);
        if handle_Heading_GNSS_control(i,1)<-pi
            handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)-2*pi;
        end
        if handle_Heading_GNSS_control(i,1)<-3*pi
            handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)-4*pi;
        end
        if handle_Heading_GNSS_control(i,1)<-5*pi
            handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)-6*pi;
        end
        if handle_Heading_GNSS_control(i,1)<-7*pi
            handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)-8*pi;
        end
        if handle_Heading_GNSS_control(i,1)<-9*pi
            handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)-10*pi;
        end
        if handle_Heading_GNSS_control(i,1)>pi
            handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)+2*pi;
        end
        if handle_Heading_GNSS_control(i,1)>3*pi
            handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)+4*pi;
        end
        if handle_Heading_GNSS_control(i,1)>5*pi
            handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)+6*pi;
        end
        if handle_Heading_GNSS_control(i,1)>7*pi
            handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)+8*pi;
        end
        if handle_Heading_GNSS_control(i,1)>9*pi
            handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)+10*pi;
        end
        state_gnss_yaw=handle_Heading_GNSS;


        %%
        %adaptive true
        % clear windowSIze
        % windowSize = 1000; % 2以上
        window_center = floor(windowSize/2)+mod(windowSize,2);
        win = windowSize-window_center;
        state_PR3d = sqrt(status_PositionResidual(:,1).^2+status_PositionResidual(:,2).^2+status_PositionResidual(:,3).^2);

        switch mod(windowSize,2)
            case 1
                if i < window_center
                    state_dispersion_param_obs_noise_gnss_x = var(status_PositionResidual(1:i+win,1));%観測ノイズ　x座標 分散
                    state_dispersion_param_obs_noise_gnss_y = var(status_PositionResidual(1:i+win,2));%観測ノイズ　y座標 分散
                    state_dispersion_param_obs_noise_gnss_z = var(status_PositionResidual(1:i+win,3));%観測ノイズ　z座標 分散
                    state_dispersion_param_obs_noise_imu_v = var(status_VelocityResidual(1:i+win,1));%観測ノイズ　velosity
                    state_movemean_param_obs_noise_gnss_x = mean(status_PositionResidual(1:i+win,1));%観測ノイズ　x座標 分散
                    state_movemean_param_obs_noise_gnss_y = mean(status_PositionResidual(1:i+win,2));%観測ノイズ　y座標 分散
                    state_movemean_param_obs_noise_gnss_z = mean(status_PositionResidual(1:i+win,3));%観測ノイズ　z座標 分散
                    state_std2_param_obs_noise_gnss = std2(state_PR3d(1:i+win,1));%観測ノイズ 標準偏差
                elseif i > time_SimEpoch-win
                    state_dispersion_param_obs_noise_gnss_x = var(status_PositionResidual(i-win:end,1));%観測ノイズ　x座標 分散
                    state_dispersion_param_obs_noise_gnss_y = var(status_PositionResidual(i-win:end,2));%観測ノイズ　y座標 分散
                    state_dispersion_param_obs_noise_gnss_z = var(status_PositionResidual(i-win:end,3));%観測ノイズ　z座標 分散
                    state_dispersion_param_obs_noise_imu_v = var(status_VelocityResidual(i-win:end,1));%観測ノイズ　velosity
                    state_movemean_param_obs_noise_gnss_x = mean(status_PositionResidual(i-win:end,1));%観測ノイズ　x座標 分散
                    state_movemean_param_obs_noise_gnss_y = mean(status_PositionResidual(i-win:end,2));%観測ノイズ　y座標 分散
                    state_movemean_param_obs_noise_gnss_z = mean(status_PositionResidual(i-win:end,3));%観測ノイズ　z座標 分散
                    state_std2_param_obs_noise_gnss = std2(state_PR3d(i-win:end,1));%観測ノイズ\ 標準偏差
                else
                    state_dispersion_param_obs_noise_gnss_x = var(status_PositionResidual(i-win:i+win,1));%観測ノイズ　x座標 分散
                    state_dispersion_param_obs_noise_gnss_y = var(status_PositionResidual(i-win:i+win,2));%観測ノイズ　y座標 分散
                    state_dispersion_param_obs_noise_gnss_z = var(status_PositionResidual(i-win:i+win,3));%観測ノイズ　z座標 分散
                    state_dispersion_param_obs_noise_imu_v = var(status_VelocityResidual(i-win:i+win,1));%観測ノイズ　velosity
                    state_movemean_param_obs_noise_gnss_x = mean(status_PositionResidual(i-win:i+win,1));%観測ノイズ　x座標 分散
                    state_movemean_param_obs_noise_gnss_y = mean(status_PositionResidual(i-win:i+win,2));%観測ノイズ　y座標 分散
                    state_movemean_param_obs_noise_gnss_z = mean(status_PositionResidual(i-win:i+win,3));%観測ノイズ　z座標 分散
                    state_std2_param_obs_noise_gnss = std2(state_PR3d(i-win:i+win,1));%観測ノイズ\ 標準偏差
                end
            case 0
                if i < window_center
                    state_dispersion_param_obs_noise_gnss_x = var(status_PositionResidual(1:i+win,1));%観測ノイズ　x座標 分散
                    state_dispersion_param_obs_noise_gnss_y = var(status_PositionResidual(1:i+win,2));%観測ノイズ　y座標 分散
                    state_dispersion_param_obs_noise_gnss_z = var(status_PositionResidual(1:i+win,3));%観測ノイズ　z座標 分散
                    state_dispersion_param_obs_noise_imu_v = var(status_VelocityResidual(1:i+win,1));%観測ノイズ　velosity
                    state_movemean_param_obs_noise_gnss_x = mean(status_PositionResidual(1:i+win,1));%観測ノイズ　x座標 分散
                    state_movemean_param_obs_noise_gnss_y = mean(status_PositionResidual(1:i+win,2));%観測ノイズ　y座標 分散
                    state_movemean_param_obs_noise_gnss_z = mean(status_PositionResidual(1:i+win,3));%観測ノイズ　z座標 分散
                    state_std2_param_obs_noise_gnss = std2(state_PR3d(1:i+win,1));%観測ノイズ\ 標準偏差
                elseif i > time_SimEpoch-win
                    state_dispersion_param_obs_noise_gnss_x = var(status_PositionResidual(i-(win-1):end,1));%観測ノイズ　x座標 分散
                    state_dispersion_param_obs_noise_gnss_y = var(status_PositionResidual(i-(win-1):end,2));%観測ノイズ　y座標 分散
                    state_dispersion_param_obs_noise_gnss_z = var(status_PositionResidual(i-(win-1):end,3));%観測ノイズ　z座標 分散
                    state_dispersion_param_obs_noise_imu_v = var(status_VelocityResidual(i-(win-1):end,1));%観測ノイズ　velosity
                    state_movemean_param_obs_noise_gnss_x = mean(status_PositionResidual(i-(win-1):end,1));%観測ノイズ　x座標 分散
                    state_movemean_param_obs_noise_gnss_y = mean(status_PositionResidual(i-(win-1):end,2));%観測ノイズ　y座標 分散
                    state_movemean_param_obs_noise_gnss_z = mean(status_PositionResidual(i-(win-1):end,3));%観測ノイズ　z座標 分散
                    state_std2_param_obs_noise_gnss = std2(state_PR3d(i-(win-1):end,1));%観測ノイズ\ 標準偏差
                else
                    state_dispersion_param_obs_noise_gnss_x = var(status_PositionResidual(i-(win-1):i+win,1));%観測ノイズ　x座標 分散
                    state_dispersion_param_obs_noise_gnss_y = var(status_PositionResidual(i-(win-1):i+win,2));%観測ノイズ　y座標 分散
                    state_dispersion_param_obs_noise_gnss_z = var(status_PositionResidual(i-(win-1):i+win,3));%観測ノイズ　z座標 分散
                    state_dispersion_param_obs_noise_imu_v = var(status_VelocityResidual(i-(win-1):i+win,1));%観測ノイズ　velosity
                    state_movemean_param_obs_noise_gnss_x = mean(status_PositionResidual(i-(win-1):i+win,1));%観測ノイズ　x座標 分散
                    state_movemean_param_obs_noise_gnss_y = mean(status_PositionResidual(i-(win-1):i+win,2));%観測ノイズ　y座標 分散
                    state_movemean_param_obs_noise_gnss_z = mean(status_PositionResidual(i-(win-1):i+win,3));%観測ノイズ　z座標 分散
                    state_std2_param_obs_noise_gnss = std2(state_PR3d(i-(win-1):i+win,1));%観測ノイズ\ 標準偏差
                end
        end



        % VR3d = sqrt(VelocityResidual(:,1).^2+VelocityResidual(:,2).^2+VelocityResidual(:,3).^2);
        %  switch mod(windowSize,2)
        %      case 1
        %          if i < center_window
        %              dispersion_param_obs_noise_gnss_vx = var(VelocityResidual(1:i+win,1));%観測ノイズ　x座標 分散
        %              dispersion_param_obs_noise_gnss_vy = var(VelocityResidual(1:i+win,2));%観測ノイズ　y座標 分散
        %              dispersion_param_obs_noise_gnss_vz = var(VelocityResidual(1:i+win,3));%観測ノイズ　z座標 分散
        %              movemean_param_obs_noise_gnss_vx = mean(VelocityResidual(1:i+win,1));%観測ノイズ　x座標 分散
        %              movemean_param_obs_noise_gnss_vy = mean(VelocityResidual(1:i+win,2));%観測ノイズ　y座標 分散
        %              movemean_param_obs_noise_gnss_vz = mean(VelocityResidual(1:i+win,3));%観測ノイズ　z座標 分散
        %              std2_param_obs_noise_gnssv = std2(VR3d(1:i+win,1));%観測ノイズ 標準偏差
        %          elseif i > time_Simepoch-win
        %              dispersion_param_obs_noise_gnss_vx = var(VelocityResidual(i-win:end,1));%観測ノイズ　x座標 分散
        %              dispersion_param_obs_noise_gnss_vy = var(VelocityResidual(i-win:end,2));%観測ノイズ　y座標 分散
        %              dispersion_param_obs_noise_gnss_vz = var(VelocityResidual(i-win:end,3));%観測ノイズ　z座標 分散
        %              movemean_param_obs_noise_gnss_vx = mean(VelocityResidual(i-win:end,1));%観測ノイズ　x座標 分散
        %              movemean_param_obs_noise_gnss_vy = mean(VelocityResidual(i-win:end,2));%観測ノイズ　y座標 分散
        %              movemean_param_obs_noise_gnss_vz = mean(VelocityResidual(i-win:end,3));%観測ノイズ　z座標 分散
        %              std2_param_obs_noise_gnssv = std2(VR3d(i-win:end,1));%観測ノイズ\ 標準偏差
        %          else
        %              dispersion_param_obs_noise_gnss_vx = var(VelocityResidual(i-win:i+win,1));%観測ノイズ　x座標 分散
        %              dispersion_param_obs_noise_gnss_vy = var(VelocityResidual(i-win:i+win,2));%観測ノイズ　y座標 分散
        %              dispersion_param_obs_noise_gnss_vz = var(VelocityResidual(i-win:i+win,3));%観測ノイズ　z座標 分散
        %              movemean_param_obs_noise_gnss_vx = mean(VelocityResidual(i-win:i+win,1));%観測ノイズ　x座標 分散
        %              movemean_param_obs_noise_gnss_vy = mean(VelocityResidual(i-win:i+win,2));%観測ノイズ　y座標 分散
        %              movemean_param_obs_noise_gnss_vz = mean(VelocityResidual(i-win:i+win,3));%観測ノイズ　z座標 分散
        %              std2_param_obs_noise_gnssv = std2(VR3d(i-win:i+win,1));%観測ノイズ\ 標準偏差
        %          end
        %      case 0
        %          if i < center_window
        %              dispersion_param_obs_noise_gnss_vx = var(VelocityResidual(1:i+win,1));%観測ノイズ　x座標 分散
        %              dispersion_param_obs_noise_gnss_vy = var(VelocityResidual(1:i+win,2));%観測ノイズ　y座標 分散
        %              dispersion_param_obs_noise_gnss_vz = var(VelocityResidual(1:i+win,3));%観測ノイズ　z座標 分散
        %              movemean_param_obs_noise_gnss_vx = mean(VelocityResidual(1:i+win,1));%観測ノイズ　x座標 分散
        %              movemean_param_obs_noise_gnss_vy = mean(VelocityResidual(1:i+win,2));%観測ノイズ　y座標 分散
        %              movemean_param_obs_noise_gnss_vz = mean(VelocityResidual(1:i+win,3));%観測ノイズ　z座標 分散
        %              std2_param_obs_noise_gnssv = std2(VR3d(1:i+win,1));%観測ノイズ\ 標準偏差
        %          elseif i > time_Simepoch-win
        %              dispersion_param_obs_noise_gnss_vx = var(VelocityResidual(i-(win-1):end,1));%観測ノイズ　x座標 分散
        %              dispersion_param_obs_noise_gnss_vy = var(VelocityResidual(i-(win-1):end,2));%観測ノイズ　y座標 分散
        %              dispersion_param_obs_noise_gnss_vz = var(VelocityResidual(i-(win-1):end,3));%観測ノイズ　z座標 分散
        %              movemean_param_obs_noise_gnss_vx = mean(VelocityResidual(i-(win-1):end,1));%観測ノイズ　x座標 分散
        %              movemean_param_obs_noise_gnss_vy = mean(VelocityResidual(i-(win-1):end,2));%観測ノイズ　y座標 分散
        %              movemean_param_obs_noise_gnss_vz = mean(VelocityResidual(i-(win-1):end,3));%観測ノイズ　z座標 分散
        %              std2_param_obs_noise_gnssv = std2(VR3d(i-(win-1):end,1));%観測ノイズ\ 標準偏差
        %          else
        %              dispersion_param_obs_noise_gnss_vx = var(VelocityResidual(i-(win-1):i+win,1));%観測ノイズ　x座標 分散
        %              dispersion_param_obs_noise_gnss_vy = var(VelocityResidual(i-(win-1):i+win,2));%観測ノイズ　y座標 分散
        %              dispersion_param_obs_noise_gnss_vz = var(VelocityResidual(i-(win-1):i+win,3));%観測ノイズ　z座標 分散
        %              movemean_param_obs_noise_gnss_vx = mean(VelocityResidual(i-(win-1):i+win,1));%観測ノイズ　x座標 分散
        %              movemean_param_obs_noise_gnss_vy = mean(VelocityResidual(i-(win-1):i+win,2));%観測ノイズ　y座標 分散
        %              movemean_param_obs_noise_gnss_vz = mean(VelocityResidual(i-(win-1):i+win,3));%観測ノイズ　z座標 分散
        %              std2_param_obs_noise_gnssv = std2(VR3d(i-(win-1):i+win,1));%観測ノイズ\ 標準偏差
        %          end
        %  end
        %

        
        % state_dispersion_param_obs_noise_gnss_x = state_dispersion_param_obs_noise_gnss_x.*(1./(1+abs(status_POE_diff_5(i,1))));
        % state_dispersion_param_obs_noise_gnss_y = state_dispersion_param_obs_noise_gnss_y.*(1./(1+abs(status_POE_diff_5(i,2))));
        % state_dispersion_param_obs_noise_gnss_z = state_dispersion_param_obs_noise_gnss_z.*(1./(1+abs(status_POE_diff_5(i,3))));
        state_dispersion_param_obs_noise_imu_v  = state_dispersion_param_obs_noise_imu_v .*(1./(1+abs(status_POE_diff_5(i,3))));
        
        state_dispersion_param_obs_noise_gnss_x = state_dispersion_param_obs_noise_gnss_x.*(1+gain_velocity);
        state_dispersion_param_obs_noise_gnss_y = state_dispersion_param_obs_noise_gnss_y.*(1+gain_velocity);
        state_dispersion_param_obs_noise_gnss_z = state_dispersion_param_obs_noise_gnss_z.*(1+gain_velocity);
        % state_dispersion_param_obs_noise_imu_v  = state_dispersion_param_obs_noise_imu_v;

        status_dispersion_param_obs_noise_gnssX(i,1) = state_dispersion_param_obs_noise_gnss_x;
        status_dispersion_param_obs_noise_gnssY(i,1) = state_dispersion_param_obs_noise_gnss_y;
        status_dispersion_param_obs_noise_gnssZ(i,1) = state_dispersion_param_obs_noise_gnss_z;
        status_variance_param_obs_noise_imuV(i,1) = state_dispersion_param_obs_noise_imu_v;
        status_movemean_param_obs_noise_gnssX(i,1) = state_movemean_param_obs_noise_gnss_x;
        status_movemean_param_obs_noise_gnssY(i,1) = state_movemean_param_obs_noise_gnss_y;
        status_movemean_param_obs_noise_gnssZ(i,1) = state_movemean_param_obs_noise_gnss_z;
        status_std2_param_obs_noise_gnss3d(i,1) = state_std2_param_obs_noise_gnss;


        % Q = ...
        %     [dispersion_param_obs_noise_gnss_x^2, 0, 0, 0, 0, 0
        %     0, dispersion_param_obs_noise_gnss_y^2, 0, 0, 0, 0
        %     0, 0, dispersion_param_obs_noise_gnss_z^2, 0, 0, 0
        %     0, 0, 0, dispersion_param_obs_noise_gnss_vx^2, 0, 0
        %     0, 0, 0, 0, dispersion_param_obs_noise_gnss_vy^2, 0
        %     0, 0, 0, 0, 0, dispersion_param_obs_noise_gnss_vz^2];


        R_DR_im = ...
            [state_dispersion_param_obs_noise_gnss_x^2, 0, 0, 0, 0, 0, 0
            0, state_dispersion_param_obs_noise_gnss_y^2, 0, 0, 0, 0, 0
            0, 0, state_dispersion_param_obs_noise_gnss_z^2, 0, 0, 0, 0
            0, 0, 0, param_obs_noise_im_roll^2, 0, 0, 0
            0, 0, 0, 0, param_obs_noise_im_pitch^2, 0, 0
            0, 0, 0, 0, 0, param_obs_noise_DR_yaw^2, 0
            0, 0, 0, 0, 0, 0, state_dispersion_param_obs_noise_imu_v^2];


        R_DR_re = ...
            [state_dispersion_param_obs_noise_gnss_x^2, 0, 0, 0, 0, 0, 0
            0, state_dispersion_param_obs_noise_gnss_y^2, 0, 0, 0, 0, 0
            0, 0, state_dispersion_param_obs_noise_gnss_z^2, 0, 0, 0, 0
            0, 0, 0, param_obs_noise_roll^2, 0, 0, 0
            0, 0, 0, 0, param_obs_noise_pitch^2, 0, 0
            0, 0, 0, 0, 0, param_obs_noise_DR_yaw^2, 0
            0, 0, 0, 0, 0, 0, state_dispersion_param_obs_noise_imu_v^2];

        R_GNSS_im = ...
            [state_dispersion_param_obs_noise_gnss_x^2, 0, 0, 0, 0, 0, 0
            0, state_dispersion_param_obs_noise_gnss_y^2, 0, 0, 0, 0, 0
            0, 0, state_dispersion_param_obs_noise_gnss_z^2, 0, 0, 0, 0
            0, 0, 0, param_obs_noise_im_roll^2, 0, 0, 0
            0, 0, 0, 0, param_obs_noise_im_pitch^2, 0, 0
            0, 0, 0, 0, 0, param_obs_noise_gnss_yaw^2, 0
            0, 0, 0, 0, 0, 0, state_dispersion_param_obs_noise_imu_v^2];

        R_GNSS_re = ...
            [state_dispersion_param_obs_noise_gnss_x/2, 0, 0, 0, 0, 0, 0
            0, state_dispersion_param_obs_noise_gnss_y/2, 0, 0, 0, 0, 0
            0, 0, state_dispersion_param_obs_noise_gnss_z/2, 0, 0, 0, 0
            0, 0, 0, param_obs_noise_roll^2, 0, 0, 0
            0, 0, 0, 0, param_obs_noise_pitch^2, 0, 0
            0, 0, 0, 0, 0, param_obs_noise_gnss_yaw^2, 0
            0, 0, 0, 0, 0, 0, state_dispersion_param_obs_noise_imu_v^2];



        %% Update　更新
        %----------------eagleye_log(i,4)---------------------
        % gnss_yaw(:,1) = eagleye.yaw(:);
        if option_Pitch_type==0 %ピッチ角を車両モデルから求める場合

            if flag_gnss(i) == 1&&flag_test3(i)==1 %GNSS　◯　　ロール、ピッチ　◯　
                KF_obs = [state_pos_Eagleye_enu(i,1), state_pos_Eagleye_enu(i,2), state_pos_Eagleye_enu(i,3), element_R_roll(i), element_P_pitch(i), state_gnss_yaw(i), data_velocity(i)]'; %観測方程式
                H = ...
                    [1, 0, 0, 0, 0, 0, 0
                    0, 1, 0, 0, 0, 0, 0
                    0, 0, 1, 0, 0, 0, 0
                    0, 0, 0, 1, 0, 0, 0
                    0, 0, 0, 0, 1, 0, 0
                    0, 0, 0, 0, 0, 1, 0
                    0, 0, 0, 0, 0, 0, 1];

                R = R_GNSS_re;
            end

            if flag_gnss(i) == 1&&flag_test3(i)==0 %GNSS　◯　　ロール、ピッチ　☓　
                KF_obs = [state_pos_Eagleye_enu(i,1), state_pos_Eagleye_enu(i,2), state_pos_Eagleye_enu(i,3), 0, 0, state_gnss_yaw(i), data_velocity(i)]'; %観測方程式
                H = ...
                    [1, 0, 0, 0, 0, 0, 0
                    0, 1, 0, 0, 0, 0, 0
                    0, 0, 1, 0, 0, 0, 0
                    0, 0, 0, 0, 0, 0, 0
                    0, 0, 0, 0, 0, 0, 0
                    0, 0, 0, 0, 0, 1, 0
                    0, 0, 0, 0, 0, 0, 1];

                R = R_GNSS_im;
            end

            if flag_gnss(i) == 0&&flag_test3(i)==1 %GNSS　☓　　ロール、ピッチ　◯
                KF_obs = [state_pos_Eagleye_enu(i,1), state_pos_Eagleye_enu(i,2), state_pos_Eagleye_enu(i,3),...
                    element_R_roll(i), element_P_pitch(i), state_gnss_yaw(i), status_velocity(i)]'; %観測方程式
                H = ...
                    [1, 0, 0, 0, 0, 0, 0
                    0, 1, 0, 0, 0, 0, 0
                    0, 0, 1, 0, 0, 0, 0
                    0, 0, 0, 1, 0, 0, 0
                    0, 0, 0, 0, 1, 0, 0
                    0, 0, 0, 0, 0, 0, 0
                    0, 0, 0, 0, 0, 0, 1];

                R = R_DR_re;
            end

            if flag_gnss(i) == 0&&flag_test3(i)==0 %GNSS　☓　　ロール、ピッチ　☓
                KF_obs = [state_pos_Eagleye_enu(i,1), state_pos_Eagleye_enu(i,2), state_pos_Eagleye_enu(i,3),...
                    0, 0, state_gnss_yaw(i), (data_velocity(i)+element_P_pitch(i))/2]'; %観測方程式

                H = ...
                    [1, 0, 0, 0, 0, 0, 0
                    0, 1, 0, 0, 0, 0, 0
                    0, 0, 1, 0, 0, 0, 0
                    0, 0, 0, 0, 0, 0, 0
                    0, 0, 0, 0, 0, 0, 0
                    0, 0, 0, 0, 0, 0, 0
                    0, 0, 0, 0, 0, 0, 1];
                R = R_DR_im;
            end

        end

        %%
        y = KF_obs - H*Pre_KF_status; %y(k)=z(k)-H(x(k|k-1))
        S = H*Pre_KF_covariance*H' + R; %S(k)=H*P(k|k-1)*H(k)^T+R
        K = Pre_KF_covariance*H'*inv(S); %K(k)=P(k|k-1)*H(k)^T*(S(k)^(-1))

        Update_KF_status = Pre_KF_status + K*y; %x(k|k)=x(k|k-1)+K(k)*y(k)
        Update_KF_covariance = (eye(7) - K*H)*Pre_KF_covariance; %P(k|k)=(I(単位行列)-K(k)*H(k))*P(k|k-1)

        %KF後の値を代入
        KF_x(i) = Update_KF_status(1);
        KF_y(i) = Update_KF_status(2);
        KF_z(i) = Update_KF_status(3);
        KF_roll(i) = Update_KF_status(4);
        KF_pitch(i) = Update_KF_status(5);
        KF_yaw(i) = Update_KF_status(6);
        KF_velocity(i) = Update_KF_status(7);
        KF_covariance(i,:) = setVecCovariance(Update_KF_covariance,7);
        P = Update_KF_covariance;

    end
    KF_enu = [KF_x(:,1),KF_y(:,1),KF_z(:,1)];
    analize_error_EKF(:,1:3) = KF_enu(:,1:3)  - data_pos_Ref_enu(:,1:3);
    analize_error_EKF3d(:,1) = sqrt(analize_error_EKF(:,1).^2+analize_error_EKF(:,2).^2+analize_error_EKF(:,3).^2);

    disp(num2str(mean(abs(analize_error_EKF3d))))

% end

%%
cd ..

%% Utilities/function
function outCov = setVecCovariance(inCov,Dim)
    outCov = zeros(Dim*Dim,1);
    for i = 1:Dim
        for j = 1:Dim
            outCov((i-1)*Dim+j,1) = inCov(i,j);
        end
    end
end

function outCov = setMatCovariance(inCov,Dim)
    outCov = zeros(Dim,Dim);
    for i = 1:Dim
        for j = 1:Dim
            outCov(i,j) = inCov(:,(i-1)*Dim+j);
        end
    end
end