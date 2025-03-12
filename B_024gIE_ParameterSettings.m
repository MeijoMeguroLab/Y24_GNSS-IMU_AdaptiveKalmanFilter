% cd B.ParameterSettings
disp('B.ParameterSettings')

%%
% ノイズ
%プロセスノイズ
param_pros_noise_x         = 0.3; %プロセスノイズ　x座標
param_pros_noise_y         = 0.3;%プロセスノイズ　y座標
param_pros_noise_z         = 0.3;%プロセスノイズ　z座標
param_pros_noise_v_x       = 0.7;%プロセスノイズ　x方向速度
param_pros_noise_v_y       = 0.7;%プロセスノイズ　y方向速度
param_pros_noise_v_z       = 0.7;%プロセスノイズ　z方向速度
param_pros_noise_roll      = 2.0;%プロセスノイズ　ロール角
param_pros_noise_pitch     = 0.25;%プロセスノイズ　ピッチ角
param_pros_noise_yaw       = 0.0009; %プロセスノイズ　方位角
param_pros_noise_rollrate  = 0.01;%プロセスノイズ　ロールレート
param_pros_noise_pitchrate = 0.1;%プロセスノイズ　ピッチレート
param_pros_noise_yawrate   = 0.1; %プロセスノイズ　ヨーレート
param_pros_noise_velocity  = 0.7; %プロセスノイズ　速度

%観測ノイズ
param_obs_noise_roll      = 90.0; %観測ノイズ　ロール角(推定値が実数)
param_obs_noise_pitch     =100.0; %観測ノイズ　ピッチ角(推定値が実数)
% param_obs_noise_yaw      = 1.0;%観測ノイズ　マルチアンテナを使う場合
param_obs_noise_rollrate  = 0.01; %観測ノイズ　ロールレート
param_obs_noise_pitchrate = 0.01; %観測ノイズ　ピッチレート
param_obs_noise_yawrate   = 0.0001; %観測ノイズ　ヨーレート
param_obs_noise_velocity  = 2; %観測ノイズ　速度

param_obs_noise_gnss_x = 0.01; %GNSS時の観測ノイズ　GNSSのx座標
param_obs_noise_gnss_y = 0.01; %GNSS時の観測ノイズ　GNSSのy座標
param_obs_noise_gnss_z = 0.81; %GNSS時の観測ノイズ　GNSSのz座標
param_obs_noise_gnss_yaw      = 100/180*pi; %GNSS時の観測ノイズ GNSSのヨー角
param_obs_noise_gnss_pitch    = 1/180*pi;%GNSS時の観測ノイズ GNSSのピッチ角
param_obs_noise_gnss_velocity = 0.03;

param_obs_noise_DR_x = param_pros_noise_x*1; %DR時の観測ノイズ　x座標
param_obs_noise_DR_y = param_pros_noise_y*1; %DR時の観測ノイズ　y座標
param_obs_noise_DR_z = param_pros_noise_z*1; %DR時の観測ノイズ　z座標
param_obs_noise_DR_yaw      = param_pros_noise_yaw*1; %DR観測ノイズ　ヨー角
param_obs_noise_DR_pitch    = param_pros_noise_pitch*1; %DR観測ノイズ　ヨー角
param_obs_noise_DR_velocity = param_obs_noise_velocity*1;

param_obs_noise_im_roll  = param_pros_noise_roll*1;%観測ノイズ　ロール角(推定値が虚数)
param_obs_noise_im_pitch = param_pros_noise_pitch*1;%観測ノイズ　ピッチ角(推定値が虚数)

handle_doppler_number = 10;%ドップラー速度の信頼性判定に使用　車輪速との差が？[m/s]以上のデータを排除


init_roll  = element_R_roll(1,1);%ロール角
init_pitch = element_P_pitch(1,1);%ピッチ角
init_yaw   = element_Y_yaw(1,1);%ヨー角
init_rollrate  = data_struct_Eagleye.Roll_rate(1,1);%ロールレート
init_pitchrate = data_struct_Eagleye.Pitch_rate(1,1);%ピッチレート
init_yawrate   = data_yawrate(1,1);%ヨーレート
% velocity = sqrt(Vx3.^2+Vy3.^2+Vz3.^2);
init_velocity = data_velocity(1,1);
% 観測値

KF_x = zeros(time_SimEpoch,1);
KF_y = zeros(time_SimEpoch,1);
KF_z = zeros(time_SimEpoch,1);
KF_roll  = zeros(time_SimEpoch,1);
KF_pitch = zeros(time_SimEpoch,1);
KF_yaw   = zeros(time_SimEpoch,1);
KF_velocity   = zeros(time_SimEpoch,1);
KF_covariance = zeros(time_SimEpoch,49);

% yawrate(i,1) = eagleye.Yaw_rate(i,1);

%プロセスノイズ
Q = ...
    [param_pros_noise_x^2, 0, 0, 0, 0, 0, 0
    0, param_pros_noise_y^2, 0, 0, 0, 0, 0
    0, 0, param_pros_noise_z^2, 0, 0, 0, 0
    0, 0, 0, param_pros_noise_roll^2, 0, 0, 0
    0, 0, 0, 0, param_pros_noise_pitch^2, 0, 0
    0, 0, 0, 0, 0, param_pros_noise_yaw^2, 0
    0, 0, 0, 0, 0, 0, param_pros_noise_velocity^2];



% option_Pitch_type = 0;

if option_Pitch_type == 0 %ピッチ角の推定を加速度と角速度から求める場合

    % 観測ノイズ(GNSSが使えない&観測値のロール角、ピッチ角が虚数)
    R_DR_im = ...
        [param_obs_noise_DR_x^2, 0, 0, 0, 0, 0, 0
        0, param_obs_noise_DR_y^2, 0, 0, 0, 0, 0
        0, 0, param_obs_noise_DR_z^2, 0, 0, 0, 0
        0, 0, 0, param_obs_noise_im_roll^2, 0, 0, 0
        0, 0, 0, 0, param_obs_noise_im_pitch^2, 0, 0
        0, 0, 0, 0, 0, param_obs_noise_DR_yaw^2, 0
        0, 0, 0, 0, 0, 0, param_obs_noise_DR_velocity^2];


    %観測ノイズ(GNSSが使えない&観測値のロール角、ピッチ角が実数)
    R_DR_re = ...
        [param_obs_noise_DR_x^2, 0, 0, 0, 0, 0, 0
        0, param_obs_noise_DR_y^2, 0, 0, 0, 0, 0
        0, 0, param_obs_noise_DR_z^2, 0, 0, 0, 0
        0, 0, 0, param_obs_noise_roll^2, 0, 0, 0
        0, 0, 0, 0, param_obs_noise_pitch^2, 0, 0
        0, 0, 0, 0, 0, param_obs_noise_DR_yaw^2, 0
        0, 0, 0, 0, 0, 0, param_obs_noise_DR_velocity^2];

    % 観測ノイズ(FIX解&観測値のロール角、ピッチ角が虚数)
    R_GNSS_im = ...
        [param_obs_noise_gnss_x^2, 0, 0, 0, 0, 0, 0
        0, param_obs_noise_gnss_y^2, 0, 0, 0, 0, 0
        0, 0, param_obs_noise_gnss_z^2, 0, 0, 0, 0
        0, 0, 0, param_obs_noise_im_roll^2, 0, 0, 0
        0, 0, 0, 0, param_obs_noise_im_pitch^2, 0, 0
        0, 0, 0, 0, 0, param_obs_noise_gnss_yaw^2, 0
        0, 0, 0, 0, 0, 0, param_obs_noise_gnss_velocity^2];

    % 観測ノイズ(FIX解&観測値のロール角、ピッチ角が実数)
    R_GNSS_re = ...
        [param_obs_noise_gnss_x^2, 0, 0, 0, 0, 0, 0
        0, param_obs_noise_gnss_y^2, 0, 0, 0, 0, 0
        0, 0, param_obs_noise_gnss_z^2, 0, 0, 0, 0
        0, 0, 0, param_obs_noise_roll^2, 0, 0, 0
        0, 0, 0, 0, param_obs_noise_pitch^2, 0, 0
        0, 0, 0, 0, 0, param_obs_noise_gnss_yaw^2, 0
        0, 0, 0, 0, 0, 0, param_obs_noise_gnss_velocity^2];

end

if option_Pitch_type == 1 %ピッチ角をGNSSのドップラー速度から求める場合

    % 観測ノイズ(GNSSが使えない&観測値のロール角が虚数)
    R_DR_im = ...
        [param_obs_noise_DR_x^2, 0, 0, 0, 0, 0, 0
        0, param_obs_noise_DR_y^2, 0, 0, 0, 0, 0
        0, 0, param_obs_noise_DR_z^2, 0, 0, 0, 0
        0, 0, 0, param_obs_noise_im_roll^2, 0, 0, 0
        0, 0, 0, 0, param_obs_noise_DR_pitch^2, 0, 0
        0, 0, 0, 0, 0, param_obs_noise_DR_yaw^2, 0
        0, 0, 0, 0, 0, 0, param_obs_noise_DR_velocity^2];


    %観測ノイズ(GNSSが使えない&観測値のロール角が実数)
    R_DR_re = ...
        [param_obs_noise_DR_x^2, 0, 0, 0, 0, 0, 0
        0, param_obs_noise_DR_y^2, 0, 0, 0, 0, 0
        0, 0, param_obs_noise_DR_z^2, 0, 0, 0, 0
        0, 0, 0, param_obs_noise_roll^2, 0, 0, 0
        0, 0, 0, 0, param_obs_noise_DR_pitch^2, 0, 0
        0, 0, 0, 0, 0, param_obs_noise_DR_yaw^2, 0
        0, 0, 0, 0, 0, 0, param_obs_noise_DR_velocity^2];

    % 観測ノイズ(FIX解&観測値のロール角が虚数)
    R_GNSS_im = ...
        [param_obs_noise_gnss_x^2, 0, 0, 0, 0, 0, 0
        0, param_obs_noise_gnss_y^2, 0, 0, 0, 0, 0
        0, 0, param_obs_noise_gnss_z^2, 0, 0, 0, 0
        0, 0, 0, param_obs_noise_im_roll^2, 0, 0, 0
        0, 0, 0, 0, param_obs_noise_gnss_pitch^2, 0, 0
        0, 0, 0, 0, 0, param_obs_noise_gnss_yaw^2, 0
        0, 0, 0, 0, 0, 0, param_obs_noise_gnss_velocity^2];

    % 観測ノイズ(FIX解&観測値のロール角、ピッチ角が実数)
    R_GNSS_re = ...
        [param_obs_noise_gnss_x^2, 0, 0, 0, 0, 0, 0
        0, param_obs_noise_gnss_y^2, 0, 0, 0, 0, 0
        0, 0, param_obs_noise_gnss_z^2, 0, 0, 0, 0
        0, 0, 0, param_obs_noise_roll^2, 0, 0, 0
        0, 0, 0, 0, param_obs_noise_gnss_pitch^2, 0, 0
        0, 0, 0, 0, 0, param_obs_noise_gnss_yaw^2, 0
        0, 0, 0, 0, 0, 0, param_obs_noise_gnss_velocity^2];

end

% 共分散の初期値
init_covariance = ...
    [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0
    0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0
    0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0
    0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001];

%%
cd ..