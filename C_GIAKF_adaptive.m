disp('C_EKF Adaptive')
disp('C_EKF Adaptive with Curve Compensation')

state_pos_Eagleye_enu = data_correction_Eagleye_enu;

init_x = data_pos_Ref_enu(1,1);
init_y = data_pos_Ref_enu(1,2);
init_z = data_pos_Ref_enu(1,3);

%% 適応型フィルタ用の状態変数初期化
% 分散パラメータ格納用配列
status_variance_param_obs_noise_gnssX = zeros(time_SimEpoch,1);
status_variance_param_obs_noise_gnssY = zeros(time_SimEpoch,1);
status_variance_param_obs_noise_gnssZ = zeros(time_SimEpoch,1);
status_variance_param_obs_noise_imuV = zeros(time_SimEpoch,1);
status_movemean_param_obs_noise_gnssX = zeros(time_SimEpoch,1);
status_movemean_param_obs_noise_gnssY = zeros(time_SimEpoch,1);
status_movemean_param_obs_noise_gnssZ = zeros(time_SimEpoch,1);
status_kalmanGain = zeros(1,1,time_SimEpoch);

status_variance_param_obs_noise_gnssvX = zeros(time_SimEpoch,1);
status_variance_param_obs_noise_gnssvY = zeros(time_SimEpoch,1);
status_variance_param_obs_noise_gnssvZ = zeros(time_SimEpoch,1);

status_std2_param_obs_noise_gnss3d = zeros(time_SimEpoch,1);

% 誤差計算
status_VelocityResidual(:,1) = state_velocity(:,1) - data_velocity_REF(:,1);
status_PositionResidual(:,1:3) = state_pos_Eagleye_enu(:,1:3) - data_pos_Ref_enu(:,1:3);

%%
% シエンタ用車両パラメータの設定
vehicle_params = struct(...
    'wheelBase', 2.75, ...      % シエンタのホイールベース [m]
    'cog_height', 0.55, ...     % シエンタの重心高さ [m]
    'track_width', 1.49);       % シエンタのトレッド幅 [m]

% 状態検出パラメータの直接設定
state_detect_params = struct(...
    'curve_threshold', 0.1, ... % カーブ判定用ヨーレート閾値 [rad/s]
    'stop_threshold', 0.5);     % 停止判定用速度閾値 [m/s]

% カーブ補正モデルの初期化
curveErrorModel = CurveErrorCompensation();
curveErrorModel.wheelBase = vehicle_params.wheelBase;
curveErrorModel.cog_height = vehicle_params.cog_height;
curveErrorModel.track_width = vehicle_params.track_width;
curveErrorModel.slip_coefficient = 0.15; % タイヤスリップ係数
curveErrorModel.bank_angle_max = 0.05; % 最大路面バンク角[rad]

% 適応フィルタの初期化
adaptiveKF = ImprovedAdaptiveKF();
adaptiveKF.windowSize = 50; % 観測ウィンドウサイズ
adaptiveKF.minCovThreshold = 0.01; % 最小共分散閾値
adaptiveKF.curvingThreshold = state_detect_params.curve_threshold; % カーブ判定用ヨーレート閾値
adaptiveKF.stoppingThreshold = state_detect_params.stop_threshold; % 停止判定用速度閾値

% カーブ補正結果格納用配列初期化
KF_beta = zeros(time_SimEpoch, 1);
KF_compensated_vel = zeros(time_SimEpoch, 1);
isCurvingArray = false(time_SimEpoch, 1);
isStoppedArray = false(time_SimEpoch, 1);

% 疑似誤差格納用
pseudoErrorArray = zeros(time_SimEpoch, 1);

%% 時間差分と疑似観測誤差の計算
status_timedifference = zeros(time_SimEpoch-1,3);
status_pseudo_observation_error = zeros(time_SimEpoch-1,3);
for i = 2:time_SimEpoch
    status_timedifference(i,1:3) = state_pos_Eagleye_enu(i,1:3) - state_pos_Eagleye_enu(i-1,1:3);
end

% 重み設定
weight_Eagleye = 1;
weight_Doppler = 1; % fixの場合は2、floatの場合は1

% 疑似観測誤差の計算
for i = 2:time_SimEpoch
    status_pseudo_observation_error(i,1:3) = status_timedifference(i,1:3) -  ...
                                           (weight_Doppler * data_velocity(i-1,1:3) * const_dt_Gnss * const_dt_Gnss +  ...
                                            weight_Eagleye * state_velocity(i-1,1:3) * const_dt3(i-1)) /  ...
                                           (weight_Doppler + weight_Eagleye);
end
status_pseudo_observation_error(:,1:3) = abs(status_pseudo_observation_error(:,1:3));

%% 疑似観測誤差の変換（POE: Pseudo Observation Error）
% handle_POEの値に応じた変換方法
% switch handle_POE
%     case 0 % 変換なし
%         analize_POE = status_pseudo_observation_error;
%     case 1 % 閾値方式
%         analize_POE = zeros(length(status_pseudo_observation_error),3);
%         for i2 = 1:3
%             for i = 1:length(status_pseudo_observation_error)
%                 if status_pseudo_observation_error(i,i2) >= handle_POEnum
%                     analize_POE(i,i2) = status_pseudo_observation_error(i,i2);
%                 else
%                     analize_POE(i,i2) = handle_POEnum;
%                 end
%             end
%         end
%     case 2 % 1次関数変換
        analize_POE(:,1:3) = handle_POEnum_A .* status_pseudo_observation_error + handle_POEnum_B;
%     case 3 % 2次関数変換
%         analize_POE(:,1:3) = handle_POEnum_A .* status_pseudo_observation_error.^2;
%     case 4 % 指数関数変換
%         handle_POEnum_B = 1;
%         analize_POE(:,1:3) = handle_POEnum_B .* handle_POEnum_A .^ status_pseudo_observation_error;
% end

%% リファレンスとの差分計算
status_timedifference_ref = zeros(time_SimEpoch-1,3);
status_dist_refer_enu = zeros(time_SimEpoch-1,3);
status_dist_refer_3d = zeros(time_SimEpoch-1,3);
for i = 2:time_SimEpoch
    status_timedifference_ref(i,1:3) = data_pos_Ref_enu(i,1:3) - data_pos_Ref_enu(i-1,1:3);
    status_dist_refer_enu(i,1:3) = status_timedifference(i,1:3) - status_timedifference_ref(i,1:3);
    status_dist_refer_3d(i,1) = sqrt(status_dist_refer_enu(i,1)^2 +  ...
                                      status_dist_refer_enu(i,2)^2 + ...
                                      status_dist_refer_enu(i,3)^2);
end

% 3D距離計算
status_dist_refer_3d_loc = zeros(time_SimEpoch-1,1);
status_dist_refer_3d_ref = zeros(time_SimEpoch-1,1);
for i = 2:time_SimEpoch
    status_dist_refer_3d_loc(i,1) = sqrt(status_timedifference(i,1)^2 +  ...
                                          status_timedifference(i,2)^2 + ...
                                          status_timedifference(i,3)^2);
    status_dist_refer_3d_ref(i,1) = sqrt(status_timedifference_ref(i,1)^2 +  ...
                                          status_timedifference_ref(i,2)^2 + ...
                                          status_timedifference_ref(i,3)^2);
end

%% 位置残差の設定
status_PositionResidual(:,1:3) = analize_POE(:,1:3);

%% 拡張カルマンフィルタ実装
% ウィンドウサイズ設定
windowSize = 50;

for i = 1:time_SimEpoch
    if i == 1  % 初期値設定
        KF_x(i) = init_x;
        KF_y(i) = init_y;
        KF_z(i) = init_z;
        KF_roll(i) = init_roll;
        KF_pitch(i) = init_pitch;
        KF_yaw(i) = init_yaw;
        KF_velocity(i) = init_velocity;
        KF_covariance(i,:) = setVecCovariance(init_covariance,7);
        P = init_covariance;
        continue
    end

    %% 予測ステップ
    % 姿勢角の予測
    Pre_KF_roll = KF_roll(i-1) + (data_struct_Eagleye.Roll_rate(i-1,1) +  ...
                                 data_struct_Eagleye.Pitch_rate(i-1,1) * sin(KF_roll(i-1)) * tan(KF_pitch(i-1)) +  ...
                                 data_yawrate(i-1,1) * cos(KF_roll(i-1)) * tan(KF_pitch(i-1))) * const_dt3(i);
    Pre_KF_pitch = KF_pitch(i-1) + (data_struct_Eagleye.Pitch_rate(i-1,1) * cos(KF_roll(i-1)) -  ...
                                   data_yawrate(i-1,1) * sin(KF_roll(i-1))) * const_dt3(i);
    Pre_KF_yaw = KF_yaw(i-1) + data_yawrate(i-1,1) * const_dt3(i);
    Pre_KF_z = KF_z(i-1) + sin(KF_pitch(i-1)) * state_velocity(i) * const_dt3(i);

    % 位置の予測
    % 車両状態の検出
    adaptiveKF.updateVehicleState(data_yawrate(i-1), KF_velocity(i-1));
    isCurvingArray(i) = adaptiveKF.isCurving;
    isStoppedArray(i) = adaptiveKF.isStopped;

    % GNSS品質の設定
    if flag_gnss(i) == 1
        adaptiveKF.setGNSSQuality('FIX');
    else
        adaptiveKF.setGNSSQuality('FLOAT');
    end

    %状態予測（カーブ補正付き）
    if adaptiveKF.isCurving
        % カーブ時のスリップ角推定と速度補正
        [compensated_vel, beta] = curveErrorModel.compensateCurveErrors(KF_velocity(i-1), data_yawrate(i-1), element_ay_EAGLEYE(i-1));
        KF_beta(i) = beta;
        KF_compensated_vel(i) = compensated_vel;

        % カーブ補正付き位置予測
        [dx, dy] = curveErrorModel.predictCurvePosition(KF_velocity(i-1), KF_yaw(i-1), data_yawrate(i-1), const_dt3(i));
        Pre_KF_x = KF_x(i-1) + dx;
        Pre_KF_y = KF_y(i-1) + dy;
    else
        % 通常の予測
        KF_beta(i) = 0;
        KF_compensated_vel(i) = KF_velocity(i-1);

        Pre_KF_x = KF_x(i-1) - KF_velocity(i-1) * (data_yawrate(i-1,1)^(-1)) * ...
            (-cos(KF_yaw(i-1,1) + data_yawrate(i-1,1) * const_dt3(i)) + cos(KF_yaw(i-1,1)));
        Pre_KF_y = KF_y(i-1) - KF_velocity(i-1) * (data_yawrate(i-1,1)^(-1)) * ...
            (sin(KF_yaw(i-1,1) + data_yawrate(i-1,1) * const_dt3(i)) - sin(KF_yaw(i-1,1)));
    end
    % if option_calc_type == 0
    %     Pre_KF_x = KF_x(i-1) - KF_velocity(i-1) * (data_yawrate(i-1,1)^(-1)) *  ...
    %                            (-cos(KF_yaw(i-1,1) + data_yawrate(i-1,1) * const_dt3(i)) + cos(KF_yaw(i-1,1)));
    %     Pre_KF_y = KF_y(i-1) - KF_velocity(i-1) * (data_yawrate(i-1,1)^(-1)) *  ...
    %                            (sin(KF_yaw(i-1,1) + data_yawrate(i-1,1) * const_dt3(i)) - sin(KF_yaw(i-1,1)));
    % end
    
    KF_vec_x(i-1,1) = -KF_velocity(i-1) * (data_yawrate(i-1,1)^(-1)) *  ...
                      (-cos(KF_yaw(i-1,1) + data_yawrate(i-1,1) * const_dt3(i)) + cos(KF_yaw(i-1,1)));
    KF_vec_y(i-1,1) = -KF_velocity(i-1) * (data_yawrate(i-1,1)^(-1)) *  ...
                      (sin(KF_yaw(i-1,1) + data_yawrate(i-1,1) * const_dt3(i)) - sin(KF_yaw(i-1,1)));

    % 速度の予測
    Pre_KF_velocity(1,1) = KF_velocity(i-1) + data_acc_Eagleye_enu3d(i-1,1) * const_dt3(i);

    %% ヤコビアン行列の計算
    jF = [
        1, 0, 0, 0, 0, state_velocity(i-1) * (data_yawrate(i-1,1)^(-1)) * (sin(KF_yaw(i-1,1) + data_yawrate(i-1,1) * const_dt3(i)) - sin(KF_yaw(i-1,1))), 0;
        0, 1, 0, 0, 0, state_velocity(i-1) * (data_yawrate(i-1,1)^(-1)) * (cos(KF_yaw(i-1,1) + data_yawrate(i-1,1) * const_dt3(i)) - cos(KF_yaw(i-1,1))), 0;
        0, 0, 1, 0, 0, 0, 0;
        0, 0, 0, 1 + (data_struct_Eagleye.Pitch_rate(i-1,1) * cos(KF_roll(i-1)) * tan(KF_pitch(i-1)) - data_yawrate(i-1,1) * sin(KF_roll(i-1)) * tan(KF_pitch(i-1))) * const_dt3(i), (data_struct_Eagleye.Pitch_rate(i-1,1) * sin(KF_roll(i-1)) / (cos(KF_pitch(i-1)) * cos(KF_pitch(i-1))) + data_yawrate(i-1,1) * cos(KF_roll(i-1)) / (cos(KF_pitch(i-1)) * cos(KF_pitch(i-1)))) * const_dt3(i), 0, 0;
        0, 0, 0, (-data_struct_Eagleye.Pitch_rate(i-1,1) * sin(KF_roll(i-1)) - data_yawrate(i-1,1) * cos(KF_roll(i-1))) * const_dt3(i), 1, 0, 0;
        0, 0, 0, 0, 0, 1, 0;
        0, 0, 0, 0, 0, 0, 1
    ];

    % 予測値を行列に設定
    Pre_KF_status = [Pre_KF_x, Pre_KF_y, Pre_KF_z, Pre_KF_roll, Pre_KF_pitch, Pre_KF_yaw, Pre_KF_velocity]';
    
    % 予測共分散の計算
    Pre_KF_covariance = jF * P * jF' + Q;

    %% 観測値の方位角調整
    handle_Heading_GNSS_control(i,1) = Pre_KF_yaw - handle_Heading_GNSS(i,1);
    handle_Heading_GNSS(i,1) = handle_Heading_GNSS(i,1);
    
    % 角度補正（2πの倍数単位）
    if handle_Heading_GNSS_control(i,1) < -pi
        handle_Heading_GNSS(i,1) = handle_Heading_GNSS(i,1) - 2*pi;
    end
    if handle_Heading_GNSS_control(i,1) < -3*pi
        handle_Heading_GNSS(i,1) = handle_Heading_GNSS(i,1) - 4*pi;
    end
    if handle_Heading_GNSS_control(i,1) < -5*pi
        handle_Heading_GNSS(i,1) = handle_Heading_GNSS(i,1) - 6*pi;
    end
    if handle_Heading_GNSS_control(i,1) < -7*pi
        handle_Heading_GNSS(i,1) = handle_Heading_GNSS(i,1) - 8*pi;
    end
    if handle_Heading_GNSS_control(i,1) < -9*pi
        handle_Heading_GNSS(i,1) = handle_Heading_GNSS(i,1) - 10*pi;
    end
    if handle_Heading_GNSS_control(i,1) > pi
        handle_Heading_GNSS(i,1) = handle_Heading_GNSS(i,1) + 2*pi;
    end
    if handle_Heading_GNSS_control(i,1) > 3*pi
        handle_Heading_GNSS(i,1) = handle_Heading_GNSS(i,1) + 4*pi;
    end
    if handle_Heading_GNSS_control(i,1) > 5*pi
        handle_Heading_GNSS(i,1) = handle_Heading_GNSS(i,1) + 6*pi;
    end
    if handle_Heading_GNSS_control(i,1) > 7*pi
        handle_Heading_GNSS(i,1) = handle_Heading_GNSS(i,1) + 8*pi;
    end
    if handle_Heading_GNSS_control(i,1) > 9*pi
        handle_Heading_GNSS(i,1) = handle_Heading_GNSS(i,1) + 10*pi;
    end
    state_gnss_yaw = handle_Heading_GNSS;

    %% 適応フィルタのためのパラメータ計算
    % ウィンドウ設定
    window_center = floor(windowSize/2) + mod(windowSize,2);
    win = windowSize - window_center;
    state_PR3d = sqrt(status_PositionResidual(:,1).^2 +  ...
                     status_PositionResidual(:,2).^2 +  ...
                     status_PositionResidual(:,3).^2);

    % ウィンドウサイズに応じた分散計算
    switch mod(windowSize,2)
        case 1  % 奇数サイズの場合
            if i < window_center
                state_dispersion_param_obs_noise_gnss_x = var(status_PositionResidual(1:i+win,1));
                state_dispersion_param_obs_noise_gnss_y = var(status_PositionResidual(1:i+win,2));
                state_dispersion_param_obs_noise_gnss_z = var(status_PositionResidual(1:i+win,3));
                state_dispersion_param_obs_noise_imu_v = var(status_VelocityResidual(1:i+win,1));
                state_movemean_param_obs_noise_gnss_x = mean(status_PositionResidual(1:i+win,1));
                state_movemean_param_obs_noise_gnss_y = mean(status_PositionResidual(1:i+win,2));
                state_movemean_param_obs_noise_gnss_z = mean(status_PositionResidual(1:i+win,3));
                state_std2_param_obs_noise_gnss = std2(state_PR3d(1:i+win,1));
            elseif i > time_SimEpoch-win
                state_dispersion_param_obs_noise_gnss_x = var(status_PositionResidual(i-win:end,1));
                state_dispersion_param_obs_noise_gnss_y = var(status_PositionResidual(i-win:end,2));
                state_dispersion_param_obs_noise_gnss_z = var(status_PositionResidual(i-win:end,3));
                state_dispersion_param_obs_noise_imu_v = var(status_VelocityResidual(i-win:end,1));
                state_movemean_param_obs_noise_gnss_x = mean(status_PositionResidual(i-win:end,1));
                state_movemean_param_obs_noise_gnss_y = mean(status_PositionResidual(i-win:end,2));
                state_movemean_param_obs_noise_gnss_z = mean(status_PositionResidual(i-win:end,3));
                state_std2_param_obs_noise_gnss = std2(state_PR3d(i-win:end,1));
            else
                state_dispersion_param_obs_noise_gnss_x = var(status_PositionResidual(i-win:i+win,1));
                state_dispersion_param_obs_noise_gnss_y = var(status_PositionResidual(i-win:i+win,2));
                state_dispersion_param_obs_noise_gnss_z = var(status_PositionResidual(i-win:i+win,3));
                state_dispersion_param_obs_noise_imu_v = var(status_VelocityResidual(i-win:i+win,1));
                state_movemean_param_obs_noise_gnss_x = mean(status_PositionResidual(i-win:i+win,1));
                state_movemean_param_obs_noise_gnss_y = mean(status_PositionResidual(i-win:i+win,2));
                state_movemean_param_obs_noise_gnss_z = mean(status_PositionResidual(i-win:i+win,3));
                state_std2_param_obs_noise_gnss = std2(state_PR3d(i-win:i+win,1));
            end
        case 0  % 偶数サイズの場合
            if i < window_center
                state_dispersion_param_obs_noise_gnss_x = var(status_PositionResidual(1:i+win,1));
                state_dispersion_param_obs_noise_gnss_y = var(status_PositionResidual(1:i+win,2));
                state_dispersion_param_obs_noise_gnss_z = var(status_PositionResidual(1:i+win,3));
                state_dispersion_param_obs_noise_imu_v = var(status_VelocityResidual(1:i+win,1));
                state_movemean_param_obs_noise_gnss_x = mean(status_PositionResidual(1:i+win,1));
                state_movemean_param_obs_noise_gnss_y = mean(status_PositionResidual(1:i+win,2));
                state_movemean_param_obs_noise_gnss_z = mean(status_PositionResidual(1:i+win,3));
                state_std2_param_obs_noise_gnss = std2(state_PR3d(1:i+win,1));
            elseif i > time_SimEpoch-win
                state_dispersion_param_obs_noise_gnss_x = var(status_PositionResidual(i-(win-1):end,1));
                state_dispersion_param_obs_noise_gnss_y = var(status_PositionResidual(i-(win-1):end,2));
                state_dispersion_param_obs_noise_gnss_z = var(status_PositionResidual(i-(win-1):end,3));
                state_dispersion_param_obs_noise_imu_v = var(status_VelocityResidual(i-(win-1):end,1));
                state_movemean_param_obs_noise_gnss_x = mean(status_PositionResidual(i-(win-1):end,1));
                state_movemean_param_obs_noise_gnss_y = mean(status_PositionResidual(i-(win-1):end,2));
                state_movemean_param_obs_noise_gnss_z = mean(status_PositionResidual(i-(win-1):end,3));
                state_std2_param_obs_noise_gnss = std2(state_PR3d(i-(win-1):end,1));
            else
                state_dispersion_param_obs_noise_gnss_x = var(status_PositionResidual(i-(win-1):i+win,1));
                state_dispersion_param_obs_noise_gnss_y = var(status_PositionResidual(i-(win-1):i+win,2));
                state_dispersion_param_obs_noise_gnss_z = var(status_PositionResidual(i-(win-1):i+win,3));
                state_dispersion_param_obs_noise_imu_v = var(status_VelocityResidual(i-(win-1):i+win,1));
                state_movemean_param_obs_noise_gnss_x = mean(status_PositionResidual(i-(win-1):i+win,1));
                state_movemean_param_obs_noise_gnss_y = mean(status_PositionResidual(i-(win-1):i+win,2));
                state_movemean_param_obs_noise_gnss_z = mean(status_PositionResidual(i-(win-1):i+win,3));
                state_std2_param_obs_noise_gnss = std2(state_PR3d(i-(win-1):i+win,1));
            end
    end

    % 計算された分散パラメータを記録
    status_dispersion_param_obs_noise_gnssX(i,1) = state_dispersion_param_obs_noise_gnss_x;
    status_dispersion_param_obs_noise_gnssY(i,1) = state_dispersion_param_obs_noise_gnss_y;
    status_dispersion_param_obs_noise_gnssZ(i,1) = state_dispersion_param_obs_noise_gnss_z;
    status_variance_param_obs_noise_imuV(i,1) = state_dispersion_param_obs_noise_imu_v;
    status_movemean_param_obs_noise_gnssX(i,1) = state_movemean_param_obs_noise_gnss_x;
    status_movemean_param_obs_noise_gnssY(i,1) = state_movemean_param_obs_noise_gnss_y;
    status_movemean_param_obs_noise_gnssZ(i,1) = state_movemean_param_obs_noise_gnss_z;
    status_std2_param_obs_noise_gnss3d(i,1) = state_std2_param_obs_noise_gnss;

    % 疑似観測誤差を適応フィルタに渡す
    if i > 2
        % IMUとGNSS速度の構造体作成
        velocity_struct = struct('imu', state_velocity(i-1,:), 'gnss', data_velocity(i-1,:));

        % 疑似誤差計算
        pseudoError = adaptiveKF.generatePseudoError(state_pos_Eagleye_enu(i,:), state_pos_Eagleye_enu(i-1,:), velocity_struct, const_dt3(i-1));
        pseudoErrorArray(i) = pseudoError;

        % カーブ時は観測ノイズを増加
        if adaptiveKF.isCurving
            % 観測共分散を更新
            curve_R_scale = adaptiveKF.updateObservationCovariance(pseudoErrorArray(1:i));

            % R行列のスケーリング
            % 注: ここでは各要素に直接適用しない方針なので、必要に応じて変更
            status_dispersion_param_obs_noise_gnssX(i) = status_dispersion_param_obs_noise_gnssX(i) * (1 + curve_R_scale);
            status_dispersion_param_obs_noise_gnssY(i) = status_dispersion_param_obs_noise_gnssY(i) * (1 + curve_R_scale);
        end
    end

    % 適応型の観測ノイズ共分散行列計算
    R_DR_im = [
        state_dispersion_param_obs_noise_gnss_x^2, 0, 0, 0, 0, 0, 0;
        0, state_dispersion_param_obs_noise_gnss_y^2, 0, 0, 0, 0, 0;
        0, 0, state_dispersion_param_obs_noise_gnss_z^2, 0, 0, 0, 0;
        0, 0, 0, param_obs_noise_im_roll^2, 0, 0, 0;
        0, 0, 0, 0, param_obs_noise_im_pitch^2, 0, 0;
        0, 0, 0, 0, 0, param_obs_noise_DR_yaw^2, 0;
        0, 0, 0, 0, 0, 0, state_dispersion_param_obs_noise_imu_v^2
    ];

    R_DR_re = [
        state_dispersion_param_obs_noise_gnss_x^2, 0, 0, 0, 0, 0, 0;
        0, state_dispersion_param_obs_noise_gnss_y^2, 0, 0, 0, 0, 0;
        0, 0, state_dispersion_param_obs_noise_gnss_z^2, 0, 0, 0, 0;
        0, 0, 0, param_obs_noise_roll^2, 0, 0, 0;
        0, 0, 0, 0, param_obs_noise_pitch^2, 0, 0;
        0, 0, 0, 0, 0, param_obs_noise_DR_yaw^2, 0;
        0, 0, 0, 0, 0, 0, state_dispersion_param_obs_noise_imu_v^2
    ];

    R_GNSS_im = [
        state_dispersion_param_obs_noise_gnss_x^2, 0, 0, 0, 0, 0, 0;
        0, state_dispersion_param_obs_noise_gnss_y^2, 0, 0, 0, 0, 0;
        0, 0, state_dispersion_param_obs_noise_gnss_z^2, 0, 0, 0, 0;
        0, 0, 0, param_obs_noise_im_roll^2, 0, 0, 0;
        0, 0, 0, 0, param_obs_noise_im_pitch^2, 0, 0;
        0, 0, 0, 0, 0, param_obs_noise_gnss_yaw^2, 0;
        0, 0, 0, 0, 0, 0, state_dispersion_param_obs_noise_imu_v^2
    ];

    R_GNSS_re = [
        state_dispersion_param_obs_noise_gnss_x/2, 0, 0, 0, 0, 0, 0;
        0, state_dispersion_param_obs_noise_gnss_y/2, 0, 0, 0, 0, 0;
        0, 0, state_dispersion_param_obs_noise_gnss_z/2, 0, 0, 0, 0;
        0, 0, 0, param_obs_noise_roll^2, 0, 0, 0;
        0, 0, 0, 0, param_obs_noise_pitch^2, 0, 0;
        0, 0, 0, 0, 0, param_obs_noise_gnss_yaw^2, 0;
        0, 0, 0, 0, 0, 0, state_dispersion_param_obs_noise_imu_v^2
    ];

    %% 更新ステップ
    % フラグに応じた観測値と観測行列の設定
    if option_Pitch_type == 0  % ピッチ角を車両モデルから求める場合
        % GNSS使用可能かつロール・ピッチ角が実数のとき
        if flag_gnss(i) == 1 && flag_test3(i) == 1
            KF_obs = [state_pos_Eagleye_enu(i,1), state_pos_Eagleye_enu(i,2), ...
                      state_pos_Eagleye_enu(i,3), element_R_roll(i), element_P_pitch(i), ...
                      state_gnss_yaw(i), data_velocity(i)]';
            H = [
                1, 0, 0, 0, 0, 0, 0;
                0, 1, 0, 0, 0, 0, 0;
                0, 0, 1, 0, 0, 0, 0;
                0, 0, 0, 1, 0, 0, 0;
                0, 0, 0, 0, 1, 0, 0;
                0, 0, 0, 0, 0, 1, 0;
                0, 0, 0, 0, 0, 0, 1
            ];
            R = R_GNSS_re;
        end

        % GNSS使用可能だがロール・ピッチ角が虚数のとき
        if flag_gnss(i) == 1 && flag_test3(i) == 0
            KF_obs = [state_pos_Eagleye_enu(i,1), state_pos_Eagleye_enu(i,2), ...
                      state_pos_Eagleye_enu(i,3), 0, 0, ...
                      state_gnss_yaw(i), data_velocity(i)]';
            H = [
                1, 0, 0, 0, 0, 0, 0;
                0, 1, 0, 0, 0, 0, 0;
                0, 0, 1, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 1, 0;
                0, 0, 0, 0, 0, 0, 1
            ];
            R = R_GNSS_im;
        end

        % GNSS使用不可だがロール・ピッチ角が実数のとき
        if flag_gnss(i) == 0 && flag_test3(i) == 1
            KF_obs = [state_pos_Eagleye_enu(i,1), state_pos_Eagleye_enu(i,2), ...
                      state_pos_Eagleye_enu(i,3), element_R_roll(i), element_P_pitch(i), ...
                      state_gnss_yaw(i), data_velocity(i)]';
            H = [
                1, 0, 0, 0, 0, 0, 0;
                0, 1, 0, 0, 0, 0, 0;
                0, 0, 1, 0, 0, 0, 0;
                0, 0, 0, 1, 0, 0, 0;
                0, 0, 0, 0, 1, 0, 0;
                0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 1
            ];
            R = R_DR_re;
        end

        % GNSS使用不可でロール・ピッチ角も虚数のとき
        if flag_gnss(i) == 0 && flag_test3(i) == 0
            KF_obs = [state_pos_Eagleye_enu(i,1), state_pos_Eagleye_enu(i,2), ...
                      state_pos_Eagleye_enu(i,3), 0, 0, ...
                      state_gnss_yaw(i), (data_velocity(i) + element_P_pitch(i)) / 2]';
            H = [
                1, 0, 0, 0, 0, 0, 0;
                0, 1, 0, 0, 0, 0, 0;
                0, 0, 1, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 0, 1
            ];
            R = R_DR_im;
        end
    end

    %% カルマンゲイン計算と状態更新
    y = KF_obs - H * Pre_KF_status;
    S = H * Pre_KF_covariance * H' + R;
    K = Pre_KF_covariance * H' * inv(S);

    % 状態と共分散の更新
    Update_KF_status = Pre_KF_status + K * y;
    Update_KF_covariance = (eye(7) - K * H) * Pre_KF_covariance;

    % 更新値の保存
    KF_x(i) = Update_KF_status(1);
    KF_y(i) = Update_KF_status(2);
    KF_z(i) = Update_KF_status(3);
    KF_roll(i) = Update_KF_status(4);
    KF_pitch(i) = Update_KF_status(5);
    KF_yaw(i) = Update_KF_status(6);
    KF_velocity(i) = Update_KF_status(7);
    KF_covariance(i,:) = setVecCovariance(Update_KF_covariance, 7);
    P = Update_KF_covariance;
end

% 結果の計算
KF_enu = [KF_x(:,1), KF_y(:,1), KF_z(:,1)];
analize_error_EKF(:,1:3) = KF_enu(:,1:3) - data_pos_Ref_enu(:,1:3);
analize_error_EKF3d(:,1) = sqrt(analize_error_EKF(:,1).^2 +  ...
                               analize_error_EKF(:,2).^2 +  ...
                               analize_error_EKF(:,3).^2);

disp(['平均誤差: ', num2str(mean(abs(analize_error_EKF3d))), ' m'])


% カーブ補正効果の評価
curve_segments = find(isCurvingArray == 1);
straight_segments = find(isCurvingArray == 0 & isStoppedArray == 0);
stopped_segments = find(isStoppedArray == 1);

disp(['カーブ区間誤差: ', num2str(mean(abs(analize_error_EKF3d(curve_segments)))), ' m'])
disp(['直線区間誤差: ', num2str(mean(abs(analize_error_EKF3d(straight_segments)))), ' m'])
disp(['停止区間誤差: ', num2str(mean(abs(analize_error_EKF3d(stopped_segments)))), ' m'])

% 走行状態に応じた評価
if ~isempty(curve_segments)
    figure;
    subplot(2,1,1);
    plot(time_Eagleye, data_yawrate, 'b');
    hold on;
    plot(time_Eagleye(curve_segments), data_yawrate(curve_segments), 'r.');
    title('ヨーレートとカーブ検出');
    xlabel('時間 [s]');
    ylabel('ヨーレート [rad/s]');
    legend('ヨーレート', 'カーブ区間');

    subplot(2,1,2);
    plot(time_Eagleye, analize_error_EKF3d, 'b');
    hold on;
    plot(time_Eagleye(curve_segments), analize_error_EKF3d(curve_segments), 'r.');
    title('位置誤差とカーブ検出');
    xlabel('時間 [s]');
    ylabel('位置誤差 [m]');
    legend('位置誤差', 'カーブ区間');
end


cd ..

%% ユーティリティ関数
function outCov = setVecCovariance(inCov, Dim)
    % 行列を一次元配列に変換
    outCov = zeros(Dim*Dim, 1);
    for i = 1:Dim
        for j = 1:Dim
            outCov((i-1)*Dim+j, 1) = inCov(i, j);
        end
    end
end

function outCov = setMatCovariance(inCov, Dim)
    % 一次元配列を行列に変換
    outCov = zeros(Dim, Dim);
    for i = 1:Dim
        for j = 1:Dim
            outCov(i, j) = inCov(:, (i-1)*Dim+j);
        end
    end
end