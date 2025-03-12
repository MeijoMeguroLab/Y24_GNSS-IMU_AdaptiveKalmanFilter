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
status_pseudo_observation_error = zeros(time_SimEpoch-1,3);
for i = 2:time_SimEpoch
    status_timedifference(i,1:3) = state_pos_Eagleye_enu(i,1:3) - state_pos_Eagleye_enu(i-1,1:3);
end

weight_Eagleye = 1;
weight_Doppler = 1; % fix:2 float:1 tosuru yotei

for i = 2:time_SimEpoch
    status_pseudo_observation_error(i,1:3) = status_timedifference(i,1:3)...
        - (weight_Doppler*data_velocity(i-1,1:3).*const_dt_Gnss.*const_dt_Gnss ...
        +  weight_Eagleye*state_velocity(i-1,1:3).*const_dt3(i-1))/(weight_Doppler+weight_Eagleye);
end
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

%
status_dist_refer_3d_loc = zeros(time_SimEpoch-1,1);
status_dist_refer_3d_ref = zeros(time_SimEpoch-1,1);
for i = 2:time_SimEpoch
    status_dist_refer_3d_loc(i,1) = sqrt(status_timedifference(i,1).^2 + status_timedifference(i,2).^2 ...
        +status_timedifference(i,3).^2);
    status_dist_refer_3d_ref(i,1) = sqrt(status_timedifference_ref(i,1).^2 + status_timedifference_ref(i,2).^2 ...
        +status_timedifference_ref(i,3).^2);
end

%%
clear status_PositionResidual

status_PositionResidual(:,1:3) = analize_POE(:,1:3);

%%
% 車両状態管理用パラメータの設定
state_params = struct(...
    'thresholds', struct(...
        'curving', 0.1, ...        % カーブ判定のヨーレート閾値 [rad/s]
        'stopping', 0.5, ...       % 停止判定の速度閾値 [m/s]
        'high_lat_acc', 0.2*9.81));% 高横加速度判定閾値 [m/s^2]

% 状態管理用の初期データ構造
current_data = struct(...
    'yaw_rate', 0, ...
    'velocity', zeros(1,3), ...
    'lateral_acc', 0);

% 状態管理の初期化
prev_state = [];
[vehicle_state, update_flags] = StateManager(current_data, prev_state, state_params);

%%
state_pos_Eagleye_enu = data_correction_Eagleye_enu;

% 疑似誤差生成パラメータの初期化
pseudo_error_params = struct(...
    'weights', struct('gnss', 1, 'imu', 1), ...
    'thresholds', struct(...
        'min_velocity', 0.5, ...    % 最小速度閾値 [m/s]
        'ref_velocity', 5.0, ...    % 基準速度 [m/s]
        'max_error', 10.0), ...     % 最大誤差閾値 [m]
    'outlier', struct('k', 3.0));   % 異常値判定係数

% 初期状態の設定
init_pos = state_pos_Eagleye_enu(1,:);
init_vel = zeros(1,3);  % 初期速度を0とする

% 位置データの初期化
pos_data = struct(...
    'current', init_pos, ...
    'previous', init_pos);

% 速度データの初期化
vel_data = struct(...
    'gnss', init_vel, ...
    'imu', init_vel);

% リファレンスデータの初期化
ref_data = struct(...
    'pos', data_pos_Ref_enu(1,:), ...
    'vel', data_sync_Ref(1,12:14));

% 初期の疑似誤差を0に設定
[init_pseudo_error, ~] = PseudoErrorGenerator(...
    pos_data, vel_data, ref_data, pseudo_error_params);
status_pseudo_observation_error(1,:) = zeros(1,3);  % 初期値は0

%%
% for k = 1:10:1000
%     windowSize = k;
% for k = [2 50 100 360 390]
%     windowSize = k;

    % % k 5 10 40


    %%
    windowSize = 50;

%%
% 共分散更新パラメータの初期化
covariance_params = struct(...
    'ref_std', 0.1, ...         % 基準標準偏差 [m]
    'max_yaw_rate', 0.5, ...    % 最大ヨーレート [rad/s]
    'ref_velocity', 5.0, ...    % 基準速度 [m/s]
    'min_velocity', 0.5);       % 最小速度 [m/s]

% 車両状態の初期化
vehicle_state = struct(...
    'isCurving', false, ...
    'isStopped', true, ...
    'yaw_rate', 0, ...
    'velocity', zeros(1,3));


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

        %-------------------------
        % 現在のセンサデータの更新
        current_data.yaw_rate = element_Y_yaw(i);
        current_data.velocity = state_velocity(i,:);
        current_data.lateral_acc = element_ay_EAGLEYE(i);

        % 車両状態の更新
        [vehicle_state, update_flags] = StateManager(current_data, vehicle_state, state_params);

        % 状態に応じた処理の分岐
        if vehicle_state.isCurving
            % カーブ時の特別処理
            param_obs_noise_gnss_x = param_obs_noise_gnss_x * 1.5;
            param_obs_noise_gnss_y = param_obs_noise_gnss_y * 1.5;
        end

        if vehicle_state.isStopped
            % 停止時の特別処理
            param_obs_noise_velocity = param_obs_noise_velocity * 0.5;
        end

        if update_flags.curveStart
            % カーブ開始時の特別処理
            init_covariance(1:2,1:2) = init_covariance(1:2,1:2) * 1.2;
        end

        % 既存の疑似誤差生成処理
        if option_velocity_type == 0
            wander_x_vel(i,1)  = element_ax_EAGLEYE(i,1)*const_dt2(i);
            wander_x_vel3(i,1) = element_ax_EAGLEYE(i,1)*const_dt3(i);
        end

        %-------------------------
        % 車両状態の更新
        vehicle_state.yaw_rate = element_Y_yaw(i);
        vehicle_state.velocity = state_velocity(i,:);
        vehicle_state.isCurving = abs(element_Y_yaw(i)) > state_params.thresholds.curving;
        vehicle_state.isStopped = norm(state_velocity(i,:)) < state_params.thresholds.stopping;

        %-------------------------

        % 位置データの更新
        pos_data = struct(...
            'current', state_pos_Eagleye_enu(i,:), ...
            'previous', state_pos_Eagleye_enu(i-1,:));

        % 速度データの更新
        vel_data = struct(...
            'gnss', data_velocity(i,:), ...
            'imu', state_velocity(i,:));

        % リファレンスデータの更新
        ref_data = struct(...
            'pos', data_pos_Ref_enu(i,:), ...
            'vel', data_sync_Ref(i,12:14));

        % 疑似誤差の生成
        [pseudo_error, error_stats] = PseudoErrorGenerator(...
            pos_data, vel_data, ref_data, pseudo_error_params);

        % 車両状態に応じた重み付けの更新
        if vehicle_state.isCurving
            pseudo_error_params.weights.imu = 1.2;    % カーブ中はIMUの重みを増加
            pseudo_error_params.weights.gnss = 0.8;
        else
            pseudo_error_params.weights.imu = 0.8;
            pseudo_error_params.weights.gnss = 1.2;   % 直線中はGNSSの重みを増加
        end

        % 疑似誤差の適用
        status_pseudo_observation_error(i,:) = pseudo_error;

        %-------------------------

        % Predict　予測 t=k-1までの状態からt=kの状態を予測
        Pre_KF_roll = KF_roll(i-1) + (data_struct_Eagleye.Roll_rate(i-1,1)+data_struct_Eagleye.Pitch_rate(i-1,1)*sin(KF_roll(i-1))*tan(KF_pitch(i-1))+data_yawrate(i-1,1)*cos(KF_roll(i-1))*tan(KF_pitch(i-1)))*const_dt3(i); %
        % Pre_KF_pitch = KF_pitch(i-1) + (Pre_KF_pitchrate*cos(KF_roll(i-1))-Pre_KF_yawrate*sin(KF_roll(i-1)))*const_dt3(i); %
        Pre_KF_pitch = KF_pitch(i-1) + (data_struct_Eagleye.Pitch_rate(i-1,1)*cos(KF_roll(i-1))-data_yawrate(i-1,1)*sin(KF_roll(i-1)))*const_dt3(i);
        Pre_KF_yaw = KF_yaw(i-1) + data_yawrate(i-1,1)*const_dt3(i); %PΘ(k)=Θ(k-1)+yawrate(k)*const_dt3
        Pre_KF_z = KF_z(i-1) + sin(KF_pitch(i-1))*state_velocity(i)*const_dt3(i);

        % option_calc_type = 0;

        if option_calc_type==0
            Pre_KF_x = KF_x(i-1) - KF_velocity(i-1)*(data_yawrate(i-1,1)^(-1))*(-cos(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))+cos(KF_yaw(i-1,1)));
            Pre_KF_y = KF_y(i-1) - KF_velocity(i-1)*(data_yawrate(i-1,1)^(-1))*(sin(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))-sin(KF_yaw(i-1,1)));
        end
        KF_vec_x(i-1,1) = -KF_velocity(i-1)*(data_yawrate(i-1,1)^(-1))...
            *(-cos(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))+cos(KF_yaw(i-1,1)));
        KF_vec_y(i-1,1) = -KF_velocity(i-1)*(data_yawrate(i-1,1)^(-1))...
            *(sin(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))-sin(KF_yaw(i-1,1)));


        Pre_KF_velocity(1,1) = KF_velocity(i-1) + data_acc_Eagleye_enu3d(i-1,1)*const_dt3(i);


        % A(i)=(Pre_KF_pitchrate*cos(KF_roll(i-1))-Pre_KF_yawrate*sin(KF_roll(i-1)))*const_dt3(i);


        %ヤコビアン

        jF = ...
            [1, 0, 0, 0, 0, state_velocity(i-1)*(data_yawrate(i-1,1)^(-1))*(sin(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))-sin(KF_yaw(i-1,1))), 0
            0, 1, 0, 0, 0, state_velocity(i-1)*(data_yawrate(i-1,1)^(-1))*(cos(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))-cos(KF_yaw(i-1,1))), 0
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
        % メインループ内での曲線補正の適用
        if vehicle_state.isCurving
            % 曲線補正のパラメータ設定
            curve_params = struct(...
                'wheelBase', 2.69, ...  % シエンタのホイールベース
                'cog_height', 0.57, ... % 重心高さ
                'track_width', 1.49, ... % トレッド
                'mass', 1440);         % 車両質量

            % 現在の状態量を取得
            current_pos = [Pre_KF_x, Pre_KF_y, Pre_KF_z];
            current_vel = [wander_x_vel(i), wander_y_vel(i), wander_z_vel(i)];

            % 曲線補正の適用
            [comp_pos, comp_vel, comp_info] = CurveCompensation(...
                current_pos, ...
                current_vel, ...
                element_Y_yaw(i), ...
                element_ay_EAGLEYE(i), ...
                const_dt3(i), ...
                curve_params);

            % 補正結果の適用
            Pre_KF_x = comp_pos(1);
            Pre_KF_y = comp_pos(2);
            Pre_KF_z = comp_pos(3);

            % 観測ノイズ共分散の調整
            if isfield(comp_info, 'slip_angle') && ~isempty(comp_info.slip_angle)
                slip_factor = cos(comp_info.slip_angle);  % スリップ角が大きいほど信頼度低下
                R(1:3,1:3) = R(1:3,1:3) / slip_factor;
            end
        end

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
                    element_R_roll(i), element_P_pitch(i), state_gnss_yaw(i), data_velocity(i)]'; %観測方程式
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

        % 適応的共分散更新
        if flag_gnss(i) == 1 && flag_test3(i) == 1  % GNSS利用可能かつ実数解の場合
            [R_updated, update_info] = AdaptiveCovarianceUpdater(...
                R_GNSS_re, error_stats, vehicle_state, covariance_params);
            R = R_updated;
        elseif flag_gnss(i) == 1 && flag_test3(i) == 0  % GNSS利用可能かつ虚数解の場合
            [R_updated, update_info] = AdaptiveCovarianceUpdater(...
                R_GNSS_im, error_stats, vehicle_state, covariance_params);
            R = R_updated;
        elseif flag_gnss(i) == 0 && flag_test3(i) == 1  % GNSS利用不可かつ実数解の場合
            [R_updated, update_info] = AdaptiveCovarianceUpdater(...
                R_DR_re, error_stats, vehicle_state, covariance_params);
            R = R_updated;
        else  % GNSS利用不可かつ虚数解の場合
            [R_updated, update_info] = AdaptiveCovarianceUpdater(...
                R_DR_im, error_stats, vehicle_state, covariance_params);
            R = R_updated;
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