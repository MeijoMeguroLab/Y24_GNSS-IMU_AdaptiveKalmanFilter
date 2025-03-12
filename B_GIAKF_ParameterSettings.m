% cd B.ParameterSettings
disp('B.ParameterSettings')

%% プロセスノイズパラメータ設定
% 位置関連
param_pros_noise_x     = 0.3;  % プロセスノイズ x座標
param_pros_noise_y     = 0.3;  % プロセスノイズ y座標
param_pros_noise_z     = 0.3;  % プロセスノイズ z座標

% 速度関連
param_pros_noise_v_x   = 0.7;  % プロセスノイズ x方向速度
param_pros_noise_v_y   = 0.7;  % プロセスノイズ y方向速度
param_pros_noise_v_z   = 0.7;  % プロセスノイズ z方向速度
param_pros_noise_velocity = 0.7;  % プロセスノイズ 速度

% 姿勢角関連
param_pros_noise_roll  = 2.0;   % プロセスノイズ ロール角
param_pros_noise_pitch = 0.25;  % プロセスノイズ ピッチ角
param_pros_noise_yaw   = 0.0009; % プロセスノイズ 方位角

% 角速度関連
param_pros_noise_rollrate  = 0.01;  % プロセスノイズ ロールレート
param_pros_noise_pitchrate = 0.1;  % プロセスノイズ ピッチレート
param_pros_noise_yawrate   = 0.1;  % プロセスノイズ ヨーレート

%% 観測ノイズパラメータ設定
% 姿勢角関連（実数値時）
param_obs_noise_roll  = 90.0;  % 観測ノイズ ロール角（実数）
param_obs_noise_pitch = 100.0; % 観測ノイズ ピッチ角（実数）

% 姿勢角関連（虚数値時）
param_obs_noise_im_roll  = param_pros_noise_roll*1;   % 観測ノイズ ロール角（虚数）
param_obs_noise_im_pitch = param_pros_noise_pitch*1;  % 観測ノイズ ピッチ角（虚数）

% 角速度関連
param_obs_noise_rollrate  = 0.01;   % 観測ノイズ ロールレート
param_obs_noise_pitchrate = 0.01;   % 観測ノイズ ピッチレート
param_obs_noise_yawrate   = 0.0001; % 観測ノイズ ヨーレート

% 速度関連
param_obs_noise_velocity = 2;  % 観測ノイズ 速度

% GNSS関連（Fix解時）
param_obs_noise_gnss_x = 0.01;  % GNSS時の観測ノイズ x座標
param_obs_noise_gnss_y = 0.01;  % GNSS時の観測ノイズ y座標
param_obs_noise_gnss_z = 0.81;  % GNSS時の観測ノイズ z座標
param_obs_noise_gnss_yaw = 100/180*pi;  % GNSS時の観測ノイズ ヨー角
param_obs_noise_gnss_pitch = 1/180*pi;  % GNSS時の観測ノイズ ピッチ角
param_obs_noise_gnss_velocity = 0.03;   % GNSS時の観測ノイズ 速度

% DR関連（Dead Reckoning, GNSS不使用時）
param_obs_noise_DR_x = param_pros_noise_x*1;  % DR時の観測ノイズ x座標
param_obs_noise_DR_y = param_pros_noise_y*1;  % DR時の観測ノイズ y座標
param_obs_noise_DR_z = param_pros_noise_z*1;  % DR時の観測ノイズ z座標
param_obs_noise_DR_yaw = param_pros_noise_yaw*1;  % DR観測ノイズ ヨー角
param_obs_noise_DR_pitch = param_pros_noise_pitch*1;  % DR観測ノイズ ピッチ角
param_obs_noise_DR_velocity = param_obs_noise_velocity*1;  % DR観測ノイズ 速度

% その他パラメータ
handle_doppler_number = 10;  % ドップラー速度の信頼性判定値 [m/s]

%% カーブ補正と改良型適応フィルタのパラメータ
% 車両パラメータは既存の値を使用
param_vehicle.slip_coefficient = 0.15;   % タイヤスリップ係数
param_vehicle.bank_angle_max = 0.05;     % 最大路面バンク角[rad]

% シエンタ用車両パラメータ
param_vehicle = struct(...
    'wheelBase', 2.75, ...      % ホイールベース[m]
    'cog_height', 0.55, ...     % 重心高さ[m]
    'track_width', 1.49, ...    % トレッド幅[m]
    'slip_coefficient', 0.15, ...% タイヤスリップ係数
    'bank_angle_max', 0.05);    % 最大路面バンク角[rad]

% 状態検出パラメータ
param_state_detect = struct(...
    'curve_threshold', 0.1, ... % カーブ判定用ヨーレート閾値[rad/s]
    'stop_threshold', 0.5);     % 停止判定用速度閾値[m/s]

% 適応フィルタパラメータ
param_adaptive = struct(...
    'windowSize', 50, ...                % 観測ウィンドウサイズ
    'minCovThreshold', 0.01, ...         % 最小共分散閾値
    'gnss_weight_fix', 0.67, ...         % GNSS重み (FIX時)
    'gnss_weight_float', 0.5, ...        % GNSS重み (FLOAT時)
    'imu_weight_straight', 0.33, ...     % IMU重み (直線時)
    'imu_weight_curve', 0.6, ...         % IMU重み (カーブ時)
    'imu_weight_stop', 0.2);             % IMU重み (停止時)

% カーブエラー補正モデルパラメータ
param_curve_error = struct(...
    'slip_kf_p', 0.1, ...                % スリップ角KF共分散初期値
    'slip_kf_q', 0.01, ...               % スリップ角KFプロセスノイズ
    'slip_kf_r', 0.1, ...                % スリップ角KF観測ノイズ
    'roll_stiffness', 0.8, ...           % ロール剛性係数
    'roll_damping', 0.2, ...             % ロール減衰係数
    'curve_covariance_scale', 1.2);      % カーブ区間共分散スケール
%% 初期値設定
init_roll  = element_R_roll(1,1);    % ロール角初期値
init_pitch = element_P_pitch(1,1);   % ピッチ角初期値
init_yaw   = element_Y_yaw(1,1);     % ヨー角初期値
init_rollrate  = data_struct_Eagleye.Roll_rate(1,1);   % ロールレート初期値
init_pitchrate = data_struct_Eagleye.Pitch_rate(1,1);  % ピッチレート初期値
init_yawrate   = data_yawrate(1,1);                    % ヨーレート初期値
init_velocity = data_velocity(1,1);                    % 速度初期値

% カルマンフィルタ結果格納用配列
KF_x = zeros(time_SimEpoch,1);
KF_y = zeros(time_SimEpoch,1);
KF_z = zeros(time_SimEpoch,1);
KF_roll  = zeros(time_SimEpoch,1);
KF_pitch = zeros(time_SimEpoch,1);
KF_yaw   = zeros(time_SimEpoch,1);
KF_velocity = zeros(time_SimEpoch,1);
KF_covariance = zeros(time_SimEpoch,49);

%% 共分散行列の設定
% プロセスノイズ共分散行列
Q = [
    param_pros_noise_x^2, 0, 0, 0, 0, 0, 0;
    0, param_pros_noise_y^2, 0, 0, 0, 0, 0;
    0, 0, param_pros_noise_z^2, 0, 0, 0, 0;
    0, 0, 0, param_pros_noise_roll^2, 0, 0, 0;
    0, 0, 0, 0, param_pros_noise_pitch^2, 0, 0;
    0, 0, 0, 0, 0, param_pros_noise_yaw^2, 0;
    0, 0, 0, 0, 0, 0, param_pros_noise_velocity^2
];

% 観測ノイズ共分散行列の設定
if option_Pitch_type == 0  % ピッチ角の推定を加速度と角速度から求める場合
    % GNSSが使えない & 観測値のロール角、ピッチ角が虚数の場合
    R_DR_im = [
        param_obs_noise_DR_x^2, 0, 0, 0, 0, 0, 0;
        0, param_obs_noise_DR_y^2, 0, 0, 0, 0, 0;
        0, 0, param_obs_noise_DR_z^2, 0, 0, 0, 0;
        0, 0, 0, param_obs_noise_im_roll^2, 0, 0, 0;
        0, 0, 0, 0, param_obs_noise_im_pitch^2, 0, 0;
        0, 0, 0, 0, 0, param_obs_noise_DR_yaw^2, 0;
        0, 0, 0, 0, 0, 0, param_obs_noise_DR_velocity^2
    ];

    % GNSSが使えない & 観測値のロール角、ピッチ角が実数の場合
    R_DR_re = [
        param_obs_noise_DR_x^2, 0, 0, 0, 0, 0, 0;
        0, param_obs_noise_DR_y^2, 0, 0, 0, 0, 0;
        0, 0, param_obs_noise_DR_z^2, 0, 0, 0, 0;
        0, 0, 0, param_obs_noise_roll^2, 0, 0, 0;
        0, 0, 0, 0, param_obs_noise_pitch^2, 0, 0;
        0, 0, 0, 0, 0, param_obs_noise_DR_yaw^2, 0;
        0, 0, 0, 0, 0, 0, param_obs_noise_DR_velocity^2
    ];

    % FIX解 & 観測値のロール角、ピッチ角が虚数の場合
    R_GNSS_im = [
        param_obs_noise_gnss_x^2, 0, 0, 0, 0, 0, 0;
        0, param_obs_noise_gnss_y^2, 0, 0, 0, 0, 0;
        0, 0, param_obs_noise_gnss_z^2, 0, 0, 0, 0;
        0, 0, 0, param_obs_noise_im_roll^2, 0, 0, 0;
        0, 0, 0, 0, param_obs_noise_im_pitch^2, 0, 0;
        0, 0, 0, 0, 0, param_obs_noise_gnss_yaw^2, 0;
        0, 0, 0, 0, 0, 0, param_obs_noise_gnss_velocity^2
    ];

    % FIX解 & 観測値のロール角、ピッチ角が実数の場合
    R_GNSS_re = [
        param_obs_noise_gnss_x^2, 0, 0, 0, 0, 0, 0;
        0, param_obs_noise_gnss_y^2, 0, 0, 0, 0, 0;
        0, 0, param_obs_noise_gnss_z^2, 0, 0, 0, 0;
        0, 0, 0, param_obs_noise_roll^2, 0, 0, 0;
        0, 0, 0, 0, param_obs_noise_pitch^2, 0, 0;
        0, 0, 0, 0, 0, param_obs_noise_gnss_yaw^2, 0;
        0, 0, 0, 0, 0, 0, param_obs_noise_gnss_velocity^2
    ];

elseif option_Pitch_type == 1  % ピッチ角をGNSSのドップラー速度から求める場合
    % GNSSが使えない & 観測値のロール角が虚数の場合
    R_DR_im = [
        param_obs_noise_DR_x^2, 0, 0, 0, 0, 0, 0;
        0, param_obs_noise_DR_y^2, 0, 0, 0, 0, 0;
        0, 0, param_obs_noise_DR_z^2, 0, 0, 0, 0;
        0, 0, 0, param_obs_noise_im_roll^2, 0, 0, 0;
        0, 0, 0, 0, param_obs_noise_DR_pitch^2, 0, 0;
        0, 0, 0, 0, 0, param_obs_noise_DR_yaw^2, 0;
        0, 0, 0, 0, 0, 0, param_obs_noise_DR_velocity^2
    ];

    % GNSSが使えない & 観測値のロール角が実数の場合
    R_DR_re = [
        param_obs_noise_DR_x^2, 0, 0, 0, 0, 0, 0;
        0, param_obs_noise_DR_y^2, 0, 0, 0, 0, 0;
        0, 0, param_obs_noise_DR_z^2, 0, 0, 0, 0;
        0, 0, 0, param_obs_noise_roll^2, 0, 0, 0;
        0, 0, 0, 0, param_obs_noise_DR_pitch^2, 0, 0;
        0, 0, 0, 0, 0, param_obs_noise_DR_yaw^2, 0;
        0, 0, 0, 0, 0, 0, param_obs_noise_DR_velocity^2
    ];

    % FIX解 & 観測値のロール角が虚数の場合
    R_GNSS_im = [
        param_obs_noise_gnss_x^2, 0, 0, 0, 0, 0, 0;
        0, param_obs_noise_gnss_y^2, 0, 0, 0, 0, 0;
        0, 0, param_obs_noise_gnss_z^2, 0, 0, 0, 0;
        0, 0, 0, param_obs_noise_im_roll^2, 0, 0, 0;
        0, 0, 0, 0, param_obs_noise_gnss_pitch^2, 0, 0;
        0, 0, 0, 0, 0, param_obs_noise_gnss_yaw^2, 0;
        0, 0, 0, 0, 0, 0, param_obs_noise_gnss_velocity^2
    ];

    % FIX解 & 観測値のロール角、ピッチ角が実数の場合
    R_GNSS_re = [
        param_obs_noise_gnss_x^2, 0, 0, 0, 0, 0, 0;
        0, param_obs_noise_gnss_y^2, 0, 0, 0, 0, 0;
        0, 0, param_obs_noise_gnss_z^2, 0, 0, 0, 0;
        0, 0, 0, param_obs_noise_roll^2, 0, 0, 0;
        0, 0, 0, 0, param_obs_noise_gnss_pitch^2, 0, 0;
        0, 0, 0, 0, 0, param_obs_noise_gnss_yaw^2, 0;
        0, 0, 0, 0, 0, 0, param_obs_noise_gnss_velocity^2
    ];
end

% 共分散行列の初期値
init_covariance = [
    0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0;
    0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0;
    0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0;
    0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0;
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0;
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001
];

cd ..