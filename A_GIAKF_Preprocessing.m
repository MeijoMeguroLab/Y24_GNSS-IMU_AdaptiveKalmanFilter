%% 入力データの読み込み
% ファイルパス設定
input_pos_name = ['observation' '\' 'nagoya1_float']; % pos file
input_ref_name = ['20230616' '\' 'POSLV' '\' 'POSLV_run2']; % POSLV
input_imu_name = ['20230616' '\' 'Eagleye' '\' 'run2' '\' 'eagleye_log']; % eagleye 2023 log file

% ファイル読み込み
input_pos = ['..\0.Data\' input_pos_name '.pos'];
ref0 = load(['..\0.Data\' input_ref_name '.mat']); 
raw_Ref = ref0.trajectory; 
clear ref0
raw_Eagleye_table = readtable(['..\0.Data\' input_imu_name '.csv']);
raw_EagleyeLog = table2array(readtable(['..\0.Data\' input_imu_name '.csv']));
clc

% kinematic測位ファイルの読み込み
[raw_dataPos, raw_basePos] = read_pos(input_pos);

clear input_pos_name input_ref_name input_imu_name input_pos

%% 時間同期/設定

% パラメータの設定
time_offset = 0; % sec
time_Eagleye = raw_EagleyeLog(:,1)/ 10.^9;
time_TOW_POSLV = raw_Ref(:,1);
index_eval_section = find(raw_EagleyeLog(:,86) == 1);
time_TOW_Eagleye_ = raw_EagleyeLog(:,8)/1000;
time_TOW_Eagleye = time_TOW_Eagleye_;
data_yawrate_Eagleye = -raw_EagleyeLog(:,4); % ROSの軸とPOSLVの軸が逆なのでマイナス

%% TOWの設定
tic
disp('TOWの設定')
flag_GNSS = zeros(length(raw_EagleyeLog),1);
for i = 2:length(raw_EagleyeLog)
    if time_TOW_Eagleye_(i) ~= time_TOW_Eagleye_(i-1)
        flag_GNSS(i) = 1;
    end
end
for i = 2:length(raw_EagleyeLog)
    const_dt = time_Eagleye(i) - time_Eagleye(i-1);
    if flag_GNSS(i) == 1
        time_TOW_Eagleye(i) = time_TOW_Eagleye_(i);
    else
        time_TOW_Eagleye(i) = time_TOW_Eagleye(i-1) + const_dt;
    end
end

%% POSLVと同期とデータの設定
f = waitbar(0,'効率化開始');
disp('POSLVと同期とデータの設定')
data_sync0_Ref = zeros(length(time_Eagleye),size(raw_Ref,2));
flag_poslv = zeros(length(time_Eagleye),1);
disp(['POSLVのオフセット時間: ', num2str(time_offset), '秒'])
for i = 1 : length(time_Eagleye)
    [Y,I] = min(abs(time_TOW_Eagleye(i) + time_offset - time_TOW_POSLV));
    if abs(Y) < 0.005
        data_sync0_Ref(i,:) = raw_Ref(I,:);
        flag_poslv(i) = I;
        if mod(floor(100*i/length(time_Eagleye)),3) == 0
            if floor(rem(i,400)) == 0
                waitbar(i/length(time_Eagleye), f, ['読み込み中    (' num2str(i) '/' num2str(length(time_Eagleye)) ')']);
            elseif floor(rem(i,400)) == 1
                waitbar(i/length(time_Eagleye), f, ['読み込み中.   (' num2str(i) '/' num2str(length(time_Eagleye)) ')']);
            elseif floor(rem(i,400)) == 2
                waitbar(i/length(time_Eagleye), f, ['読み込み中..  (' num2str(i) '/' num2str(length(time_Eagleye)) ')']);
            elseif floor(rem(i,400)) == 3
                waitbar(i/length(time_Eagleye), f, ['読み込み中... (' num2str(i) '/' num2str(length(time_Eagleye)) ')']);
            end
        end
    end
end
close(f)

% flag_poslvの 全0 判定
index_poslv_ = find(flag_poslv > 0);
index_poslv = index_poslv_(1):length(time_Eagleye);
toc

clear f index_poslv_ Y I

%% 時刻同期
tic
disp('時刻同期')
M = zeros(length(raw_dataPos), 1);
I = zeros(length(raw_dataPos), 1);
for i = 1:length(raw_dataPos)
    [M(i,1), I(i,1)] = min(abs(time_TOW_Eagleye(:) - raw_dataPos(i,2)));
end

dI = zeros(length(I)-1,1);
for i = 1:length(I)-1
    dI(i,1) = I(i+1) - I(i);
end
I0 = ~dI; 
Ig = find(dI);
I1 = I; 
I1(I0) = [];

data_sync_data_Gnss = raw_dataPos(Ig(:,1),:);
data_sync_Eagleye = raw_EagleyeLog(I1(:,1),:);
data_sync_Eagleye(:,8) = time_TOW_Eagleye(I1(:,1),:);
data_sync_Eagleye(end,:) = [];
data_sync_Ref = data_sync0_Ref(I1(:,1),:);
data_sync_Ref(end,:) = [];
toc

const_dt_Gnss = raw_dataPos(2,2) - raw_dataPos(1,2);
const_dt_Eagleye = data_sync_Eagleye(2,8) - data_sync_Eagleye(1,8);
const_dt_Ref = raw_Ref(2,1) - raw_Ref(1,1);

clear I0 Ig I1 dI M I 

%% Eagleye構造体作成
field1 = 'Time';
field2 = 'Roll_rate';
field3 = 'Pitch_rate';
field4 = 'Yaw_rate';
field5 = 'roll';
field6 = 'pitch';
field7 = 'yaw';
data_struct_Eagleye = struct(field1,[],field2,[],field3,[],field4,[],field5,[],field6,[],field7,[]);
clear field1 field2 field3 field4 field5 field6 field7

data_struct_Eagleye.Time = data_sync_Eagleye(:,8);
time_Eagleye = data_sync_Eagleye(:,8);
data_struct_Eagleye.Roll_rate = data_sync_Eagleye(:,2);
data_struct_Eagleye.Pitch_rate = data_sync_Eagleye(:,3);
data_struct_Eagleye.Yaw_rate = data_sync_Eagleye(:,4);
data_struct_Eagleye.roll = data_sync_Eagleye(:,124);
data_struct_Eagleye.pitch = data_sync_Eagleye(:,115);
data_struct_Eagleye.yaw = data_sync_Eagleye(:,58);

%% 時刻設定
time_SimEpoch = length(data_struct_Eagleye.Time(:,1));
const_g = 9.80665; % 重力加速度

% 時間間隔の計算
const_dt2 = zeros(time_SimEpoch,1);
const_dt3 = zeros(time_SimEpoch,1);
for i = 1:time_SimEpoch
    if i < time_SimEpoch
        const_dt2(i) = time_Eagleye(i+1) - time_Eagleye(i);
        const_dt3(i) = time_Eagleye(i+1) - time_Eagleye(i);
    end
    if i == time_SimEpoch
        const_dt2(i) = 0.01;
        const_dt3(i) = 0.01;
    elseif const_dt3(i) > 10
        const_dt3(i) = 0.2;
    end
end

% 時刻データの調整
time_Obs(:,1) = data_sync_data_Gnss(:,2); 
time_Obs = time_Obs - time_Obs(1,1);
time_Ref(:,1) = data_sync_Ref(:,1);       
time_Ref = time_Ref - data_sync_Ref(1,1);

%% データ解析のための準備
disp('対象データ解析')

% 位置データ
data_sync_Gnss_xyz(:,1) = data_sync_data_Gnss(:,3);
data_sync_Gnss_xyz(:,2) = data_sync_data_Gnss(:,4);
data_sync_Gnss_xyz(:,3) = data_sync_data_Gnss(:,5);

% 速度データ
data_vel_Obs(:,1) = data_sync_data_Gnss(:,16);
data_vel_Obs(:,2) = data_sync_data_Gnss(:,17);
data_vel_Obs(:,3) = data_sync_data_Gnss(:,18);

% リファレンスデータ
data_pos_Ref_xyz(:,1) = data_sync_Ref(:,6);
data_pos_Ref_xyz(:,2) = data_sync_Ref(:,7);
data_pos_Ref_xyz(:,3) = data_sync_Ref(:,8);

%% 座標をXYZからENUに変換
handle_ORG_XYZ = [data_sync_Gnss_xyz(1,1) data_sync_Gnss_xyz(1,2) data_sync_Gnss_xyz(1,3)];

% 観測データのENU変換
data_pos_Obs_enu = zeros(length(data_sync_Gnss_xyz),3);
data_vel_Obs_enu = zeros(length(data_sync_Gnss_xyz),3);
for i = 1:length(data_sync_Gnss_xyz)
    data_pos_Obs_enu(i,:) = xyz2enu(data_sync_Gnss_xyz(i,:), handle_ORG_XYZ);
    data_vel_Obs_enu(i,:) = xyz2enu_vel(data_vel_Obs(i,:), handle_ORG_XYZ);
end 

% Eagleyeデータのあ
state_pos_Eagleye_enu = zeros(length(data_sync_Eagleye),3);
for i = 1:length(data_sync_Eagleye(:,9:11))
    state_pos_Eagleye_enu(i,:) = xyz2enu(data_sync_Eagleye(i,9:11), handle_ORG_XYZ);
end

% リファレンスデータのENU変換
data_pos_Ref_enu = zeros(length(data_pos_Ref_xyz),3);
for i = 1:length(data_pos_Ref_xyz)
    data_pos_Ref_enu(i,:) = xyz2enu(data_pos_Ref_xyz(i,:), handle_ORG_XYZ);
end

% 速度データの要素別分解
state_vel_Obs_enu_x = data_vel_Obs_enu(:,1);
state_vel_Obs_enu_y = data_vel_Obs_enu(:,2);
state_vel_Obs_enu_z = data_vel_Obs_enu(:,3);
state_vel_Ref_enu_x = data_sync_Ref(:,12);
state_vel_Ref_enu_y = data_sync_Ref(:,13);
state_vel_Ref_enu_z = data_sync_Ref(:,14);

%% 速度フラグの設定
data_vel_Obs_enu_3d = zeros(length(data_vel_Obs_enu),1);
for i = 1:length(data_vel_Obs_enu)
    data_vel_Obs_enu_3d(i,1) = sqrt(data_vel_Obs_enu(i,1)^2 + ...
                                     data_vel_Obs_enu(i,2)^2 + ...
                                     data_vel_Obs_enu(i,3)^2);
end

% 停止状態の検出
index_v_stop = find(abs(data_vel_Obs_enu_3d(:,1)) < 0.3);
flag_v_stop = zeros(size(data_sync_data_Gnss, 1), 1);
flag_v_stop(index_v_stop) = 1;

% FIX状態の検出
index_gnss = find(data_sync_data_Gnss(:,6) == 1);
index_nfix = find(data_sync_data_Gnss(:,6) ~= 1);
flag_gnss = zeros(size(data_sync_Gnss_xyz, 1), 1);
flag_gnss(index_gnss) = 1;

% missfixをfloatフラグに変換
data_difference = sqrt((data_sync_Gnss_xyz(:,1) - data_pos_Ref_xyz(:,1)).^2 + ...
                       (data_sync_Gnss_xyz(:,2) - data_pos_Ref_xyz(:,2)).^2 + ...
                       (data_sync_Gnss_xyz(:,3) - data_pos_Ref_xyz(:,3)).^2); 
index_missfix = zeros(length(data_sync_Gnss_xyz),1);
for i = 1:length(data_sync_Gnss_xyz)
    if data_sync_data_Gnss(i,6) == 1 && data_difference(i,1) > 1
        index_missfix(i,1) = 1;
    else
        index_missfix(i,1) = 0;
    end
end

% インデックスの更新
index_missfix2 = find(index_missfix == 1);
for i = 1:length(index_missfix2)
    index_gnss(index_gnss == index_missfix2(i)) = [];
    flag_gnss(index_missfix2(i)) = 0;
end
index_nfix = vertcat(index_nfix, index_missfix2(:)); 
index_nfix = sort(index_nfix);

clear index_missfix index_missfix2

%% ヨーレートの設定
handle_VEL_STOP = 0.01;
handle_YawrateOffset_Stop = data_sync_Eagleye(:,62);
handle_YawrateOffset_Eagleye = data_sync_Eagleye(:,68);

data_velocity_REF = sqrt(data_sync_Ref(:,12).^2 + data_sync_Ref(:,13).^2 + data_sync_Ref(:,14).^2);

data_yawrate = zeros(time_SimEpoch,1);
for i = 1:time_SimEpoch
    if i == 0
        data_yawrate(i,1) = data_sync_Eagleye(i,4) + handle_YawrateOffset_Stop(i,1);
    end
    if i > 1 && data_velocity_REF(i) > handle_VEL_STOP
        data_yawrate(i,1) = data_sync_Eagleye(i,4) + handle_YawrateOffset_Eagleye(i,1);
    else
        data_yawrate(i,1) = data_sync_Eagleye(i,4) + handle_YawrateOffset_Stop(i,1);
    end
end
data_sync_Eagleye(:,4) = data_yawrate(:,1);

%% 加速度と姿勢角の準備
% IMUデータの取り出し
element_ax_EAGLEYE =  data_sync_Eagleye(:,5);  % 加速度 x軸
element_ay_EAGLEYE =  data_sync_Eagleye(:,6);  % 加速度 y軸
element_az_EAGLEYE = -data_sync_Eagleye(:,7);  % 加速度 z軸

% 速度計算
data_velocity(:,1:3) = data_vel_Obs_enu(:,1:3) * const_dt_Gnss;

% x軸速度（前後速度）
wander_x_vel = zeros(time_SimEpoch,1);
wander_x_vel3 = zeros(time_SimEpoch,1);
for i = 1:time_SimEpoch
    if option_velocity_type == 0
        wander_x_vel(i,1) = element_ax_EAGLEYE(i,1) * const_dt2(i);   % IMU加速度から計算
        wander_x_vel3(i,1) = element_ax_EAGLEYE(i,1) * const_dt3(i);  % IMU加速度から計算
    else
        wander_x_vel(i,1) = data_velocity(i,1);  % GNSSドップラー速度を使用
        wander_x_vel3(i,1) = data_velocity(i,1); % GNSSドップラー速度を使用
    end
end

% y軸速度（横速度）
wander_y_vel = zeros(time_SimEpoch,1);
wander_y_vel3 = zeros(time_SimEpoch,1);
for i = 1:time_SimEpoch
    if option_velocity_type == 0
        wander_y_vel(i,1) = element_ay_EAGLEYE(i,1) * const_dt2(i);   % IMU加速度から計算
        wander_y_vel3(i,1) = element_ay_EAGLEYE(i,1) * const_dt3(i);  % IMU加速度から計算
    else
        wander_y_vel(i,1) = data_velocity(i,2);  % GNSSドップラー速度を使用
        wander_y_vel3(i,1) = data_velocity(i,2); % GNSSドップラー速度を使用
    end
end

% z軸速度（上下速度）
wander_z_vel = zeros(time_SimEpoch,1);
wander_z_vel3 = zeros(time_SimEpoch,1);
for i = 1:time_SimEpoch
    if option_velocity_type == 0
        wander_z_vel(i,1) = element_az_EAGLEYE(i,1) * const_dt2(i);   % IMU加速度から計算
        wander_z_vel3(i,1) = element_az_EAGLEYE(i,1) * const_dt3(i);  % IMU加速度から計算
    else
        wander_z_vel(i,1) = data_velocity(i,3);  % GNSSドップラー速度を使用
        wander_z_vel3(i,1) = data_velocity(i,3); % GNSSドップラー速度を使用
    end
end

%% 速度の微分計算
% 前後速度（x軸）の微分
wander_x_vel_diff = zeros(time_SimEpoch,1);
wander_x_vel_diff3 = zeros(time_SimEpoch,1);
for i = 1:time_SimEpoch
    if i == 1
        wander_x_vel_diff(i,1) = element_ax_EAGLEYE(1,1);
    else
        wander_x_vel_diff(i,1) = (wander_x_vel(i-1,1) - wander_x_vel(i,1)) / const_dt2(i);
        wander_x_vel_diff3(i,1) = (wander_x_vel(i-1,1) - wander_x_vel(i,1)) / const_dt3(i);
    end
end

% 横速度（y軸）の微分
wander_y_vel_diff = zeros(time_SimEpoch,1);
wander_y_vel_diff3 = zeros(time_SimEpoch,1);
for i = 1:time_SimEpoch
    if i == 1
        wander_y_vel_diff(i,1) = element_ay_EAGLEYE(1,1);
    else
        wander_y_vel_diff(i,1) = (wander_y_vel(i-1,1) - wander_y_vel(i,1)) / const_dt2(i);
        wander_y_vel_diff3(i,1) = (wander_y_vel(i-1,1) - wander_y_vel(i,1)) / const_dt3(i);
    end
end

% データ数を調整
wander_x_vel_diff2 = zeros(time_SimEpoch,1);
wander_y_vel_diff2 = zeros(time_SimEpoch,1);
wander_x_vel_diff3_ = zeros(time_SimEpoch,1);
wander_y_vel_diff3_ = zeros(time_SimEpoch,1);
for i = 1:time_SimEpoch
    if i == 1
        wander_x_vel_diff2(i,1) = element_ax_EAGLEYE(1,1);
        wander_y_vel_diff2(i,1) = element_ay_EAGLEYE(1,1);
    end

    if i > 1
        wander_x_vel_diff2(i,1) = wander_x_vel_diff(i-1,1);
        wander_y_vel_diff2(i,1) = wander_y_vel_diff(i-1,1);
        wander_x_vel_diff3_(i,1) = wander_x_vel_diff3(i-1,1);
        wander_y_vel_diff3_(i,1) = wander_y_vel_diff3(i-1,1);
    end
end

%% 姿勢角計算のための変数セット
element_Vx = wander_x_vel;
element_Vy = wander_y_vel;
element_Vz = wander_z_vel;
element_Vx3 = wander_x_vel3;
element_Vy3 = wander_y_vel3;
element_Vz3 = wander_z_vel3;
state_velocity = [element_Vx3 element_Vy3 element_Vz3];
element_Ax = element_ax_EAGLEYE;
element_Ay = -element_ay_EAGLEYE;
element_Az = -element_az_EAGLEYE;
element_rr = data_struct_Eagleye.Roll_rate;
element_pr = -data_struct_Eagleye.Pitch_rate;
element_yr = -data_struct_Eagleye.Yaw_rate;

% ピッチ角の計算
wander_test_pitch_value = zeros(time_SimEpoch,1);
wander_obs_pitch = zeros(time_SimEpoch,1);
for i = 1:time_SimEpoch
    wander_test_pitch_value(i,1) = (wander_x_vel_diff2(i,1) + ...
                                  element_pr(i,1) * element_Vz(i,1) - ...
                                  element_yr(i,1) * element_Vy(i,1) - ...
                                  element_Ax(i,1)) / -const_g;
    wander_obs_pitch(i,1) = asin((wander_x_vel_diff2(i,1) + ...
                               element_pr(i,1) * element_Vz(i,1) - ...
                               element_yr(i,1) * element_Vy(i,1) - ...
                               element_Ax(i,1)) / -const_g);
end

handle_WinsowSize = 100;
wander_obs_pitch = movmean(wander_obs_pitch, handle_WinsowSize);

% ロール角の計算
wander_roll_value = zeros(time_SimEpoch,1);
wander_obs_roll = zeros(time_SimEpoch,1);
for i = 1:time_SimEpoch
    wander_roll_value(i,1) = (wander_y_vel_diff2(i,1) + ...
                            element_yr(i,1) * element_Vx(i,1) - ...
                            element_rr(i,1) * element_Vz(i,1) - ...
                            element_Ay(i,1)) / (-const_g * cos(wander_obs_pitch(i,1)));
    wander_obs_roll(i,1) = asin((wander_y_vel_diff2(i,1) + ...
                              element_yr(i,1) * element_Vx(i,1) - ...
                              element_rr(i,1) * element_Vz(i,1) - ...
                              element_Ay(i,1)) / (-const_g * cos(wander_obs_pitch(i,1))));
end

handle_WinsowSize = 100;
wander_obs_roll = movmean(wander_obs_roll, handle_WinsowSize);

% 姿勢角データの設定
data_angle_roll_obs = wander_obs_roll;
data_angle_pitch_obs = wander_obs_pitch;

% 有効データフラグの設定
flag_obs_roll = (imag(data_angle_roll_obs) == 0);   % ロール角が実数のデータフラグ
flag_obs_pitch = (imag(data_angle_pitch_obs) == 0); % ピッチ角が実数のデータフラグ
flag_obs_real = flag_obs_roll + flag_obs_pitch;     % 両方実数のデータフラグ

for i = 1:time_SimEpoch
    if flag_obs_real(i,1) == 2
        flag_obs_real(i,1) = 1;
    else
        flag_obs_real(i,1) = 0;
    end
end

index_obs_real = find(flag_obs_real(:,1) == 1);
flag_test3 = zeros(length(time_Eagleye),1);
flag_test3(index_obs_real(:, 1)) = 1;

%% 方位角データの処理
data_Heading_POSLV = data_sync_Ref(:,11) / 180 * pi;
data_Heading_Eagleye = data_sync_Eagleye(:,58);
flag_HeadingRAW = data_sync_Eagleye(:,57);

% POSLVデータの処理
data_POSLV_enu = data_sync_Ref(:,12:14);
data_velocity_POSLV = sqrt(data_POSLV_enu(:,1).^2 +  ...
                          data_POSLV_enu(:,2).^2 +  ...
                          data_POSLV_enu(:,3).^2);
index_vel = find(data_velocity_POSLV > 0.1); % 移動中のインデックス

index_GNSS = find(flag_GNSS == 1);
index_eval_section = find(data_sync_Eagleye(:,110) == 1);
index_GNSS = intersect(index_GNSS, index_eval_section);

% GNSS方位角の計算
handle_Heading_GNSS = zeros(length(data_sync_data_Gnss) ,1);
for i = 1:length(data_sync_data_Gnss)
    handle_Heading_GNSS(i) = atan2(state_vel_Obs_enu_x(i,1), state_vel_Obs_enu_y(i,1));
end

% POSLV方位角の補正
index_fastvel = find(data_velocity_POSLV > 3);
index_ = intersect(index_fastvel, index_GNSS);
wander_T = mod(handle_Heading_GNSS(index_), 2*pi) - mod(data_Heading_POSLV(index_), 2*pi);
handle_offset_POSLVHeading = median(wander_T);
data_Heading_POSLV = data_Heading_POSLV + handle_offset_POSLVHeading;
disp(['POSLVのオフセット角: ', num2str(handle_offset_POSLVHeading * 180/pi), ' deg'])

%% アンテナ位置の補正
flag_eval = zeros(length(data_sync_Eagleye),1);
flag_eval(index_eval_section) = 1;
data_correction_Eagleye_enu = Correct_AntPos(handle_AntVec, data_Heading_POSLV, flag_eval, state_pos_Eagleye_enu);
data_correction_GNSS_enu = Correct_AntPos(handle_AntVec, data_Heading_POSLV, flag_eval, data_pos_Obs_enu);

%% 加速度、角速度の設定
element_R_roll = data_sync_Eagleye(:,2);    % 角速度 ロール角
element_P_pitch = data_sync_Eagleye(:,3);   % 角速度 ピッチ角
element_Y_yaw = data_sync_Eagleye(:,4);     % 角速度 ヨー角

element_rollrate_POSLV = data_sync_Ref(:,24);   % 角速度 ロール角 POSLV
element_pitchrate_POSLV = data_sync_Ref(:,25);  % 角速度 ピッチ角 POSLV
element_yawrate_POSLV = data_sync_Ref(:,26);    % 角速度 ヨー角 POSLV

% 観測量
element_ax_EAGLEYE = data_sync_Eagleye(:,5);    % 加速度 x軸
element_ay_EAGLEYE = data_sync_Eagleye(:,6);    % 加速度 y軸
element_az_EAGLEYE = -data_sync_Eagleye(:,7);   % 加速度 z軸

element_ax_POSLV = data_sync_Ref(:,21);  % 加速度 x軸 POSLV
element_ay_POSLV = data_sync_Ref(:,22);  % 加速度 y軸 POSLV
element_az_POSLV = data_sync_Ref(:,23);  % 加速度 z軸 POSLV

%% 角速度のオフセット除去
% キャリブレーションパラメータ
handle_gyro_offset_x = 0;
handle_gyro_offset_y = 0;
handle_gyro_offset_z = 0;
handle_Calibration_Time = 30;  % キャリブレーション時間（秒）

% 初期値
init_x = 0;
init_y = 0;
init_z = 0;

% オフセット計算
for i = 1:handle_Calibration_Time/0.01
    handle_gyro_offset_x = handle_gyro_offset_x + element_R_roll(i,1);
    handle_gyro_offset_y = handle_gyro_offset_y + element_P_pitch(i,1);
    handle_gyro_offset_z = handle_gyro_offset_z + element_Y_yaw(i,1);

    % 初期値の計算
    init_x = init_x + data_sync_Eagleye(i,9);
    init_y = init_y + data_sync_Eagleye(i,10);
    init_z = init_z + data_sync_Eagleye(i,11);
end

% 平均値計算
handle_gyro_offset_x = handle_gyro_offset_x / (handle_Calibration_Time/0.01);
handle_gyro_offset_y = handle_gyro_offset_y / (handle_Calibration_Time/0.01);
handle_gyro_offset_z = handle_gyro_offset_z / (handle_Calibration_Time/0.01);

init_x = init_x / (handle_Calibration_Time/0.01);
init_y = init_y / (handle_Calibration_Time/0.01);
init_z = init_z / (handle_Calibration_Time/0.01);

% オフセット除去
for i = 1:time_SimEpoch
    element_R_roll(i) = element_R_roll(i) - handle_gyro_offset_x;
    element_P_pitch(i) = element_P_pitch(i) - handle_gyro_offset_y;
    element_Y_yaw(i) = element_Y_yaw(i) - handle_gyro_offset_z;
end

%% 姿勢角関連の処理
% 角速度観測値の設定
data_angle_rollrate_obs = element_R_roll;
data_angle_pitchrate_obs = element_P_pitch;
data_angle_roll_Eagleye(:,1) = data_sync_Eagleye(:,124);
data_angle_pitch_Eagleye(:,1) = data_sync_Eagleye(:,115);

% 角速度積算による角度計算
wander_phi_test2 = zeros(time_SimEpoch,1);
wander_theta_test2 = zeros(time_SimEpoch,1);
wander_psi_test2 = zeros(time_SimEpoch,1);
for i = 1:time_SimEpoch
    if i == 1
        wander_phi_test2(i,1) = data_angle_roll_Eagleye(1,1);
        wander_theta_test2(i,1) = data_angle_pitch_Eagleye(1,1);
        wander_psi_test2(i,1) = data_Heading_Eagleye(1,1);
    end

    if i > 1
        wander_phi_test2(i,1) = wander_phi_test2(i-1) + element_R_roll(i) * const_dt2(i);
        wander_theta_test2(i,1) = wander_theta_test2(i-1) + element_P_pitch(i) * const_dt2(i);
        wander_psi_test2(i,1) = wander_psi_test2(i-1,1) + element_Y_yaw(i) * const_dt2(i);
    end
end

%% GNSS方位角の調整
handle_Heading_GNSS = zeros(length(time_Eagleye),1);
handle_Heading_GNSS_control = zeros(time_SimEpoch,1);
for i = 1:time_SimEpoch
    handle_Heading_GNSS_control(i,1) = wander_psi_test2(i,1) - handle_Heading_GNSS(i,1);
    handle_Heading_GNSS(i,1) = handle_Heading_GNSS(i,1);
    
    % 角度補正（2πの倍数単位）
    if handle_Heading_GNSS_control(i,1) < -pi
        handle_Heading_GNSS(i,1) = handle_Heading_GNSS(i,1) - 2*pi;
    end
    if handle_Heading_GNSS_control(i,1) < -3*pi
        handle_Heading_GNSS(i,1) = handle_Heading_GNSS(i,1) - 4*pi;
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
end

%% 加速度オフセット補正とENU変換
% x軸加速度のオフセット補正　
handle_ax_EAGLEYE_offset = data_sync_Eagleye(:,118);
element_ax_EAGLEYE = element_ax_EAGLEYE + handle_ax_EAGLEYE_offset;

% データ整理
data_acc_Eagleye_xyz = [element_ax_EAGLEYE element_ay_EAGLEYE element_az_EAGLEYE];
data_vel_Eagleye_xyz = data_sync_Eagleye(:,2:4);
data_pos_Eagleye_llhrad = [data_sync_Eagleye(:,132)/180*pi data_sync_Eagleye(:,133)/180*pi data_sync_Eagleye(:,134)];
data_pos_Eagleye_xyz = zeros(time_SimEpoch,3);
data_pos_Eagleye_enu = zeros(time_SimEpoch,3);
data_vel_Eagleye_enu = zeros(time_SimEpoch,3);
data_acc_Eagleye_enu = zeros(time_SimEpoch,3);

% 座標変換処理
for i = 1:length(data_pos_Eagleye_llhrad)
    data_pos_Eagleye_xyz(i,:) = llh2xyz(data_pos_Eagleye_llhrad(i,:));
    data_pos_Eagleye_enu(i,:) = xyz2enu(data_pos_Eagleye_xyz(i,:), handle_ORG_XYZ);
    data_vel_Eagleye_enu(i,:) = xyz2enu_vel(data_vel_Eagleye_xyz(i,:), handle_ORG_XYZ);
    data_acc_Eagleye_enu(i,:) = xyz2enu_vel(data_acc_Eagleye_xyz(i,:), handle_ORG_XYZ);
end

% 3D加速度計算
data_acc_Eagleye_enu3d(:,1) = sqrt(data_acc_Eagleye_enu(:,1).^2 +  ...
                                   data_acc_Eagleye_enu(:,2).^2 +  ...
                                   data_acc_Eagleye_enu(:,3).^2);

cd ..