% CurveCompensation.m
function [compensated_pos, compensated_vel, compensation_info] = CurveCompensation(...
    raw_pos, raw_vel, yaw_rate, lateral_acc, dt, params)
    % 曲線走行時の位置・速度補正
    %
    % Inputs:
    %   raw_pos      - [E,N,U] 位置 [m]
    %   raw_vel      - [vE,vN,vU] 速度 [m/s]
    %   yaw_rate     - ヨーレート [rad/s]
    %   lateral_acc  - 横加速度 [m/s^2]
    %   dt           - サンプリング時間 [s]
    %   params       - 車両パラメータ構造体
    %
    % Outputs:
    %   compensated_pos  - 補正後の位置 [m]
    %   compensated_vel  - 補正後の速度 [m/s]
    %   compensation_info - 補正情報の構造体
    
    % Initialize outputs
    compensated_pos = raw_pos;
    compensated_vel = raw_vel;
    compensation_info = struct();
    
    % シエンタのパラメータ
    Lw = 2.69;  % ホイールベース [m]
    h = 0.57;   % 重心高さ [m]
    Tw = 1.49;  % トレッド [m]
    m = 1440;   % 車両質量 [kg]
    g = 9.81;   % 重力加速度 [m/s^2]
    
    % Velocity threshold check
    vel_mag = norm(raw_vel);
    if vel_mag < 0.5
        compensation_info.skip_reason = 'low_velocity';
        return;
    end
    
    % 1. スリップ角の推定
    beta = estimateSlipAngle(raw_vel, yaw_rate, lateral_acc, Lw);
    compensation_info.slip_angle = beta;
    
    % 2. 遠心力による影響の計算
    centripetal_acc = vel_mag * yaw_rate;  % 求心加速度
    roll_angle = estimateRollAngle(centripetal_acc, h, Tw);
    compensation_info.roll_angle = roll_angle;
    
    % 3. バンク角の推定
    bank_angle = estimateBankAngle(lateral_acc, yaw_rate, vel_mag);
    compensation_info.bank_angle = bank_angle;
    
    % 4. 速度の補正
    [compensated_vel, vel_correction] = compensateVelocity(...
        raw_vel, beta, roll_angle, bank_angle);
    compensation_info.velocity_correction = vel_correction;
    
    % 5. 位置の補正（積分）
    compensated_pos = integratePosition(...
        raw_pos, compensated_vel, dt, compensation_info);
end

function beta = estimateSlipAngle(vel, yaw_rate, lat_acc, wheelbase)
    % スリップ角の推定
    vel_mag = norm(vel);
    
    % キネマティックモデルによるスリップ角
    beta_kinematic = atan2(wheelbase * yaw_rate, vel_mag);
    
    % 横加速度からのスリップ角
    beta_dynamic = atan2(lat_acc, 9.81);
    
    % 重み付き平均（高速ではdynamic、低速ではkinematicを重視）
    w = min(vel_mag / 5.0, 1.0);  % 5.0 m/s で完全に切り替わる
    beta = w * beta_dynamic + (1-w) * beta_kinematic;
    
    % リミッター
    beta = max(min(beta, pi/6), -pi/6);  % ±30度に制限
end

function roll = estimateRollAngle(centripetal_acc, cog_height, track_width)
    % ロール角の推定
    roll_stiffness = 0.8;  % ロール剛性係数
    roll_damping = 0.2;    % ロール減衰係数
    
    % シンプルなロールモデル
    roll = (centripetal_acc * cog_height * roll_stiffness) / ...
        (track_width * (1 + roll_damping));
    
    % リミッター
    roll = max(min(roll, pi/12), -pi/12);  % ±15度に制限
end

function bank = estimateBankAngle(lat_acc, yaw_rate, velocity)
    % 路面バンク角の推定
    g = 9.81;
    
    % 横加速度からの推定
    bank_from_acc = asin(lat_acc / g);
    
    % 車両運動の影響を補正
    dynamic_effect = velocity * yaw_rate / g;
    
    % 推定値の合成
    bank = bank_from_acc - dynamic_effect;
    
    % リミッター
    bank = max(min(bank, pi/12), -pi/12);  % ±15度に制限
end

function [comp_vel, correction] = compensateVelocity(...
    raw_vel, slip_angle, roll_angle, bank_angle)
    % 速度ベクトルの補正
    
    % 1. スリップ角による補正
    R_slip = [cos(slip_angle) -sin(slip_angle) 0;
              sin(slip_angle)  cos(slip_angle) 0;
              0               0                1];
    
    % 2. ロール角による補正
    R_roll = [1  0           0;
              0  cos(roll_angle) -sin(roll_angle);
              0  sin(roll_angle)  cos(roll_angle)];
    
    % 3. バンク角による補正
    R_bank = [cos(bank_angle) 0 -sin(bank_angle);
              0               1  0;
              sin(bank_angle) 0  cos(bank_angle)];
    
    % 補正の適用
    comp_vel = (R_bank * R_roll * R_slip * raw_vel')';
    correction = comp_vel - raw_vel;
end

function pos = integratePosition(raw_pos, vel, dt, info)
    % 補正速度による位置の積分
    pos = raw_pos + vel * dt;
end