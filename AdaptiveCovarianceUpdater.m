% AdaptiveCovarianceUpdater.m
function [R_updated, update_info] = AdaptiveCovarianceUpdater(R_current, error_stats, vehicle_state, params)
    % 適応的共分散行列更新器
    %
    % 入力:
    %   R_current     - 現在の観測雑音共分散行列 [7x7]
    %   error_stats   - 誤差統計情報構造体
    %   vehicle_state - 車両状態構造体
    %   params        - 更新パラメータ構造体
    %
    % 出力:
    %   R_updated    - 更新後の共分散行列
    %   update_info  - 更新情報構造体

    % 初期化
    R_updated = R_current;
    update_info = struct();
    
    % 最小分散の設定（数値安定性のため）
    min_variance = struct(...
        'position', 0.01^2, ...    % 位置の最小分散 [m^2]
        'angle', (0.1*pi/180)^2, ... % 角度の最小分散 [rad^2]
        'velocity', 0.01^2);       % 速度の最小分散 [(m/s)^2]

    % 状態に応じたスケーリング係数の計算
    scaling_factors = computeScalingFactors(vehicle_state, error_stats, params);
    
    % 共分散行列の更新
    R_updated = updateCovarianceMatrix(R_current, scaling_factors, min_variance);
    
    % 更新情報の記録
    update_info = recordUpdateInfo(scaling_factors, R_current, R_updated);
end

function scale = computeScalingFactors(vehicle_state, error_stats, params)
    % スケーリング係数の計算
    
    % 基本スケール係数の初期化
    scale = struct(...
        'position', ones(1,3), ...
        'angle', ones(1,3), ...
        'velocity', 1);
    
    % 位置スケール係数の計算
    for i = 1:3
        % 誤差統計に基づく調整
        error_scale = min(max(error_stats.std(i) / params.ref_std, 0.5), 2.0);
        
        % 信頼性による重み付け
        reliability_weight = max(error_stats.reliability(i), 0.5);
        
        scale.position(i) = error_scale / reliability_weight;
    end
    
    % 姿勢角スケール係数の計算
    if vehicle_state.isCurving
        % カーブ中は姿勢角の不確かさを増加
        curve_intensity = abs(vehicle_state.yaw_rate) / params.max_yaw_rate;
        scale.angle = ones(1,3) * (1 + curve_intensity);
    end
    
    % 速度スケール係数の計算
    if vehicle_state.isStopped
        scale.velocity = 0.5;  % 停止中は速度の不確かさを減少
    else
        vel_scale = min(norm(vehicle_state.velocity) / params.ref_velocity, 1.5);
        scale.velocity = vel_scale;
    end
end

function R = updateCovarianceMatrix(R, scale, min_var)
    % 共分散行列の更新
    
    % 位置共分散の更新（E,N,U）
    for i = 1:3
        R(i,i) = max(R(i,i) * scale.position(i), min_var.position);
    end
    
    % 姿勢角共分散の更新（Roll,Pitch,Yaw）
    for i = 4:6
        R(i,i) = max(R(i,i) * scale.angle(i-3), min_var.angle);
    end
    
    % 速度共分散の更新
    R(7,7) = max(R(7,7) * scale.velocity, min_var.velocity);
    
    % 対称性の保証
    R = (R + R')/2;
end

function info = recordUpdateInfo(scale, R_old, R_new)
    % 更新情報の記録
    info = struct(...
        'scaling_factors', scale, ...
        'variance_change', struct(...
            'position', [R_new(1,1)/R_old(1,1), R_new(2,2)/R_old(2,2), R_new(3,3)/R_old(3,3)], ...
            'angle', [R_new(4,4)/R_old(4,4), R_new(5,5)/R_old(5,5), R_new(6,6)/R_old(6,6)], ...
            'velocity', R_new(7,7)/R_old(7,7)));
end