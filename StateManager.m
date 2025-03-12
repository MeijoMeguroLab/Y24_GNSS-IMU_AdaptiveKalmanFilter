% StateManager.m
function [vehicle_state, update_flags] = StateManager(current_data, prev_state, params)
    % 車両状態管理関数
    % 入力:
    %   current_data: 現在のセンサデータ
    %   prev_state: 前回の状態（初回はempty）
    %   params: パラメータ構造体
    
    % 初期状態の設定
    if isempty(prev_state)
        vehicle_state = struct(...
            'isCurving', false, ...        % カーブ状態
            'isStopped', true, ...         % 停止状態
            'isHighLatAcc', false, ...     % 高横加速度状態
            'yaw_rate', 0, ...             % ヨーレート
            'velocity', zeros(1,3), ...     % 速度
            'confidence', 1.0);            % 状態推定の信頼度
        update_flags = struct(...
            'curveStart', false, ...       % カーブ開始
            'curveEnd', false, ...         % カーブ終了
            'stopStart', false, ...        % 停止開始
            'stopEnd', false);             % 停止終了
        return;
    end
    
    % 状態の更新
    vehicle_state = prev_state;
    
    % 基本状態の更新
    new_isCurving = abs(current_data.yaw_rate) > params.thresholds.curving;
    new_isStopped = norm(current_data.velocity) < params.thresholds.stopping;
    new_isHighLatAcc = abs(current_data.lateral_acc) > params.thresholds.high_lat_acc;
    
    % 状態遷移フラグの設定
    update_flags = struct(...
        'curveStart', ~prev_state.isCurving && new_isCurving, ...
        'curveEnd', prev_state.isCurving && ~new_isCurving, ...
        'stopStart', ~prev_state.isStopped && new_isStopped, ...
        'stopEnd', prev_state.isStopped && ~new_isStopped);
    
    % 状態の更新
    vehicle_state.isCurving = new_isCurving;
    vehicle_state.isStopped = new_isStopped;
    vehicle_state.isHighLatAcc = new_isHighLatAcc;
    vehicle_state.yaw_rate = current_data.yaw_rate;
    vehicle_state.velocity = current_data.velocity;
    
    % 信頼度の計算
    vehicle_state.confidence = computeConfidence(vehicle_state, current_data, params);
end

function confidence = computeConfidence(state, data, params)
    % 基本信頼度
    confidence = 1.0;
    
    % 高横加速度時の信頼度低下
    if state.isHighLatAcc
        confidence = confidence * 0.8;
    end
    
    % 低速時の信頼度調整
    if norm(state.velocity) < 2 * params.thresholds.stopping
        vel_factor = max(0.5, norm(state.velocity) / (2 * params.thresholds.stopping));
        confidence = confidence * vel_factor;
    end
    
    % 最終的な信頼度を制限
    confidence = max(min(confidence, 1.0), 0.3);
end