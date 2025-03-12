% PseudoErrorGenerator.m
function [pseudo_error, error_stats] = PseudoErrorGenerator(pos_data, vel_data, ref_data, params)
    % 拡張疑似誤差生成器
    %
    % 入力:
    %   pos_data - 位置データ構造体
    %       .current  - 現在の位置 [E,N,U]
    %       .previous - 前回の位置 [E,N,U]
    %   vel_data - 速度データ構造体
    %       .gnss    - GNSSドップラー速度 [E,N,U]
    %       .imu     - IMU推定速度 [E,N,U]
    %   ref_data - リファレンスデータ構造体
    %   params   - パラメータ構造体
    %
    % 出力:
    %   pseudo_error - 疑似誤差 [E,N,U]
    %   error_stats  - 誤差統計情報
    
    % 時間差分の計算
    pos_diff = pos_data.current - pos_data.previous;
    
    % 速度ベースの予測位置変化量の計算
    weight_gnss = params.weights.gnss;
    weight_imu = params.weights.imu;
    vel_predicted = (weight_gnss * vel_data.gnss + weight_imu * vel_data.imu) ...
        / (weight_gnss + weight_imu);
    
    % 拡張疑似誤差の計算
    pseudo_error = computeEnhancedError(pos_diff, vel_predicted, params);
    
    % 誤差統計情報の計算
    error_stats = computeErrorStatistics(pseudo_error, ref_data, params);
end

function enhanced_error = computeEnhancedError(pos_diff, vel_pred, params)
    % 拡張誤差計算
    % 基本誤差の計算
    basic_error = abs(pos_diff - vel_pred);
    
    % 速度に基づく重み付け
    vel_magnitude = norm(vel_pred);
    if vel_magnitude < params.thresholds.min_velocity
        velocity_weight = 0.5;
    else
        velocity_weight = min(vel_magnitude / params.thresholds.ref_velocity, 1.0);
    end
    
    % 重み付き誤差の計算
    enhanced_error = basic_error * velocity_weight;
    
    % 異常値の抑制
    enhanced_error = suppressOutliers(enhanced_error, params);
end

function error_stats = computeErrorStatistics(error, ref_data, params)
    % 誤差統計情報の計算
    error_stats = struct();
    
    % 基本統計量の計算
    error_stats.mean = mean(error);
    error_stats.std = std(error);
    error_stats.max = max(error);
    
    % リファレンスとの比較
    if ~isempty(ref_data.pos)
        error_stats.ref_diff = error - ref_data.pos;
        error_stats.ref_correlation = corrcoef(error, ref_data.pos);
    end
    
    % 信頼性指標の計算
    error_stats.reliability = computeReliabilityIndex(error, params);
end

function error = suppressOutliers(error, params)
    % 異常値の抑制処理
    for i = 1:3
        % MADによる異常値検出
        mad_value = median(abs(error(:,i) - median(error(:,i))));
        threshold = params.outlier.k * mad_value;
        
        % 異常値の抑制
        outliers = abs(error(:,i) - median(error(:,i))) > threshold;
        error(outliers,i) = sign(error(outliers,i)) .* threshold;
    end
end

function reliability = computeReliabilityIndex(error, params)
    % 信頼性指標の計算
    normalized_error = error / params.thresholds.max_error;
    reliability = exp(-normalized_error.^2 / 2);
end