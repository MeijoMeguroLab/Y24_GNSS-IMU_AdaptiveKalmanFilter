function [orthogonal_error, longitudinal_error, angle, velocity_ref, velocity_est] = ...
    calculateTrajectoryErrors(ref_positions, est_positions, timestamps)
% リファレンス軌跡と推定軌跡間の誤差を計算する関数
%
% 入力:
%   ref_positions: Nx3行列 [East, North, Up] - リファレンス位置（ENU座標系）
%   est_positions: Nx3行列 [East, North, Up] - 推定位置（ENU座標系）
%   timestamps: Nx1ベクトル - 開始時刻からの経過時間 [秒]
%
% 出力:
%   orthogonal_error: Nx1ベクトル - 直交方向誤差 [m]（負値はカーブの内側）
%   longitudinal_error: Nx1ベクトル - 進行方向誤差 [m]（正値は進行方向への遅れ）
%   angle: Nx1ベクトル - 基準線からの角度 [rad]（-π/2からπ/2の範囲）
%   velocity_ref: Nx1ベクトル - リファレンスの速度 [m/s]
%   velocity_est: Nx1ベクトル - 推定値の速度 [m/s]

    % 入力データの検証
    if ~validateInputs(ref_positions, est_positions, timestamps)
        error('入力データが無効です');
    end
    
    % 出力配列の初期化
    N = size(ref_positions, 1);
    orthogonal_error = zeros(N, 1);
    longitudinal_error = zeros(N, 1);
    angle = zeros(N, 1);
    velocity_ref = zeros(N, 1);
    velocity_est = zeros(N, 1);
    
    % 全点での誤差計算
    for k = 1:N
        % 基準線の端点を取得
        [baseline_start, baseline_end] = getBaselinePoints(ref_positions, k);
        
        % 垂線との交点を計算
        [ref_intersect, est_intersect] = calculateIntersectionPoints(...
            baseline_start, baseline_end, ref_positions(k,:), est_positions(k,:));
        
        % 誤差と角度を計算
        [orthogonal_error(k), longitudinal_error(k), angle(k)] = calculateErrors(...
            baseline_start, baseline_end, ref_intersect, est_intersect, ...
            ref_positions(k,:), est_positions(k,:));
    end
    
    % 速度の計算
    [velocity_ref, velocity_est] = calculateVelocities(ref_positions, est_positions, timestamps);
end

function valid = validateInputs(ref_pos, est_pos, times)
% 入力データの形式と値を検証する関数
%
% 検証に失敗した場合はfalse、成功した場合はtrueを返す

    valid = true;
    
    % 配列サイズの整合性チェック
    if size(ref_pos,1) ~= size(est_pos,1) || size(ref_pos,1) ~= length(times)
        valid = false;
        disp('エラー: 入力データのサイズが一致しません');
        return;
    end
    
    % 最小点数のチェック
    if size(ref_pos,1) < 2
        valid = false;
        disp('エラー: 少なくとも2点以上のデータが必要です');
        return;
    end
    
    % NaN/Infのチェック
    if any(isnan([ref_pos(:); est_pos(:); times])) || ...
       any(isinf([ref_pos(:); est_pos(:); times]))
        valid = false;
        disp('エラー: 入力データにNaNまたはInfが含まれています');
        return;
    end
end

function [baseline_start, baseline_end] = getBaselinePoints(ref_positions, k)
% 誤差計算用の基準線を定義する2点を取得する関数
%
% 端点の特殊処理:
% - k=1の場合: 1点目と2点目を使用
% - k=Nの場合: N-1点目とN点目を使用
% - それ以外: k-1点目とk+1点目を使用

    N = size(ref_positions, 1);
    if k == 1
        baseline_start = ref_positions(1,:);
        baseline_end = ref_positions(2,:);
    elseif k == N
        baseline_start = ref_positions(N-1,:);
        baseline_end = ref_positions(N,:);
    else
        baseline_start = ref_positions(k-1,:);
        baseline_end = ref_positions(k+1,:);
    end
end

function [ref_intersect, est_intersect] = calculateIntersectionPoints(baseline_start, baseline_end, ref_point, est_point)
% リファレンス点と推定点から基準線への垂線の交点を計算する関数
%
% ベクトル射影を使用して、リファレンス点と推定点からの
% 垂線が基準線と交わる点を計算する

    % 基準線のベクトルを計算
    baseline_vector = baseline_end - baseline_start;
    
    % リファレンス点の射影を計算
    ref_v = ref_point - baseline_start;
    ref_proj = dot(ref_v, baseline_vector) / dot(baseline_vector, baseline_vector);
    ref_intersect = baseline_start + ref_proj * baseline_vector;
    
    % 推定点の射影を計算
    est_v = est_point - baseline_start;
    est_proj = dot(est_v, baseline_vector) / dot(baseline_vector, baseline_vector);
    est_intersect = baseline_start + est_proj * baseline_vector;
end

function [orth_err, long_err, ang] = calculateErrors(baseline_start, baseline_end, ref_intersect, est_intersect, ref_point, est_point)
% 直交誤差、進行方向誤差、角度を計算する関数
%
% 直交誤差: 垂線の交点間の距離
% 進行方向誤差: 基準線に沿った交点間の距離
% 角度: 基準線の方向と推定位置の方向の間の角度

    % 直交誤差の計算
    ref_orth_dist = norm(ref_point - ref_intersect);
    est_orth_dist = norm(est_point - est_intersect);
    
    % 直交誤差の符号を決定（カーブの内側が負）
    baseline_vector = baseline_end - baseline_start;
    cross_prod = cross(baseline_vector, est_point - est_intersect);
    orth_err = est_orth_dist - ref_orth_dist;
    if cross_prod(3) < 0  % ENU座標系を前提
        orth_err = -orth_err;
    end
    
    % 進行方向誤差の計算
    long_err = norm(est_intersect - ref_intersect);
    % 方向に基づいて符号を設定
    if dot(est_intersect - ref_intersect, baseline_vector) < 0
        long_err = -long_err;
    end
    
    % 角度の計算
    baseline_vector = baseline_end - baseline_start;
    baseline_vector = baseline_vector / norm(baseline_vector);

    % リファレンスの垂線交点から推定点へのベクトル
    error_vector = est_point - ref_intersect;
    if norm(error_vector) > 0
        error_vector = error_vector / norm(error_vector);

        % 2次元ベクトルの外積は符号付き面積として計算
        cross_z = baseline_vector(1) * error_vector(2) - baseline_vector(2) * error_vector(1);

        % ベクトル間の角度を計算（時計回りを正とする）
        ang = atan2(cross_z, dot(baseline_vector, error_vector));

        % 角度の範囲を-π/2からπ/2に制限
        if ang > pi/2
            ang = ang - pi;
        elseif ang < -pi/2
            ang = ang + pi;
        end
    else
        ang = 0;
    end
end

function [velocity_ref, velocity_est] = calculateVelocities(ref_positions, est_positions, timestamps)
% リファレンスと推定軌跡の速度を計算する関数
%
% 連続する点間の距離と時間から速度を計算
% 1点目の速度は2点目と同じ値を使用

    N = size(ref_positions, 1);
    velocity_ref = zeros(N, 1);
    velocity_est = zeros(N, 1);
    
    % 2点目からN点目までの速度を計算
    for i = 2:N
        dt = timestamps(i) - timestamps(i-1);
        
        % リファレンスの速度計算
        dist_ref = norm(ref_positions(i,:) - ref_positions(i-1,:));
        velocity_ref(i) = dist_ref / dt;
        
        % 推定値の速度計算
        dist_est = norm(est_positions(i,:) - est_positions(i-1,:));
        velocity_est(i) = dist_est / dt;
    end
    
    % 1点目の速度を2点目と同じ値に設定
    velocity_ref(1) = velocity_ref(2);
    velocity_est(1) = velocity_est(2);
end