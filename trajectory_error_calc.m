% 時間軸の設定
t = time_Obs;

x_ref = data_pos_Ref_enu(:,1);
y_ref = data_pos_Ref_enu(:,2);
z_ref = data_pos_Ref_enu(:,3);

x_est = data_pos_Obs_enu(:,1);
y_est = data_pos_Obs_enu(:,2);
z_est = data_pos_Obs_enu(:,3);

% 入力データの準備
ref_positions = data_pos_Ref_enu;
est_positions = data_pos_Obs_enu;
timestamps = t;

%% 誤差計算
[orthogonal_error, longitudinal_error, angle, velocity_ref, velocity_est] = ...
    calculateTrajectoryErrors(ref_positions, est_positions, timestamps);

%% 結果のプロット
% Figure 1: 軌跡の比較
figure('Name', '軌跡の比較');
subplot(2,1,1)
plot(x_ref, y_ref, 'r.', 'MarkerSize', 12, 'DisplayName', 'リファレンス');
hold on;
plot(x_est, y_est, 'b.', 'MarkerSize', 6, 'DisplayName', '推定値');
% 選択した点での誤差を可視化（10点おき）
for i = 1:10:length(t)
    % 基準線の端点を取得
    [baseline_start, baseline_end] = getBaselinePoints(ref_positions, i);
    % 基準線を描画
    plot([baseline_start(1), baseline_end(1)], [baseline_start(2), baseline_end(2)], ...
         'g--', 'LineWidth', 0.5);
    % 垂線を描画
    [ref_intersect, est_intersect] = calculateIntersectionPoints(...
        baseline_start, baseline_end, ref_positions(i,:), est_positions(i,:));
    plot([ref_positions(i,1), ref_intersect(1)], ...
         [ref_positions(i,2), ref_intersect(2)], 'k:', 'LineWidth', 0.5);
    plot([est_positions(i,1), est_intersect(1)], ...
         [est_positions(i,2), est_intersect(2)], 'k:', 'LineWidth', 0.5);
end
grid on;
xlabel('East [m]');
ylabel('North [m]');
title('軌跡比較と誤差の可視化');
axis equal;

% 3D表示
subplot(2,1,2)
plot3(x_ref, y_ref, z_ref, 'r.', 'MarkerSize', 12, 'DisplayName', 'リファレンス');
hold on;
plot3(x_est, y_est, z_est, 'b.', 'MarkerSize', 9, 'DisplayName', '推定値');
grid on;
xlabel('East [m]');
ylabel('North [m]');
zlabel('Up [m]');
title('3D軌跡比較');
legend('Location', 'best');
axis equal;

% Figure 2: 誤差と速度の時系列プロット
figure('Name', '誤差と速度の時系列');
subplot(4,1,1)
plot(t, orthogonal_error, 'b.');
grid on;
xlabel('時間 [s]');
ylabel('直交誤差 [m]');
title('直交方向誤差');

subplot(4,1,2)
plot(t, longitudinal_error, 'r.');
grid on;
xlabel('時間 [s]');
ylabel('進行方向誤差 [m]');
title('進行方向誤差');

subplot(4,1,3)
plot(t, rad2deg(angle), 'g-');  % ラジアンから度に変換
grid on;
xlabel('時間 [s]');
ylabel('角度 [deg]');
ylim([-90 90])
title('基準線からの角度');

subplot(4,1,4)
plot(t, velocity_ref, 'r.', 'MarkerSize', 12, 'DisplayName', 'リファレンス');
hold on;
plot(t, velocity_est, 'b.', 'MarkerSize', 6, 'DisplayName', '推定値');
grid on;
xlabel('時間 [s]');
ylabel('速度 [m/s]');
title('速度比較');
legend('Location', 'best');

% 基本的な統計情報の表示
fprintf('===== 誤差統計 =====\n');
% 統計情報の表示
fprintf('\n=== 誤差統計 ===\n');
fprintf('直交方向誤差 RMS: %.3f m\n', rms(orthogonal_error(~isnan(orthogonal_error))));
fprintf('進行方向誤差 RMS: %.3f m\n', rms(longitudinal_error(~isnan(longitudinal_error))));
fprintf('角度誤差 RMS: %.3f deg\n', rms(rad2deg(angle(~isnan(angle)))));
fprintf('速度誤差 RMS: %.3f m/s\n', rms(velocity_ref(~isnan(velocity_ref)) - velocity_est(~isnan(velocity_est))));



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
    direction_vector = est_point - est_intersect;
    ang = atan2(norm(cross(baseline_vector, direction_vector)), dot(baseline_vector, direction_vector));
    % 角度の範囲を-π/2からπ/2に制限
    if ang > pi/2
        ang = pi - ang;
    elseif ang < -pi/2
        ang = -pi - ang;
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