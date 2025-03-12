function visualizeVelocityComparison(ref_pos, ref_vel, comp_pos, comp_vel, dt, plot_type)
    % East-North平面における2次元速度ベクトル比較分析
    % 入力:
    %   ref_pos: リファレンス位置 [East North Up] (Nx3行列)
    %   ref_vel: リファレンス速度 [VE VN VU] (Nx3) または速度の大きさ (Nx1)
    %   comp_pos: 比較対象位置 [East North Up] (Nx3行列)
    %   comp_vel: 比較対象速度 [VE VN VU] (Nx3) または速度の大きさ (Nx1)
    %   dt: 時間ステップ [s] (スカラー)
    %   plot_type: プロットタイプを指定する文字列
    %   ('histogram', 'scatter', または 'compass')
    
    % 入力値の検証
    validateattributes(ref_pos, {'numeric'}, {'2d', 'ncols', 3});
    validateattributes(comp_pos, {'numeric'}, {'2d', 'ncols', 3});
    validateattributes(dt, {'numeric'}, {'scalar', 'positive'});
    validateattributes(plot_type, {'char', 'string'}, {});
    
    % 入力サイズに基づいて速度データを2次元ベクトル（East-North平面）に変換
    ref_vel_vec = convertToVelocityVector2D(ref_pos, ref_vel, dt);
    comp_vel_vec = convertToVelocityVector2D(comp_pos, comp_vel, dt);
    
    % 角度と大きさの比を計算
    [angles, magnitudes] = calculateVectorDifferences(ref_vel_vec, comp_vel_vec);
    
    % 可視化の作成
    figure;
    switch lower(plot_type)
        case 'histogram'
            polarhistogram(angles, 36, 'Normalization', 'probability');
            title('速度方向差の分布 (E-N平面)');
            
        case 'scatter'
            polarscatter(angles, magnitudes, 25, 'filled');
            title('速度の大きさの比と方向差 (E-N平面)');
            
        case 'compass'
            % 見やすさのためにサンプリング
            sample_idx = 1:10:length(angles);
            compass(angles(sample_idx), magnitudes(sample_idx));
            compass(angles, magnitudes);
            title('速度ベクトル差 (E-N平面)');
            
        otherwise
            error('無効なプロットタイプです。''histogram'', ''scatter'', または ''compass'' を使用してください');
    end
end

function vel_vec = convertToVelocityVector2D(pos, vel, dt)
    % 速度入力を2次元ベクトル（East-North平面）に変換
    if size(vel, 2) == 3
        % East-North成分のみを抽出
        vel_vec = vel(:,1:2);
    elseif size(vel, 2) == 1
        % スカラー量から2次元方向を計算
        vel_vec = zeros(length(vel), 2);
        
        % 位置から方向ベクトルを計算（中心差分法使用）
        for i = 2:(size(pos,1)-1)
            % 位置から方向ベクトルを計算（East-Northのみ）
            dir_vec = (pos(i+1,1:2) - pos(i-1,1:2)) / (2*dt);
            dir_vec = dir_vec / norm(dir_vec);  % 正規化
            
            % 方向と大きさを組み合わせる
            vel_vec(i,:) = dir_vec * vel(i);
        end
        
        % 端点を前方/後方差分で処理
        dir_vec_start = (pos(2,1:2) - pos(1,1:2)) / dt;
        dir_vec_start = dir_vec_start / norm(dir_vec_start);
        vel_vec(1,:) = dir_vec_start * vel(1);
        
        dir_vec_end = (pos(end,1:2) - pos(end-1,1:2)) / dt;
        dir_vec_end = dir_vec_end / norm(dir_vec_end);
        vel_vec(end,:) = dir_vec_end * vel(end);
    else
        error('速度入力はNx3（ベクトル）またはNx1（大きさ）である必要があります');
    end
end

function [angles, magnitudes] = calculateVectorDifferences(ref_vel, comp_vel)
    % リファレンスと比較対象ベクトル間の角度を計算
    ref_angles = atan2(ref_vel(:,2), ref_vel(:,1));    % North/East
    comp_angles = atan2(comp_vel(:,2), comp_vel(:,1));
    angles = wrapToPi(comp_angles - ref_angles);        % [-pi, pi]に収める
    
    % 大きさの比を計算
    ref_mags = vecnorm(ref_vel, 2, 2);    % 2次元ノルム
    comp_mags = vecnorm(comp_vel, 2, 2);
    magnitudes = comp_mags ./ ref_mags;
end

% 使用例:
% ケース1: ベクトル速度 (Nx3)
% ref_pos = [E1 N1 U1; E2 N2 U2; ...];   % リファレンスENU位置
% ref_vel = [VE1 VN1 VU1; VE2 VN2 VU2; ...];   % リファレンス速度
% comp_pos = [E1 N1 U1; E2 N2 U2; ...];   % 比較対象位置
% comp_vel = [VE1 VN1 VU1; VE2 VN2 VU2; ...];  % 比較対象速度
%
% ケース2: スカラー速度 (Nx1)
% ref_vel = [V1; V2; ...];   % リファレンス速度の大きさ
% comp_vel = [V1; V2; ...];  % 比較対象速度の大きさ
%
% dt = 0.2;  % サンプリング周期 5Hz
% visualizeVelocityComparison(ref_pos, ref_vel, comp_pos, comp_vel, dt, 'scatter');