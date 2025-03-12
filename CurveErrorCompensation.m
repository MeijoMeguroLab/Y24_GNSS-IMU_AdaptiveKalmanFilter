classdef CurveErrorCompensation < handle
    properties
        % 車両パラメータ
        wheelBase = 2.7;          % 車両ホイールベース [m]
        cog_height = 0.5;         % 重心高さ [m]
        track_width = 1.5;        % 車両トレッド幅 [m]
        
        % モデルパラメータ
        slip_coefficient = 0.15;   % タイヤスリップ係数
        bank_angle_max = 0.05;    % 最大路面バンク角 [rad]
        
        % スリップ角推定用カルマンフィルタ
        slip_kf = struct('P', 0.1, 'Q', 0.01, 'R', 0.1);
    end
    
    methods
        function [compensated_vel, beta] = compensateCurveErrors(obj, raw_vel, yaw_rate, lateral_acc)
            % 総合的なカーブエラー補正
            
            % 1. スリップ角推定（キネマティックモデル使用）
            beta = obj.estimateSlipAngle(raw_vel, yaw_rate, lateral_acc);
            
            % 2. 遠心加速度補正
            centripetal_acc = obj.compensateCentripetalAcc(raw_vel, yaw_rate);
            
            % 3. 路面バンク角推定と補正
            bank_angle = obj.estimateRoadBank(lateral_acc, yaw_rate, raw_vel);
            
            % 4. ロール動特性補正
            roll_effect = obj.compensateRollDynamics(lateral_acc);
            
            % 全ての補正を組み合わせる
            compensated_vel = obj.combineCompensations(raw_vel, beta, ...
                centripetal_acc, bank_angle, roll_effect);
        end
        
        function beta = estimateSlipAngle(obj, velocity, yaw_rate, lat_acc)
            % キネマティックモデルとKFを使用した拡張スリップ角推定
            
            % キネマティックスリップ角推定
            beta_kinematic = atan2(obj.wheelBase * yaw_rate, velocity);
            
            % 横加速度を使用した動的スリップ角補正
            beta_dynamic = atan2(lat_acc, 9.81);
            
            % カルマンフィルタ更新
            obj.slip_kf.P = obj.slip_kf.P + obj.slip_kf.Q;
            K = obj.slip_kf.P / (obj.slip_kf.P + obj.slip_kf.R);
            
            % キネマティックと動的推定値の結合
            beta = beta_kinematic + K * (beta_dynamic - beta_kinematic);
            
            % KF状態更新
            obj.slip_kf.P = (1 - K) * obj.slip_kf.P;
        end
        
        function acc_comp = compensateCentripetalAcc(obj, velocity, yaw_rate)
            % 遠心加速度効果の補正
            if abs(yaw_rate) < 0.001
                % ゼロ割り防止
                acc_comp = 0;
                return;
            end
            
            radius = abs(velocity / yaw_rate);
            acc_comp = velocity^2 / radius * sign(yaw_rate);
        end
        
        function bank = estimateRoadBank(obj, lat_acc, yaw_rate, velocity)
            % センサーフュージョンを使用した路面バンク角推定
            g = 9.81;  % 重力加速度 [m/s²]
            
            % 横加速度からの基本バンク角
            bank_from_acc = asin(lat_acc / g);
            
            % 車両ダイナミクス補正
            dynamic_effect = velocity * yaw_rate / g;
            
            % 推定値の組み合わせと制限
            bank = min(max(bank_from_acc - dynamic_effect, ...
                -obj.bank_angle_max), obj.bank_angle_max);
        end
        
        function roll = compensateRollDynamics(obj, lat_acc)
            % 車両ロール動特性の補正
            roll_stiffness = 0.8;  % ロール剛性係数
            roll_damping = 0.2;    % ロール減衰係数
            
            % 単純なロールモデル
            roll = (lat_acc * obj.cog_height * roll_stiffness) / ...
                (obj.track_width * (1 + roll_damping));
        end
        
        function vel_comp = combineCompensations(obj, raw_vel, beta, ...
                centripetal_acc, bank_angle, roll_effect)
            % 全ての補正効果を組み合わせる
            
            % 車両座標系での速度補正
            vel_veh = [raw_vel * cos(beta); raw_vel * sin(beta)];
            
            % バンク角補正の適用
            R_bank = [cos(bank_angle) -sin(bank_angle);
                     sin(bank_angle)  cos(bank_angle)];
            vel_bank = R_bank * vel_veh;
            
            % ロール動特性補正の適用
            roll_correction = [1 -roll_effect;
                             roll_effect 1];
            vel_roll = roll_correction * vel_bank;
            
            % 遠心加速度補正
            vel_comp = norm([vel_roll(1) - centripetal_acc * sin(beta);
                          vel_roll(2) + centripetal_acc * cos(beta)]);
        end
        
        % カーブ区間の位置予測補正
        function [dx, dy] = predictCurvePosition(obj, velocity, yaw, yaw_rate, dt)
            % カーブ走行時の位置予測（スリップ角考慮）
            beta = obj.estimateSlipAngle(velocity, yaw_rate, velocity * yaw_rate);
            
            if abs(yaw_rate) < 0.001
                % 直線区間の場合
                dx = velocity * cos(yaw + beta) * dt;
                dy = velocity * sin(yaw + beta) * dt;
            else
                % カーブ区間の場合
                dx = -velocity * (yaw_rate^(-1)) * (-cos(yaw + yaw_rate * dt + beta) + cos(yaw + beta));
                dy = -velocity * (yaw_rate^(-1)) * (sin(yaw + yaw_rate * dt + beta) - sin(yaw + beta));
                
                % 遠心力補正
                centripetal_acc = velocity^2 * yaw_rate / max(abs(velocity), 0.1);
                
                % 位置補正に遠心力効果を加味
                dx = dx - centripetal_acc * sin(beta) * dt^2/2;
                dy = dy + centripetal_acc * cos(beta) * dt^2/2;
            end
        end
    end
end