classdef ImprovedAdaptiveKF < handle
    properties
        % 状態推定パラメータ
        windowSize = 50;
        minCovThreshold = 0.01;
        
        % 車両状態検出
        isCurving = false;
        isStopped = false;
        curvingThreshold = 0.1; % ヨーレート閾値 [rad/s]
        stoppingThreshold = 0.5; % 速度閾値 [m/s]
        
        % 各状態での適応重み係数
        straightWeights = struct('gnss', 0.67, 'imu', 0.33);
        curveWeights = struct('gnss', 0.4, 'imu', 0.6);
        stopWeights = struct('gnss', 0.8, 'imu', 0.2);
        
        % 履歴保存用
        pseudoErrorHistory = [];
        currentGNSSQuality = 'FLOAT'; % デフォルト値
    end
    
    methods
        function updateVehicleState(obj, yawRate, velocity)
            % センサー値に基づく車両状態更新
            obj.isCurving = abs(yawRate) > obj.curvingThreshold;
            obj.isStopped = norm(velocity) < obj.stoppingThreshold;
        end
        
        function weights = getAdaptiveWeights(obj, gnssQuality)
            % 車両状態とGNSS品質に基づく適応重み取得
            if obj.isStopped
                weights = obj.stopWeights;
            elseif obj.isCurving
                weights = obj.curveWeights;
            else
                weights = obj.straightWeights;
            end
            
            % GNSS品質に基づく重み調整
            if strcmp(gnssQuality, 'FIX')
                weights.gnss = weights.gnss * 1.2;
                weights.imu = 1 - weights.gnss;
            end
        end
        
        function setGNSSQuality(obj, quality)
            % GNSS品質を設定
            obj.currentGNSSQuality = quality;
        end
        
        function quality = getGNSSQuality(obj)
            % 現在のGNSS品質を取得
            quality = obj.currentGNSSQuality;
        end
        
        function pseudoError = generatePseudoError(obj, currPos, prevPos, velocity, dt)
            % 状態認識型の拡張疑似誤差生成
            weights = obj.getAdaptiveWeights(obj.getGNSSQuality());
            
            % IMUベース速度に拡張誤差補正適用
            imuVel = obj.compensateIMUErrors(velocity.imu);
            
            % 品質考慮したGNSSベース速度
            gnssVel = obj.processGNSSVelocity(velocity.gnss);
            
            % 複合速度の計算
            compositeVel = weights.gnss * gnssVel + weights.imu * imuVel;
            
            % 状態に応じた疑似誤差生成
            if obj.isStopped
                pseudoError = obj.generateStopError(currPos, prevPos);
            elseif obj.isCurving
                pseudoError = obj.generateCurveError(currPos, prevPos, compositeVel, dt);
            else
                pseudoError = obj.generateStraightError(currPos, prevPos, compositeVel, dt);
            end
            
            % 履歴に追加
            obj.pseudoErrorHistory = [obj.pseudoErrorHistory; pseudoError];
        end
        
        function imuVel = compensateIMUErrors(obj, imuVel)
            % IMU積分誤差補正
            if obj.isCurving
                % カーブ用拡張誤差モデル適用
                imuVel = obj.applyCurveErrorModel(imuVel);
            end
            % 温度補正適用
            imuVel = obj.applyTempCompensation(imuVel);
        end
        
        function gnssVel = processGNSSVelocity(obj, gnssVel)
            % GNSS速度処理（単純な実装）
            gnssVel = gnssVel;
        end
        
        function vel = applyCurveErrorModel(obj, vel)
            % カーブ誤差モデル適用（単純な実装）
            % 実際のモデルはより複雑であるべき
            curveCorrection = 1.05; % 5%補正
            vel = vel * curveCorrection;
        end
        
        function vel = applyTempCompensation(obj, vel)
            % 温度補正適用（単純な実装）
            % 実際の補正はセンサー温度に依存
            vel = vel;
        end
        
        function error = generateStopError(obj, currPos, prevPos)
            % 停止時の疑似誤差生成
            % 停止時は位置変化がほぼないはず
            error = norm(currPos - prevPos);
        end
        
        function error = generateCurveError(obj, currPos, prevPos, velocity, dt)
            % カーブ区間の疑似誤差生成
            % 予測位置と実際の位置の差
            expectedPos = prevPos + velocity * dt;
            error = norm(currPos - expectedPos);
        end
        
        function error = generateStraightError(obj, currPos, prevPos, velocity, dt)
            % 直線区間の疑似誤差生成
            expectedPos = prevPos + velocity * dt;
            error = norm(currPos - expectedPos);
        end
        
        function R = updateObservationCovariance(obj, pseudoErrors)
            % 拡張適応型の観測共分散更新
            if obj.isStopped
                R = obj.updateStopCovariance(pseudoErrors);
            elseif obj.isCurving
                R = obj.updateCurveCovariance(pseudoErrors);
            else
                R = obj.updateStraightCovariance(pseudoErrors);
            end
            
            % 数値安定性確保
            R = max(R, obj.minCovThreshold);
        end
        
        function R = updateCurveCovariance(obj, pseudoErrors)
            % カーブ区間用の拡張共分散更新
            windowErrors = pseudoErrors(max(1, end-obj.windowSize+1):end);
            
            % カーブ区間のロバスト統計使用
            madValue = mad(windowErrors, 1); % 中央絶対偏差
            R = (madValue * 1.4826)^2; % MADを標準偏差近似に変換
            
            % カーブ強度に基づく共分散増加
            if obj.isCurving
                curveIntensity = obj.calculateCurveIntensity();
                R = R * (1 + curveIntensity);
            end
        end
        
        function R = updateStopCovariance(obj, pseudoErrors)
            % 停止時の共分散更新
            windowErrors = pseudoErrors(max(1, end-obj.windowSize+1):end);
            R = var(windowErrors) * 0.8; % 停止時は信頼性高め
        end
        
        function R = updateStraightCovariance(obj, pseudoErrors)
            % 直線時の共分散更新
            windowErrors = pseudoErrors(max(1, end-obj.windowSize+1):end);
            R = var(windowErrors);
        end
        
        function intensity = calculateCurveIntensity(obj)
            % ヨーレートと速度に基づくカーブ強度計算
            % （外部から値を取得する必要があるため、ダミー実装）
            intensity = 0.5; % ダミー値、実際は現在のヨーレートと速度から計算
        end
    end
end