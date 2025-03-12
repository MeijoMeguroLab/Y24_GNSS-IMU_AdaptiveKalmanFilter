clear
clc
close all

%% what
what_analyze = 1; % 0:GNSSonly 1:GNSS/IMU

%% edit option set
% velocity
option_velocity_type = 0; % 0:IMUvelocity 1:GNSSdoppler
% 観測誤差共分散
option_Pitch_type = 0; %ピッチ角の推定を加速度と角速度から求める場合
% option_Pitch_type = 1: %ピッチ角をGNSSのドップラー速度から求める場合
% ?
option_calc_type = 0;

%アンテナ位置の補正
handle_ant0_pos = [0 0 0];
handle_ant4_pos = [0.478 -0.418 -1.114];
handle_AntVec   = handle_ant4_pos; clear -regexp ^handle_ant

% movefile('myfile.m','d:/work');

%% A.Preprocessing
cd A.Preprocessing
% if what_analyze == 0; run('A_024gE_Preprocessing.m');
% else; run('A_024gIE_Preprocessing.m');
% end
run('A_024gIE_Preprocessing.m');
%% B.ParameterSettings
cd B.ParameterSettings
run('B_024gIE_ParameterSettings.m');

%% C.EKF
cd C.EKF
load('20241226.mat')
what_analyze = 0
if what_analyze == 0; run('C_024gIE_EKF.m');
else; run('C_024gIE_EKF_adaptive.m');
end

%%
cd C.EKF
%%
load('20241231.mat')
handle_POE = 2;
case2_nums_A = 2;
case2_nums_B = 1;
handle_POEnum_A = case2_nums_A;
handle_POEnum_B = case2_nums_B;
run('C_024gIE_EKF_adaptive.m');
% run('C_024gIE_AKF.m');
% run('C_024gIE_TKF_adaptive.m');
% run('C_024gIE_TAKF.m');
cd C.EKF
title_str = ['1次関数　A:' num2str(handle_POEnum_A) ' B:' num2str(handle_POEnum_B)];
% run('plot01.m')
run('plot02.m')

%%
% インデックス947:960に対して誤差楕円を描画
figure;
hold on;
grid on;
axis equal;

est_pos = KF_enu;
% P行列の作成（対角共分散行列）
P_matrix = cell(length(status_dispersion_param_obs_noise_gnssX), 1);

for i = 1:length(status_dispersion_param_obs_noise_gnssX)
    % 2×2の対角行列を作成
    P_matrix{i} = [status_dispersion_param_obs_noise_gnssX(i), 0;
                   0, status_dispersion_param_obs_noise_gnssY(i)];
end
aria1 = 947:960;
for i = aria1
    plot(data_pos_Ref_enu(i,1),data_pos_Ref_enu(i,2),'.r','MarkerSize',12)
    % デフォルトの95%信頼区間と100ポイントを使用する場合
    plotErrorEllipse(est_pos(i,1), est_pos(i,2), P_matrix{i}(1:2,1:2));
    
    % 信頼区間とポイント数を指定する場合
    % plotErrorEllipse(est_pos(i,1), est_pos(i,2), P_matrix{i}(1:2,1:2), ...
    %                  'confidence', 0.99, 'points', 200);
end

xlabel('East [m]');
ylabel('North [m]');
title('Trajectory with Error Ellipses');
legend({'reference' 'Error Ellipses' 'AKF'},'FontName','ＭＳ 明朝','fontsize',9)




%%

% pseudo_observation_error POE
handle_POE = 0;
handle_POEnum = 0;
standard_array = 0:0.1:10;
for handle_POE = 1:4
    cd C.EKF
    % clear
    load('20241226.mat')
    standard_array = 0:0.1:10;
    switch handle_POE
        case 0 % non

        case 1 % 閾値
            disp('閾値')
            for k = 1:6
                case1_nums = [0.1 0.3 0.6 1 1.5 2.1];
                handle_POEnum = case1_nums(k);
                run('C_024gIE_EKF_adaptive.m');
                cd C.EKF
                title_str = ['閾値 ' num2str(handle_POEnum)];
                run('plot01.m')
                figure
                grid on
                hold on
                plot(standard_array,handle_POEnum,'.r')
                title(['閾値　' num2str(handle_POEnum)])
                xlim([0 10])
                ylim([0 10])
                xlabel('error [m]')
                ylabel('Gain []')
            end
        case 2 % 1次関数
            disp('1次関数')
            for k = 1:3
                case2_nums_A = [0.5 1 2];
                for k_2 = 1:3
                    case2_nums_B = [-1 0 1];
                    handle_POEnum_A = case2_nums_A(k);
                    handle_POEnum_B = case2_nums_B(k_2);
                    run('C_024gIE_EKF_adaptive.m');
                    cd C.EKF
                    title_str = ['1次関数　A:' num2str(handle_POEnum_A) ' B:' num2str(handle_POEnum_B)];
                    run('plot01.m')
                    figure
                    grid on
                    hold on
                    plot(standard_array,handle_POEnum_A .* standard_array + handle_POEnum_B,'.r')
                    xlim([0 10])
                    ylim([0 10])
                    title(title_str)
                    xlabel('error [m]')
                    ylabel('Gain []')
                end
            end
        case 3 % 2次関数
            disp('2次関数')
            for k = 1:5
                case3_numsA = [0.2 0.6 1 1.5 2];
                handle_POEnum_A = case3_numsA(k);
                run('C_024gIE_EKF_adaptive.m');
                cd C.EKF
                title_str = ['2次関数　A:' num2str(handle_POEnum_A)];
                run('plot01.m')
                figure
                grid on
                hold on
                plot(standard_array,handle_POEnum_A .* standard_array.^2,'.r')
                xlim([0 10])
                ylim([0 10])
                title(title_str)
                xlabel('error [m]')
                ylabel('Gain []')
            end
        case 4 % 指数関数
            disp('指数関数')
            for k = 1:3
                case4_numsA = [1.2 1.3 sqrt(2)];
                handle_POEnum_A = case4_numsA(k);
                run('C_024gIE_EKF_adaptive.m');
                cd C.EKF
                title_str = ['指数関数　A:' num2str(handle_POEnum_A)];
                run('plot01.m')
                figure
                grid on
                hold on
                plot(standard_array,handle_POEnum_B .* handle_POEnum_A .^ standard_array,'.r')
                xlim([0 10])
                ylim([0 10])
                title(title_str)
                xlabel('error [m]')
                ylabel('Gain []')
            end
    end
    cd ..
end

%%
load('20241226_GNSSonly_all.mat')
% run('C_024gIE_EKF_adaptive.m');
% cd C.EKF
title_str = 'GNSS only';
run('plot01.m')

%% D.Evaluation
% cd D.Evaluation


% if what_analyze == 0; run('D_024gIE_Evaluation.m');
% % else; run('D_024gIE_Evaluation.m');
% end

%% E.dit
% cd E.dit
% if what_analyze == 0; open('editer.m'); end




clear what_analyze