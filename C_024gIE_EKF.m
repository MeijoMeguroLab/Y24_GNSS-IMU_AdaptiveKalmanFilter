%% EKF

disp('C_EKF start')
%%
% load('20241126_run3.mat')
for i = 1:length(data_sync_Eagleye(:,9:11))
    state_pos_Eagleye_enu(i,:) = xyz2enu( data_sync_Eagleye(i,9:11) ,  handle_ORG_XYZ );
end

init_x = data_pos_Ref_enu(1,1);
init_y = data_pos_Ref_enu(1,2);
init_z = data_pos_Ref_enu(1,3);

for i = 1:time_SimEpoch
    if i == 1          %変数に初期値を代入
        KF_x(i) = init_x;
        KF_y(i) = init_y;
        KF_z(i) = init_z;
        KF_roll(i)  = init_roll;
        KF_pitch(i) = init_pitch;
        KF_yaw(i)   = init_yaw;
        KF_velocity(i)     = init_velocity;
        KF_covariance(i,:) = setVecCovariance(init_covariance,7); %init_covarianceを1列目に入れている。（5×5の行列の25個の要素を1行に並べてる）
        P = init_covariance;
        continue
    end


    % Predict　予測 t=k-1までの状態からt=kの状態を予測
    Pre_KF_roll = KF_roll(i-1) + (data_struct_Eagleye.Roll_rate(i-1,1)+data_struct_Eagleye.Pitch_rate(i-1,1)*sin(KF_roll(i-1))*tan(KF_pitch(i-1))+data_yawrate(i-1,1)*cos(KF_roll(i-1))*tan(KF_pitch(i-1)))*const_dt3(i); %
    % Pre_KF_pitch = KF_pitch(i-1) + (Pre_KF_pitchrate*cos(KF_roll(i-1))-Pre_KF_yawrate*sin(KF_roll(i-1)))*const_dt3(i); %
    Pre_KF_pitch = KF_pitch(i-1) + (data_struct_Eagleye.Pitch_rate(i-1,1)*cos(KF_roll(i-1))-data_yawrate(i-1,1)*sin(KF_roll(i-1)))*const_dt3(i);
    Pre_KF_yaw = KF_yaw(i-1) + data_yawrate(i-1,1)*const_dt3(i); %PΘ(k)=Θ(k-1)+data_yawrate(k)*const_dt3
    Pre_KF_z = KF_z(i-1) + sin(KF_pitch(i-1))*state_velocity(i)*const_dt3(i);

    option_calc_type = 0;

    if option_calc_type==0
        Pre_KF_x = KF_x(i-1) + KF_velocity(i-1)*(data_yawrate(i-1,1)^(-1))*(-cos(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))+cos(KF_yaw(i-1,1)));
        Pre_KF_y = KF_y(i-1) + KF_velocity(i-1)*(data_yawrate(i-1,1)^(-1))*(sin(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))-sin(KF_yaw(i-1,1)));
    end


    Pre_KF_velocity(1,1) = KF_velocity(i-1) + data_acc_Eagleye_enu3d(i-1,1)*const_dt3(i);


    % A(i)=(Pre_KF_pitchrate*cos(KF_roll(i-1))-Pre_KF_yawrate*sin(KF_roll(i-1)))*const_dt3(i);


    %ヤコビアン

     jF = ...
        [1, 0, 0, 0, 0, state_velocity(i-1)*(data_yawrate(i-1,1)^(-1))*(sin(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))-sin(KF_yaw(i-1,1))), 0
         0, 1, 0, 0, 0, state_velocity(i-1)*(data_yawrate(i-1,1)^(-1))*(cos(KF_yaw(i-1,1)+data_yawrate(i-1,1)*const_dt3(i))-cos(KF_yaw(i-1,1))), 0
         0, 0, 1, 0, 0, 0, 0
         0, 0, 0, 1+(data_struct_Eagleye.Pitch_rate(i-1,1)*cos(KF_roll(i-1))*tan(KF_pitch(i-1))-data_yawrate(i-1,1)*sin(KF_roll(i-1))*tan(KF_pitch(i-1)))*const_dt3(i), (data_struct_Eagleye.Pitch_rate(i-1,1)*sin(KF_roll(i-1))/(cos(KF_pitch(i-1))*cos(KF_pitch(i-1)))+data_yawrate(i-1,1)*cos(KF_roll(i-1))/(cos(KF_pitch(i-1))*cos(KF_pitch(i-1))))*const_dt3(i), 0, 0
         0, 0, 0, (-data_struct_Eagleye.Pitch_rate(i-1,1)*sin(KF_roll(i-1))-data_yawrate(i-1,1)*cos(KF_roll(i-1)))*const_dt3(i), 1, 0, 0
         0, 0, 0, 0, 0, 1, 0
         0, 0, 0, 0, 0, 0, 1];


    %予測値の行列に値を代入
    Pre_KF_status = [Pre_KF_x, Pre_KF_y, Pre_KF_z, Pre_KF_roll, Pre_KF_pitch, Pre_KF_yaw, Pre_KF_velocity]';
    %予測値の共分散を計算
    Pre_KF_covariance = jF*P*jF'+Q; %P(k|k-1)=F(k)P(k-1|k-1)F^T(k)+Q

%%
%観測値の方位角をカルマンフィルタの推定値に近づける
        handle_Heading_GNSS_control(i,1)=Pre_KF_yaw-handle_Heading_GNSS(i,1);
        handle_Heading_GNSS(i,1)=handle_Heading_GNSS(i,1);
        if handle_Heading_GNSS_control(i,1)<-pi
           handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)-2*pi;
        end
        if handle_Heading_GNSS_control(i,1)<-3*pi
           handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)-4*pi;
        end
        if handle_Heading_GNSS_control(i,1)<-5*pi
           handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)-6*pi;
        end
        if handle_Heading_GNSS_control(i,1)<-7*pi
           handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)-8*pi;
        end
        if handle_Heading_GNSS_control(i,1)<-9*pi
           handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)-10*pi;
        end
        if handle_Heading_GNSS_control(i,1)>pi
           handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)+2*pi;
        end
        if handle_Heading_GNSS_control(i,1)>3*pi
           handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)+4*pi;
        end
        if handle_Heading_GNSS_control(i,1)>5*pi
           handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)+6*pi;
        end
        if handle_Heading_GNSS_control(i,1)>7*pi
           handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)+8*pi;
        end
        if handle_Heading_GNSS_control(i,1)>9*pi
           handle_Heading_GNSS(i,1)= handle_Heading_GNSS(i,1)+10*pi;
        end
        state_gnss_yaw=handle_Heading_GNSS;


    %% Update　更新
%----------------eagleye_log(i,4)---------------------
% gnss_yaw(:,1) = eagleye.yaw(:);
    if option_Pitch_type==0 %ピッチ角を車両モデルから求める場合

        if flag_gnss(i) == 1 && flag_test3(i) == 1 %GNSS　◯　　ロール、ピッチ　◯　
            KF_obs = [state_pos_Eagleye_enu(i,1), state_pos_Eagleye_enu(i,2), state_pos_Eagleye_enu(i,3), element_R_roll(i), element_P_pitch(i), state_gnss_yaw(i), data_velocity(i)]'; %観測方程式
            H = ...
                [1, 0, 0, 0, 0, 0, 0
                 0, 1, 0, 0, 0, 0, 0
                 0, 0, 1, 0, 0, 0, 0
                 0, 0, 0, 1, 0, 0, 0
                 0, 0, 0, 0, 1, 0, 0
                 0, 0, 0, 0, 0, 1, 0
                 0, 0, 0, 0, 0, 0, 1];

             R = R_GNSS_re;
        end

        if flag_gnss(i) == 1 && flag_test3(i) == 0 %GNSS　◯　　ロール、ピッチ　☓　
            KF_obs = [state_pos_Eagleye_enu(i,1), state_pos_Eagleye_enu(i,2), state_pos_Eagleye_enu(i,3), 0, 0, state_gnss_yaw(i), data_velocity(i)]'; %観測方程式
            H = ...
                [1, 0, 0, 0, 0, 0, 0
                 0, 1, 0, 0, 0, 0, 0
                 0, 0, 1, 0, 0, 0, 0
                 0, 0, 0, 0, 0, 0, 0
                 0, 0, 0, 0, 0, 0, 0
                 0, 0, 0, 0, 0, 1, 0
                 0, 0, 0, 0, 0, 0, 1];

             R = R_GNSS_im;
        end

        if flag_gnss(i) == 0 && flag_test3(i) == 1 %GNSS　☓　　ロール、ピッチ　◯
            KF_obs = [state_pos_Eagleye_enu(i,1), state_pos_Eagleye_enu(i,2), state_pos_Eagleye_enu(i,3),...
                element_R_roll(i), element_P_pitch(i), 0, data_velocity(i)]'; %観測方程式
            H = ...
                [1, 0, 0, 0, 0, 0, 0
                 0, 1, 0, 0, 0, 0, 0
                 0, 0, 1, 0, 0, 0, 0
                 0, 0, 0, 1, 0, 0, 0
                 0, 0, 0, 0, 1, 0, 0
                 0, 0, 0, 0, 0, 0, 0
                 0, 0, 0, 0, 0, 0, 1];

             R = R_DR_re;
        end

        if flag_gnss(i) == 0 && flag_test3(i) == 0 %GNSS　☓　　ロール、ピッチ　☓
            KF_obs = [state_pos_Eagleye_enu(i,1), state_pos_Eagleye_enu(i,2), state_pos_Eagleye_enu(i,3),...
                0, 0, 0, (data_velocity(i)+element_P_pitch(i))/2]'; %観測方程式

            H = ...
                [1, 0, 0, 0, 0, 0, 0
                 0, 1, 0, 0, 0, 0, 0
                 0, 0, 1, 0, 0, 0, 0
                 0, 0, 0, 0, 0, 0, 0
                 0, 0, 0, 0, 0, 0, 0
                 0, 0, 0, 0, 0, 0, 0
                 0, 0, 0, 0, 0, 0, 1];
             R = R_DR_im;
        end

    end

%%
    y = KF_obs - H*Pre_KF_status; %y(k)=z(k)-H(x(k|k-1))
    S = H*Pre_KF_covariance*H' + R; %S(k)=H*P(k|k-1)*H(k)^T+R
    K = Pre_KF_covariance*H'*inv(S); %K(k)=P(k|k-1)*H(k)^T*(S(k)^(-1))

    Update_KF_status = Pre_KF_status + K*y; %x(k|k)=x(k|k-1)+K(k)*y(k)
    Update_KF_covariance = (eye(7) - K*H)*Pre_KF_covariance; %P(k|k)=(I(単位行列)-K(k)*H(k))*P(k|k-1)

    %KF後の値を代入
    KF_x(i) = Update_KF_status(1);
    KF_y(i) = Update_KF_status(2);
    KF_z(i) = Update_KF_status(3);
    KF_roll(i) = Update_KF_status(4);
    KF_pitch(i) = Update_KF_status(5);
    KF_yaw(i) = Update_KF_status(6);
    KF_velocity(i) = Update_KF_status(7);
    KF_covariance(i,:) = setVecCovariance(Update_KF_covariance,7);
    P = Update_KF_covariance;

end

%%
KF_enu = [KF_x(:,1),KF_y(:,1),KF_z(:,1)];
analize_error_EKF(:,1:3) = KF_enu(:,1:3)  - data_pos_Ref_enu(:,1:3);
analize_error_EKF3d(:,1) = sqrt(analize_error_EKF(:,1).^2+analize_error_EKF(:,2).^2+analize_error_EKF(:,3).^2);


% Utilities/function
function outCov = setVecCovariance(inCov,Dim)
    outCov = zeros(Dim*Dim,1);
    for i = 1:Dim
        for j = 1:Dim
            outCov((i-1)*Dim+j,1) = inCov(i,j);
        end
    end
end

function outCov = setMatCovariance(inCov,Dim)
    outCov = zeros(Dim,Dim);
    for i = 1:Dim
        for j = 1:Dim
            outCov(i,j) = inCov(:,(i-1)*Dim+j);
        end
    end
end