% readpos
[file,path] = uigetfile('*.pos','入力するgnssデータを選択','..\0_dataHunger\');
input_pos = file; % 入力するkinematic測位をしたposファイルを指定
[rawdata_pos, base_pos] = read_pos([path,file]);
% 6列目　gnssflag
gnss_flag(:) = rawdata_pos(:,6);
for i = 1:length(rawdata_pos)
    velocity_GNSS = sqrt(rawdata_pos(:,16).^2 + rawdata_pos(:,17).^2 + rawdata_pos(:,18).^2);
end
gnss_x(:,1) = rawdata_pos(:,3); gnss_y(:,1) = rawdata_pos(:,4); gnss_z(:,1) = rawdata_pos(:,5);
%imu
[file,path] = uigetfile('*.csv','入力するIMUデータを選択','..\0_dataHunger\');

eagleyelog = readtable([path,file],'VariableNamingRule','preserve');

vname=eagleyelog.Properties.VariableNames;
eagleyelog_head = string(vname);

eagleye_log = table2array(eagleyelog);
%poslv
[file,path] = uigetfile({'*.mat'},'入力するreferenceデータを選択','..\0_dataHunger\');
load([path,file]);

%パラメータの設定
offset_time = 0;%sec
Time_Eagleye = eagleye_log(:,1)/ 10.^9;
TOW_POSLV = trajectory(:,1);
index_eval_section = find(eagleye_log(:,86) == 1);
tTOW_Eagleye = eagleye_log(:,8)/1000;
TOW_Eagleye = tTOW_Eagleye;
Distance = eagleye_log(:,35);
Yawrate_Eagleye = -eagleye_log(:,4);%ROSの軸とPOSLVの軸が逆なのでマイナス

%TOWの設定
tic
disp('TOWの設定')
flag_GNSS = zeros(length(eagleye_log),1);
for i = 2:length(eagleye_log)
    
    if tTOW_Eagleye(i) ~= tTOW_Eagleye(i-1) 
        flag_GNSS(i) = 1;        
    end    
end
for i = 2:length(eagleye_log)
    
    dt = Time_Eagleye(i)  - Time_Eagleye(i-1);
    if flag_GNSS(i) == 1
        TOW_Eagleye(i) =  tTOW_Eagleye(i);
    else
        TOW_Eagleye(i) = TOW_Eagleye(i-1) + dt;
    end
    
end
toc
%POSLVと同期とデータの設定
tic
f = waitbar(0,'efficiency START');
disp('POSLVと同期とデータの設定')
POSLV_Sync = zeros(length(Time_Eagleye),size(trajectory,2));
flag_poslv = zeros(length(Time_Eagleye),1);
disp(['POSLVのオフセット時間',num2str(offset_time),'秒'])
for i = 1 : length(Time_Eagleye)
    [Y,I] = min(abs( TOW_Eagleye(i) + offset_time - TOW_POSLV ));
    if abs(Y) < 0.005
        POSLV_Sync(i,:) = trajectory(I,:);
        flag_poslv(i) = I;
        if mod(floor(100*i/length(Time_Eagleye)),3) == 0
            if floor(rem(i,400)) == 0
                waitbar(i/length(Time_Eagleye) ,f ,['Loding    (' num2str(i) '/' num2str(length(Time_Eagleye)) ')']);
            elseif floor(rem(i,400)) == 1
                waitbar(i/length(Time_Eagleye) ,f ,['Loding.   (' num2str(i) '/' num2str(length(Time_Eagleye)) ')']);
            elseif floor(rem(i,400)) == 2
                waitbar(i/length(Time_Eagleye) ,f ,['Loding..  (' num2str(i) '/' num2str(length(Time_Eagleye)) ')']);
            elseif floor(rem(i,400)) == 3
                waitbar(i/length(Time_Eagleye) ,f ,['Loding...  (' num2str(i) '/' num2str(length(Time_Eagleye)) ')']);
            end
            % disp([num2str(100*i/length(Time_Eagleye)) '%'])
        end
    end
end
close(f)
tindex_poslv = find( flag_poslv > 0);
index_poslv = [tindex_poslv(1):length(Time_Eagleye)];
toc

% sync
tic
clear M I
disp('時刻同期')
for i = 1:length(rawdata_pos)
    [M(i,1), I(i,1)] = min(abs(TOW_Eagleye(:) - rawdata_pos(i,2)));
end
for i = 1:length(I)-1
    dI(i,1) = I(i+1) - I(i);
end
I0 = find(~dI); Ig = find(dI);
I1 = I; I1(I0) = [];

gnss = rawdata_pos(Ig(:,1),:);
EAGLEYE = zeros(length(rawdata_pos), size(eagleye_log,2));
EAGLEYE = eagleye_log(I1(:,1),:);
EAGLEYE(:,8) = TOW_Eagleye(I1(:,1),:);
EAGLEYE(end,:) = [];
ref = POSLV_Sync(I1(:,1),:);
ref(end,:) = [];
toc

dt_gnss = rawdata_pos(2,2) - rawdata_pos(1,2);
dt_EAGLEYE = EAGLEYE(2,8) - EAGLEYE(1,8);
de_ref = trajectory(2,1) - trajectory(1,1);

%/////////////////////////////////////////////////////////////////////////%

% eagleye sutruct
field1 = 'Time';
field2 = 'Roll_rate';
field3 = 'Pitch_rate';
field4 = 'Yaw_rate';
field5 = 'roll';
field6 = 'pitch';
field7 = 'yaw';
eagleye = struct(field1,[],field2,[],field3,[],field4,[],field5,[],field6,[],field7,[]);
clear -regexp ^field

eagleye.Time = EAGLEYE(:,1);
Time_Eagleye = EAGLEYE(:,8);
eagleye.Roll_rate = EAGLEYE(:,2);
eagleye.Pitch_rate = EAGLEYE(:,3);
eagleye.Yaw_rate = EAGLEYE(:,4);
eagleye.roll = EAGLEYE(:,123);
eagleye.pitch = EAGLEYE(:,115);
eagleye.yaw = EAGLEYE(:,58);

sim_epoch=length(eagleye.Time(:,1)); 

% 時刻設定

% dt
g=9.80665;
% dt
clear dt
time_test=Time_Eagleye;
for i = 1:sim_epoch
    if i<sim_epoch
        dt(i)=time_test(i+1)-time_test(i);
    end
    if i == sim_epoch 
        dt(i)=0.01;   
    end
end

obs_roll=obs_roll_test2;
obs_pitch=obs_pitch_test2;%車両モデルから求めたピッチ角
flag_obs_roll=(imag(obs_roll) == 0);%ロール角が実数のデータを探す（虚数部が0のデータを探す）
flag_obs_pitch=(imag(obs_pitch) == 0);%ピッチ角が実数のデータを探す（虚数部が0のデータを探す）
flag_obs_real=flag_obs_roll+flag_obs_pitch; %ロール角、ピッチ角ともに実数のところを探すflag
for i = 1:sim_epoch
    if flag_obs_real(i,1)==2
    flag_obs_real(i,1)=1;
    else
    flag_obs_real(i,1)=0;
    end
end
index_obs_real=find(flag_obs_real(:,1)==1);
flag_test3= zeros(length(Time_Eagleye),1);
flag_test3(index_obs_real(:, 1)) = 1;

[Heading_POSLV,Eagleye_enuTF,GNSS_enuTF] =...
    EKF2024gI_antennaSetFunc(AntVec,ref,Eagleye_enu,gnss_enu);

gnss_xyz(:,1) = gnss_x(:,1); gnss_xyz(:,2) = gnss_y(:,1); gnss_xyz(:,3) = gnss_z(:,1);
gnss_velxyz(:,1:3) = gnss(:,16:18);
ORG_XYZ = [ref(1,6) ref(1,7) ref(1,8)];
for i = 1:length(gnss_xyz)  
    gnss_enu(i,:) = xyz2enu( gnss_xyz(i,:) ,  ORG_XYZ );
    gnss_vel_enu(i,:) = xyz2enu_vel( gnss_velxyz(i,:) ,  ORG_XYZ );
    Heading_GNSS(i,1) = atan2(gnss_vel_enu(i,1),gnss_vel_enu(i,2));
end
ref_xyz(:,1:3) = ref(:,6:8);
for i = 1:length(ref_xyz)  
    ref_enu(i,:) = xyz2enu( ref_xyz(i,:) ,  ORG_XYZ );
end
ax_EAGLEYE=EAGLEYE(:,5); %加速度　x軸
ay_EAGLEYE=EAGLEYE(:,6); %加速度　y軸
az_EAGLEYE=-EAGLEYE(:,7); %加速度　z軸
% %x軸加速度のオフセットを補正　
ax_EAGLEYE_offset=EAGLEYE(:,118);
ax_EAGLEYE=ax_EAGLEYE+ax_EAGLEYE_offset;
EAGLEYE_accxyz = [ax_EAGLEYE ay_EAGLEYE az_EAGLEYE];
EAGLEYE_velxyz = EAGLEYE(:,2:4);
Eagleye_llhrad = [EAGLEYE(:,132)/180*pi EAGLEYE(:,133)/180*pi EAGLEYE(:,134)]; 
for i = 1:length(Eagleye_llhrad)
    Eagleye_xyz(i,:) = llh2xyz( Eagleye_llhrad(i,:) );
    Eagleye_enu(i,:) = xyz2enu( Eagleye_xyz(i,:) ,  ORG_XYZ );
    Eagleye_vel_enu(i,:) = xyz2enu_vel( EAGLEYE_velxyz(i,:) ,  ORG_XYZ );
    Eagleye_acc_enu(i,:) = xyz2enu_vel( EAGLEYE_accxyz(i,:) ,  ORG_XYZ );
end
Eagleye_acc_enu3d(:,1) = sqrt(Eagleye_acc_enu(:,1).^2 + Eagleye_acc_enu(:,2).^2 + Eagleye_acc_enu(:,3).^2);

