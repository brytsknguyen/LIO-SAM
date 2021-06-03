function [P_ate, rot_rmse] = check_LIOSAM_cal_err_parall(test_id, test_fullname)

% clearvars('-except', vars_to_keep{:});
close all;

% addpath('/home/tmn/MATLAB_WS');

t_shift = 0;

mygreen   = [0.0,    0.75,   0.0];
mymagenta = [1.0,    0.0,    1.0];
myorange  = [1.0,    0.3333, 0.0];
mycyan    = [0.0314, 0.75, 0.75];

fighd = [];



%% Get the exp number
exp_name =  test_fullname.name;
exp_path = [test_fullname.folder '/' test_fullname.name '/'];


gndtr_pos_fn       = [exp_path 'leica_pose.csv'];
gndtr_dji_imu_fn   = [exp_path 'dji_sdk_imu.csv'];
gndtr_vn100_imu_fn = [exp_path 'vn100_imu.csv'];
pred_est_fn        = [exp_path 'opt_odom.csv'];
trans_B2prism_fn   = [exp_path '../trans_B2prism.csv'];
rot_B2vn100_fn     = [exp_path '../rot_B2vn100.csv'];
rot_B2djiimu_fn    = [exp_path '../rot_B2djiimu.csv'];


% Translation from body frame to the prism
trans_B2prism = csvread(trans_B2prism_fn, 0, 0);



%% Read the data from log and start processing

% Read the gndtr data from logs
for dummy = 1

% Position groundtr
gndtr_pos_data = csvread(gndtr_pos_fn,  1, 0);

% Orientation groundtr
% Check if file size is 0
dji_file   = dir(gndtr_dji_imu_fn);
vn100_file = dir(gndtr_vn100_imu_fn);
imu_topic = '';
% Orientation is in z upward frame, convert it to body frame
rot_B_Beimu = eye(3);

dji_present   = (dji_file.bytes ~= 0);
vn100_present = (vn100_file.bytes ~= 0);

if vn100_present
    gndtr_vn100_data = csvread([exp_path 'vn100_imu.csv'], 1, 0);
end

if dji_present
    gndtr_dji_data = csvread(gndtr_dji_imu_fn, 1, 0);
    rot_B_Beimu    = csvread(rot_B2djiimu_fn, 0, 0);
    imu_topic = '/dji_sdk/imu';
elseif ~dji_present && vn100_present
    gndtr_dji_data = csvread([exp_path 'vn100_imu.csv'], 1, 0);
    rot_B_Beimu    = csvread(rot_B2vn100_fn,  0, 0);
    imu_topic = '/imu/imu';
end

t0_ns = gndtr_pos_data(1, 1);

% pos groundtruthdata
t_pos = (gndtr_pos_data(:, 1) - t0_ns)/1e9 + t_shift;
P     = gndtr_pos_data(:, 4:6);

% ori groundtruthdata
t_ori = (gndtr_dji_data(:, 1)- t0_ns)/1e9;
Q     =  quatnormalize(gndtr_dji_data(:, [7, 4:6]));
Q0    =  Q(1, :);

% Delete the duplicate in position groundtruth data
[~, Px_unq_idx] = unique(P(:, 1));
[~, Py_unq_idx] = unique(P(:, 2));
[~, Pz_unq_idx] = unique(P(:, 3));

P_unq_idx = union(union(Px_unq_idx, Py_unq_idx), Pz_unq_idx);
P = P(P_unq_idx, :);
t_pos = t_pos(P_unq_idx, :);

% Delete the duplicate in orientation groundtruth data
[~, Qx_unq_idx] = unique(Q(:, 1));
[~, Qy_unq_idx] = unique(Q(:, 2));
[~, Qz_unq_idx] = unique(Q(:, 3));
[~, Qw_unq_idx] = unique(Q(:, 4));

Q_unq_idx = union(union(union(Qx_unq_idx, Qy_unq_idx), Qz_unq_idx),...
                        Qw_unq_idx);
Q     = Q(Q_unq_idx, :);
t_ori = t_ori(Q_unq_idx, :);

Q_B_Beimu = rotm2quat(rot_B_Beimu);
Q = quatmultiply(Q, quatinv(Q_B_Beimu));

end

% Read the LIOSAM fusion estimate data from logs
for dummy = 1

% SLAM estimate
LIOSAM_data = csvread(pred_est_fn, 1, 0);
t_h = (LIOSAM_data(:, 1) - t0_ns)/1e9;
P_h =  LIOSAM_data(:, 4:6);
Q_h =  quatnormalize(LIOSAM_data(:, [10, 7:9]));
V_h =  LIOSAM_data(:, 11:13);

% Compensate the position estimate with the prism displacement
P_h = P_h + quatconv(Q_h, trans_B2prism);

end


%% Resample and align the groundtruth with each estimate

% Resample gndtruth by LIOSAM time
for dummy = 1
    
% Find the interpolated time stamps
[rsh_pos_itp_idx(:, 1), rsh_pos_itp_idx(:, 2)] = combteeth(t_h, t_pos);
[rsh_ori_itp_idx(:, 1), rsh_ori_itp_idx(:, 2)] = combteeth(t_h, t_ori);

% Remove the un-associatable samples
rsh_nan_idx = find(isnan(rsh_pos_itp_idx(:, 1))...
                   | isnan(rsh_pos_itp_idx(:, 2))...
                   | isnan(rsh_ori_itp_idx(:, 1))...
                   | isnan(rsh_ori_itp_idx(:, 2)));

t_h_org = t_h;
P_h_org = P_h;
Q_h_org = Q_h;
V_h_org = V_h;

rsh_pos_itp_idx(rsh_nan_idx, :) = [];
rsh_ori_itp_idx(rsh_nan_idx, :) = [];
t_h(rsh_nan_idx, :)     = [];
P_h(rsh_nan_idx, :)     = [];
Q_h(rsh_nan_idx, :)     = [];
V_h(rsh_nan_idx, :)     = [];

% interpolate the pos gndtr state
P_rsh = vecitp(P,  t_pos, t_h, rsh_pos_itp_idx);

% Align the LIOSAM estimate with ground truth
[rot_align_h, trans_align_h ] = traj_align(P_rsh, P_h);

% Align the position estimate
P_h      = (rot_align_h*P_h'     + trans_align_h)';
P_h_full = (rot_align_h*P_h_org' + trans_align_h)';

% Align the velocity estimate
V_h      = (rot_align_h*V_h')';
V_h_full = (rot_align_h*V_h_org')';

% interpolate the ori gndtr state
Q_rsh = quatitp(Q, t_ori, t_h, rsh_ori_itp_idx);


% Find the optimized rotation between the groundtruth and the estimate
rot_rsh = quat2rotm(Q_rsh);
rot_h   = quat2rotm(Q_h);

rot_rsh2h_opt = rot_opt(rot_rsh, rot_h);

% Align the ori estimate
Q_h = quatmultiply(rotm2quat(rot_rsh2h_opt), Q_h);

end


%% Calculate the position and rotation errors

% LIOSAM estimate
for dummy = 1
    
P_h_err     = P_rsh - P_h;
P_h_rmse    = rms(P_h_err);
P_h_ate     = norm(P_h_rmse);
P_h_err_nrm = sqrt(dot(P_h_err, P_h_err, 2));

Q_h_err       = quatmultiply(quatinv(Q_h), Q_rsh);
YPR_h_err	  = wrapToPi(quat2eul(Q_h_err));
rot_h_ang_err = quat2axang(Q_h_err);
% Wrap this error to -pi to pi;
rot_h_ang_err(:, end) = wrapToPi(rot_h_ang_err(:, end));
% Find the outliers
olrh_idx = find(isoutlier(rot_h_ang_err(:, end), 'mean'));
% Extract the inlier errors
rot_h_ang_err_nolr = rot_h_ang_err(:, end);
t_h_nolr           = t_h;
rot_h_ang_err_nolr(olrh_idx, :) = [];
t_h_nolr(olrh_idx)              = [];
% Calculate the error
rot_h_rmse         = rms(rot_h_ang_err_nolr(:, end))/pi*180;
% rot_h_ang_err_norm = abs(rot_h_ang_err(:, end));

end


%% Save the important variables for later analyses
save([exp_path exp_name '_poses.mat'],...
     't_h',      'P_rsh',   'Q_rsh',...
     'P_h',      'Q_h',     'rot_align_h', 'trans_align_h',...
     't_pos',    'P',...
	 't_h_org',  'P_h_org');

% Save the errors calculated
save([exp_path exp_name '_rmse.mat'],...
     'P_h_ate', 'rot_h_rmse');

 

%% Print the result
fprintf(['test: %2d. %s.\n'...
         'Error:\tPos [m]\t Rot [deg]\n'...
         'LIOS:\t%6.4f\t %7.4f.\n'...
          test_id, exp_name(8:end),...
          P_h_ate, rot_h_rmse]);
      
P_ate    = [P_h_ate];
rot_rmse = [rot_h_rmse];



%% Calculate the maximum time
t_max = max([t_pos; t_h]);



%% Plot the 3D trajectory
figpos = [1920 0 0 0] + [0, 480, 630, 400];
figure('position', figpos, 'color', 'w', 'paperpositionmode', 'auto');
fighd = [fighd gcf];
hold on;

plot3(P(:, 1), P(:, 2), P(:, 3),...
      '.r', 'markersize', 5);
plot3(P_h_full(:, 1),  P_h_full(:, 2),  P_h_full(:, 3),...
      'b', 'linewidth', 2);

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
grid on;
daspect([1 1 1]);
view([-21 15]);
set(gca, 'fontsize', 13);
tightfig;
lg_hd = legend('Groundtruth', 'LIOSAM');
set(lg_hd, 'numcolumns', 3, 'position', [0.13, 0.9175, 0.58, 0.06]);

tightfig;

saveas(gcf, [exp_path exp_name '_traj.fig']);
% saveas(gcf, [exp_path exp_name '_traj.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_traj.png']);



%% Plot the time evolution of position

figpos = [1920 0 0 0] + [0, 0, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
plot(t_pos, P(:, 1),   'color', 'r', 'linewidth', 3);
plot(t_h,   P_h(:, 1), 'color', 'b', 'linewidth', 2);

ylabel('X [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
plot(t_pos, P(:, 2),   'color', 'r', 'linewidth', 3);
plot(t_h,   P_h(:, 2), 'color', 'b', 'linewidth', 2);

ylabel('Y [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
plot(t_pos, P(:, 3),    'color', 'r', 'linewidth', 3);
plot(t_h,   P_h(:, 3),  'color', 'b', 'linewidth', 2);

xlabel('Time [s]');
ylabel('X [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);


lg_hd = legend('Groundtruth', 'LIOSAM');
set(lg_hd, 'numcolumns', 2, 'position', [0.6 0.69 0.38 0.08]);

tightfig(gcf);

saveas(gcf, [exp_path exp_name '_xyzt.fig']);
% saveas(gcf, [exp_path exp_name '_xyzt.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_xyzt.png']);



%% Plot the time evolution of orientation

% Calculate the yaw pitch rol relative to the initial position
YPR       = wrapToPi(quat2eul(quatmultiply(quatinv(Q(1, :)), Q)));
YPR_h     = wrapToPi(quat2eul(quatmultiply(quatinv(Q(1, :)), Q_h)));


figpos = [1920 0 0 0] + [630, 480, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
plot(t_ori,   YPR(:, 1)*180/pi,   'r', 'linewidth', 3);
plot(t_h,     YPR_h(:, 1)*180/pi, 'b', 'linewidth', 2);
ylabel('Yaw [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
plot(t_ori,   YPR(:, 2)*180/pi,   'r', 'linewidth', 3);
plot(t_h,     YPR_h(:, 2)*180/pi, 'b', 'linewidth', 2);
ylabel('Pitch [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
plot(t_ori,   YPR(:, 3)*180/pi,   'r', 'linewidth', 3);
plot(t_h,     YPR_h(:, 3)*180/pi, 'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('Roll [deg]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Groundtruth', 'LIOSAM');
set(lg_hd, 'numcolumns', 2, 'position', [0.15 0.31 0.39 0.06]);

tightfig(gcf);

saveas(gcf, [exp_path exp_name '_yprt.fig']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_yprt.png']);



%% Plot the time evolution of velocity estimate
figpos = [1920 0 0 0] + [630, 0, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

hold on;
plot(t_h, V_h(:, 1), 'r', 'linewidth', 2);
plot(t_h, V_h(:, 2), 'g', 'linewidth', 2);
plot(t_h, V_h(:, 3), 'b', 'linewidth', 2);
plot(t_h, sqrt(dot(V_h, V_h, 2)), 'g', 'linewidth', 2);
% xlabel('Time [s]');
ylabel('Vel. Est. [m/s]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Vx', 'Vy', 'Vz', 'norm (speed)');

set(lg_hd, 'numcolumns', 4,...
           'position', [0.2317 0.8767 0.6000 0.1000]);

tightfig(gcf);

saveas(gcf, [exp_path exp_name '_vxvyvz_t.fig']);
% saveas(gcf, [exp_path exp_name '_vxvyvz_t.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_vxvyvz_t.png']);



%% Plot the time evolution of position error
figpos = [1920 0 0 0] + [630*2, 480, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
plot(t_h,     P_h_err(:, 1),     'color', 'b',       'linewidth', 2);
ylabel('X Err. [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
plot(t_h,     P_h_err(:, 2),     'color', 'b',       'linewidth', 2);
ylabel('Y Err [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
plot(t_h,     P_h_err(:, 3),     'color', 'b',       'linewidth', 2);
ylabel('Z Err [m]');
xlabel('Time [s]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);


lg_hd = legend('LIOSAM');
set(lg_hd, 'numcolumns', 4, 'position', [0.73 0.6875 0.20 0.06]);


tightfig(gcf);
saveas(gcf, [exp_path exp_name '_xyz_err_t.fig']);
% saveas(gcf, [exp_path exp_name '_xyz_err_t.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_xyz_err_t.png']);



%% Plot the time evolution of orientation err
figpos = [1920 0 0 0] + [630*2, 0, 630, 400];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

subplot(3, 1, 1);
hold on;
plot(t_h,  YPR_h_err(:, 1)*180/pi,  'b', 'linewidth', 2);
ylabel('$\tilde{\psi}$ [deg]', 'interpreter', 'latex');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 2);
hold on;
plot(t_h, YPR_h_err(:, 2)*180/pi,    'b', 'linewidth', 2);
ylabel('$\tilde{\theta}$ [deg]', 'interpreter', 'latex');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

subplot(3, 1, 3);
hold on;
plot(t_h,  YPR_h_err(:, 3)*180/pi,    'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('$\tilde{\phi}$ [deg]', 'interpreter', 'latex');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('LIOSAM');
set(lg_hd, 'position', [0.12 0.38 0.21 0.06]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_ypr_err_t.fig']);
% saveas(gcf, [exp_path exp_name '_ypr_err_t.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_ypr_err_t.png']);



%% Plot the combined time evolution of position estimation error
figpos = [1920 0 0 0] + [930, 750, 630, 200];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

hold on;
plot(t_h, P_h_err(:, 1), 'r', 'linewidth', 2);
plot(t_h, P_h_err(:, 2), 'g', 'linewidth', 2);
plot(t_h, P_h_err(:, 3), 'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('Error [m]');
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Px error', 'Py error', 'Pz error');

set(lg_hd, 'orientation', 'horizontal',...
    'position', [0.3 0.85 0.4 0.1000]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_xyz_h_err_t.fig']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_xyz_h_err_t.png']);



%% Plot the combined time evolution of orientation estimation error
figpos = [1920 0 0 0] + [300, 750, 630, 200];
figure('position', figpos, 'color', 'w');
fighd = [fighd gcf];

hold on;
plot(t_h, YPR_h_err(:, 1)/pi*180, 'r', 'linewidth', 2);
plot(t_h, YPR_h_err(:, 2)/pi*180, 'g', 'linewidth', 2);
plot(t_h, YPR_h_err(:, 3)/pi*180, 'b', 'linewidth', 2);
xlabel('Time [s]');
ylabel('Error [deg]');
ylim([-8 8]);
grid on;
set(gca, 'fontsize', 13);
xlim([0 t_max]);

lg_hd = legend('Yaw error', 'Pitch error', 'Roll error');

set(lg_hd, 'orientation', 'horizontal',...
           'position', [0.3 0.85 0.4 0.1000]);

tightfig(gcf);
saveas(gcf, [exp_path exp_name '_ypr_h_err_t.fig']);
% saveas(gcf, [exp_path exp_name '_ypr_h_err_t.pdf']);
img = getframe(gcf);
imwrite(img.cdata, [exp_path exp_name '_ypr_h_err_t.png']);



end