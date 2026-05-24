A = load( [prp '/matlab/generated/Alon.mat'] ).A;
B = load( [prp '/matlab/generated/Blon.mat'] ).B;
c = getConstants();
A = A(1:4, 1:4);
B = B(1:4,:);

% Build the transfer functions from the 4-state state-space model.
% Directly inverting (sI - A) as a tf object keeps uncancelled repeated
% polynomial factors and can make a 4-state model look much higher order.
C = eye(4);
D = zeros(4,1);
tfs_ = tf(ss(A, B(:,c.lon.ELE_IDX), C, D));

% Store the named transfer functions with MATLAB struct fields.
tfs = struct();
tfs.vt2ele = tfs_(c.lon.VT_IDX,1);
tfs.th2ele = tfs_(c.lon.TH_IDX,1);
tfs.alf2ele = tfs_(c.lon.ALF_IDX,1);
tfs.q2ele = tfs_(c.lon.Q_IDX,1);

% tfs_ = tf(ss(A, B(:,c.lon.THTL_IDX), eye(4), zeros(4,1)));
% tfs.vt2thtl = tfs_(c.lon.VT_IDX,1);
% tfs.th2thtl = tfs_(c.lon.TH_IDX,1);
% tfs.alf2thtl = tfs_(c.lon.ALF_IDX,1);
% tfs.q2thtl = tfs_(c.lon.Q_IDX,1);
