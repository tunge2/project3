syms L1 L2 L3 L4 L5 real
syms q1 q2 q3 q4 q5 q6 q7 real
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];
% set up elbow arm in symbolic form

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% POE %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elbow.H=[ez ey ey ex ey ex];
elbow.P=[L1*ez zz L2*ez L3*ez+L4*ex zz zz L5*ex];
elbow.joint_type=[0 0 0 0 0 0];
%elbow.q=sym([0 0 0 0 0 0]');
elbow.q=sym([q1 q2 q3 q4 q5 q6]');

elbow=fwdkiniter(elbow);

disp('T_{0T} from POE method');
disp(elbow.T)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% SDH %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% set up SDH parameters
elbow1.d=[L1 0 0 L4 0 L5]';
elbow1.a=[0 L2 L3 0 0 0]';
elbow1.alpha=sym([-pi/2 0 -pi/2 pi/2 -pi/2 0]');
%elbow1.theta=sym([0 -pi/2 0 0 0 0]');
elbow1.theta=sym([q1 q2-pi/2 q3 q4 q5 q6]');
% calculate T_{06}
elbow1=fwdkinsdh(elbow1);
% additional transformation to match with POE end effector frame
T6T=[[[0 0 1 ; 0 -1 0; 1 0 0] zeros(3,1)];[zeros(1,3) 1]];
%NEW ADJUSTMENT

% This should match with POE's T_{0T}
T0T_SDH=simplify(elbow1.T*T6T);

disp('T_{0T} from SDH method');
disp(T0T_SDH);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% MDH %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set up MDH parameters
elbow2.d=[L1 0 0 L4 0 0 L5]';
elbow2.a=[0 0 L2 L3 0 0 0 0]'; % this is a_{i-1}
elbow2.alpha=sym([0 -pi/2 0 -pi/2 pi/2 -pi/2 0]);
%elbow1.theta=sym([0 -pi/2 0 0 0 0]');
elbow2.theta=sym([q1 q2-pi/2 q3 q4 q5 q6 0]');
elbow2=fwdkinmdh(elbow2);
% additional transformation to match with POE end effector frame
T7T=[[[0 0 1; 0 -1 0; 1 0 0] zeros(3,1)];[zeros(1,3) 1]];
%NEW ADJUSTMENT

% This should match with POE's T_{0T|
T0T_MDH=simplify(elbow2.T*T7T);

disp('T_{0T} from MDH method');
disp(T0T_MDH);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% check %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('difference between POE and SDH forward kinematics');
simplify(elbow.T-T0T_SDH)

disp('difference between SDH and MDH forward kinematics');
simplify(T0T_SDH-T0T_MDH)



















