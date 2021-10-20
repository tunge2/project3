clear all
close all

L1=399.1;L2=350;L3=42;L4=351;L5=82;
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

%problem 2 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% P O E %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% define abb 1200 robot using POE convention
irb1200.P=[L1*ez zz L2*ez L3*ez+L4*ex zz zz L5*ex]/1000;
irb1200.H=[ez ey ey ex ey ex];
irb1200.joint_type=[0 0 0 0 0 0];

[irb1200_POE,~]=defineRobot(irb1200,0.05);

%irb1200.q=q; % do we even need this?

figure(1);
show(irb1200_POE,[0 0 0 0 0 0]','collision','on');        
view(20,10);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% S D H %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

irb1200.d=[L1;L2;L3;L4;0;L5]/1000;
irb1200.a=[0;L2;L3;0;0;0];
irb1200.alpha=sym([-pi/2;0;-pi/2;pi/2;-pi/2;0]);
irb1200.theta=[0;-pi/2;0;pi;pi;0];
% calculate T_{06}
irb1200=fwdkinsdh(irb1200);

[irb1200_SDH,~]=defineRobot(irb1200,0.001);

figure(2);
show(irb1200_SDH,[0 0 0 0 0 0]','collision','on');        
view(20,10);












