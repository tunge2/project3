% implementation of inverse kinematic code with example code
%
% 
clear all
close all

zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

load S_sphere_path %r is in here

% plot the spherical S
figure(1);plot3(p_S(1,:),p_S(2,:),p_S(3,:),'rx','linewidth',3);
xlabel('x');ylabel('y');zlabel('z');
hold on;
% add a 3d sphere
surf(X,Y,Z)
% make it transparent
alpha .5
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);

% convert to equal path length grid
diffS=vecnorm(diff(p_S')');
ls=[0 cumsum(diffS)];
lf=sum(diffS);
N=100;
l=(0:lf/N:lf);

pS=interp1(ls,p_S',l,'spline')'; %linear interp, evenly spaced pS

% plot it out again with equal path length
figure(2);plot3(pS(1,:),pS(2,:),pS(3,:),'rx','linewidth',3);
xlabel('x');ylabel('y');zlabel('z');
hold on;
% 3d sphere
surf(X,Y,Z)
% make it transparent
alpha 0.4
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);

% check the path length is indeed equal
dlvec=vecnorm(diff(pS')');
figure(3);plot(dlvec,'x')
dl=mean(dlvec);
disp(max(abs(dlvec-dl)));

% save it as the path file
save S_sphere_path_uniform l pS

% find the end effector frame
pc=r*ez;
N=length(pS);
xT=zeros(3,N);zT=zeros(3,N);yT=zeros(3,N);
quat=zeros(4,N); % allocate space for unit quaternion representation of R_{0T}
euler=zeros(3,N);
euler2=zeros(3,N); %2 solutions to euler angles
ktheta=zeros(4,N);

for i=1:N
    xT(:,i)=(pS(:,i)-pc)/norm(pS(:,i)-pc);
    
    if i<N
        yT(:,i)=(pS(:,i+1)-pS(:,i));
    else
        yT(:,i)=yT(:,i-1);
    end
    yT(:,i)=yT(:,i)-yT(:,i)'*xT(:,i)*xT(:,i);
    yT(:,i)=yT(:,i)/norm(yT(:,i));
    
    zT(:,i)=cross(xT(:,i),yT(:,i));
    
    R=[xT(:,i) yT(:,i) zT(:,i)];
    
    quat(:,i)=R2quat(R);
    [firstsol, secondsol] = R2euler(R); %selected b2 within [0 pi]
    euler(:,i)=firstsol';
    euler2(:,i)=secondsol';

    ktheta(:,i)=R2kth(R); %stored as k1 k2 k3 theta
end

% plot out the end effector frame
m=1;
% MATLAB's plotTransforms command plots a frame at a given location
figure(4);
h=plotTransforms(pS(:,1:m:end)',quat(:,1:m:end)');
set(h,'LineWidth',.5);

% ABB IRB 1200 parameters

L1=399.1;
L2=350;
L3=42;
L4=351;

L5=82;

% P
p01=0*ex+L1*ez;
p12=zz;
p23=L2*ez;
p34=L3*ez+L4*ex;
p45=zz;
p56=zz;
p6T=L5*ex;

% H
h1=ez;
h2=ey;
h3=ey;
h4=ex;
h5=ey;
h6=ex;

% Final transformation
R6T=[-ez ey ex];

% define abb 1200 robot using POE convention
irb1200.P=[p01 p12 p23 p34 p45 p56 p6T]/1000;
irb1200.H=[h1 h2 h3 h4 h5 h6];
irb1200.joint_type=[0 0 0 0 0 0];
%irb1200.R6T=R6T;

% define collision body for abb 1200
radius=.01;
[irb1200_rbt,colLink]=defineRobot(irb1200,radius);
 
% 
% choose the inverse kinematics solution
%
Td = cell(1,101);
q = zeros(6,101);
qall = zeros(6,101,8);
ksol=3;

for j=1:8
    for i=1:N
        % specify end effector SE(3) frame
        %Td{i}=[[xT(:,i) yT(:,i) zT(:,i)]*R6T' pS(:,i);[0 0 0 1]];
        Td{i}=[[xT(:,i) yT(:,i) zT(:,i)] pS(:,i);[0 0 0 1]];
        irb1200.T=Td{i};
        %
        irb1200=invkinelbow(irb1200); % << you need to supply this!!!
        %totalcheck = fwdkiniter(irb1200)
        
        q(:,i)=irb1200.q(:,j);
        qall(:,i,j) = q(:,i);
        
        % check forward kinematics to make sure the IK solution is correct
        irb1200.q=q(:,i);
        irb1200=fwddiffkiniter(irb1200);
        T{i}=irb1200.T;
        % show robot pose (ever 5 frames)
%         if mod(i,1)==0
%             %disp(norm(T{i}-Td{i})); %displays end effector 2-norm error 
%             figure(5);
%             show(irb1200_rbt,q(:,i),'collision','on');        
%             view(150,10);
%         end
    end
end

%plotthemreps(l, quat, euler, euler2, ktheta)
%problem3(qall, l)

% R2q.m
%
% converts R in SO(3) to unit quaternion q, (q0,q_vec)

function k = vee(K)

k=[-K(2,3);K(1,3);-K(1,2)];

end

function plotthemreps(l, quat, euler, euler2, ktheta)
	figure(111)
    plot(l, quat(1,:),'linewidth',2)
    hold on
    plot(l, quat(2,:),'linewidth',2)
    plot(l, quat(3,:),'linewidth',2)
    plot(l, quat(4,:),'linewidth',2)

    title('Quaternion Representation of Robot Rotation Matricies')
    xlabel('Lambda (m)')
    ylabel('Coefficient Value')
    legend('q_0', 'q_1', 'q_2', 'q_3','location','best')
    grid on
    hold off
    
    figure(222)
    plot(l, euler(1,:),'linewidth',2)
    hold on
    plot(l, euler(2,:),'linewidth',2)
    plot(l, euler(3,:),'linewidth',2)

    title('ZYX Euler Angle Representation of Robot Rotation Matricies')
    xlabel('Lambda (m)')
    ylabel('Angle (rad)')
    legend('beta_1', 'beta_2', 'beta_3', 'location','best')
    grid on
    hold off
    
    figure(2222)
    plot(l, euler2(1,:),'linewidth',2)
    hold on
    plot(l, euler2(2,:),'linewidth',2)
    plot(l, euler2(3,:),'linewidth',2)

    title('ZYX Euler Angle Representation of Robot Rotation Matricies')
    xlabel('Lambda (m)')
    ylabel('Angle (rad)')
    legend('beta_1', 'beta_2', 'beta_3', 'location','best')
    grid on
    hold off
    
    figure(333)
    yyaxis left
    plot(l, ktheta(1,:),'linewidth',2)
    hold on
    plot(l, ktheta(2,:),'linewidth',2)
    plot(l, ktheta(3,:),'linewidth',2)
    ylabel('Rotation Axis Component ([0])')

    yyaxis right
    plot(l, ktheta(4,:),'linewidth',2)
    ylabel('Rotation Angle (rad)')

    title('Axis-Angle Product Representation of Robot Rotation Matricies')
    xlabel('Lambda (m)')
    legend('k_x', 'k_y', 'k_z', 'theta', 'location','best')
    grid on
    hold off
end

function problem3(qall, l)

for i=1:8
    y = qall(:,:,i);
    maketheqplot(i*10101,l,y,['Arm Angles vs Lambda - Solution ' num2str(i)],'Lambda (m)', 'Joint Angle (rad)')
end

end

function maketheqplot(fnum,x,y,tit,xlab,ylab)
    figure(fnum)
    plot(x, y(1,:),'linewidth',2)
    hold on
    plot(x, y(2,:),'linewidth',2)
    plot(x, y(3,:),'linewidth',2)
    plot(x, y(4,:),'linewidth',2)
    plot(x, y(5,:),'linewidth',2)
    plot(x, y(6,:),'linewidth',2)

    title(tit)
    xlabel(xlab)
    ylabel(ylab)
    legend('q1','q2','q3','q4','q5','q6','location','best')
    grid on
    hold off

end