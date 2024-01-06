%% main.m
%
% Description:
%   Program entry point. Calls various functions.
%
% Dependencies:
%   Modern Robotics
%
% Inputs:
%   none
%
% Outputs:
%   none

function main
%% Initialization
clear;
close all;
clc;

init_env;
body_name = 'rect_prism';
dimensions.rect_prism.l = 1e-1;
dimensions.rect_prism.w = 1e-1;
dimensions.rect_prism.h = 1e-1;
params = init_params(body_name,dimensions);

%% Set initial conditions and numerical integration parameters
Rwb_init = eye(3);
Twb_init = [Rwb_init,   zeros(3,1);
            zeros(1,3), 1]; % CoM frame coincident with world frame
Vb_init = [10;-10;10;0;0;0];
Fb = zeros(6,1);
        
dt = 1e-3; % timestep size
t0 = 0;
tf = 1; % simulate for 1 second
t = t0:dt:tf;

%% Simulate via numerical integration
Twb{1} = Twb_init;
Vb{1} = Vb_init;
for k = 1:numel(t)-1
    [~,Twb{k+1},Vb{k+1}] = cg4(dt,t(k),Twb{k},Vb{k},Fb,params);
end

%% Postprocess
% kinematics:
Twb_hist = NaN(4,4,numel(t));
Vb_hist = NaN(6,numel(t));
p_hist = NaN(3,numel(t));
v_hist = NaN(3,numel(t));

% determinant of Rwb:
detRwb_hist = NaN(1,numel(t));

% energy and momenta:
E_hist = NaN(1,numel(t)); % mechanical energy
L_hist = NaN(3,numel(t)); % angular momentum in body frame
P_hist = NaN(3,numel(t)); % linear momentum in body frame

for k = 1:numel(t)
    Rwb = Twb{k}(1:3,1:3); % orientation of {b} in {w}
    pwb = Twb{k}(1:3,4);   % position of origin of {b} in {s}
    
    % energy
    Gb = params.dyn.spatial_inertia_matrix_CoM;
    KE = 0.5*transpose(Vb{k})*Gb*Vb{k};
    PE = params.dyn.mass*params.dyn.grav*pwb(3);
    E = KE + PE;
    
    % momentum:
    Pb = Gb*Vb{k};
    Pw = transpose(Adjoint(TransInv(Twb{k})))*Gb*Vb{k};
    L_hist(:,k) = [eye(3),zeros(3)]*Pw; % angular momentum
    P_hist(:,k) = [zeros(3),eye(3)]*Pw; % linear momentum
    
    % velocity of CoM in world frame:
    v = Rwb*Vb{k}(4:6);
    
    Twb_hist(:,:,k) = Twb{k};
    Vb_hist(:,k) = Vb{k};
    p_hist(:,k) = pwb;
    v_hist(:,k) = v;
    detRwb_hist(k) = det(Rwb);
    E_hist(k) = E;
end

%% Visualize simulation results
% handy diagnostic functions:
%   total energy
%   angular momenta
%   linear momenta in x and y (but not z)
%   determinant of Rwb (how close is it to 1?)

figure;
plot(t,detRwb_hist,'k-','LineWidth',2)
ylabel('$\mathrm{det}\left(R_\mathrm{wb}\right)$')
xlabel('time $t$ [s]')

figure;
subplot(3,1,1)
plot(t,E_hist,'k-','LineWidth',2)
ylabel('Energy [J]')

subplot(3,1,2)
plot(t,L_hist(1,:),'r-',...
     t,L_hist(2,:),'g-',...
     t,L_hist(3,:),'b-',...
    'LineWidth',2)
legend('$L_x$','$L_y$','$L_z$','Location','Best')
ylabel('Angular momentum [kg m$^2$/s]')

subplot(3,1,3)
plot(t,P_hist(1,:),'r-',...
     t,P_hist(2,:),'g-',...
     t,P_hist(3,:),'b-',...
    'LineWidth',2)
legend('$P_x$','$P_y$','$P_z$','Location','Best')
ylabel('Linear momentum [kg m/s]')
xlabel('time $t$ [s]')

figure;
subplot(2,2,1)
plot(t,squeeze(Twb_hist(1,1,:)),'r-',...
     t,squeeze(Twb_hist(1,2,:)),'g-',...
     t,squeeze(Twb_hist(1,3,:)),'b-',...
     t,squeeze(Twb_hist(2,1,:)),'r--',...
     t,squeeze(Twb_hist(2,2,:)),'g--',...
     t,squeeze(Twb_hist(2,3,:)),'b--',...
     t,squeeze(Twb_hist(3,1,:)),'r:',...
     t,squeeze(Twb_hist(3,2,:)),'g:',...
     t,squeeze(Twb_hist(3,3,:)),'b:',...
     'LineWidth',2);
legend('$R_{11}$','$R_{12}$','$R_{13}$',...
       '$R_{21}$','$R_{22}$','$R_{23}$',...
       '$R_{31}$','$R_{32}$','$R_{33}$',...
       'Location','Best')
ylabel('orientation $R_\mathrm{wb}$');

subplot(2,2,3)
plot(t,Vb_hist(1,:),'r-',...
     t,Vb_hist(2,:),'g-',...
     t,Vb_hist(3,:),'b-',...
    'LineWidth',2);
legend('$\omega_x$','$\omega_y$','$\omega_z$','Location','Best')
ylabel('angular velocity $\omega$ [rad/s] in $\left\{b\right\}$');
xlabel('time $t$ [s]')

subplot(2,2,2)
plot(t,p_hist(1,:),'r-',...
     t,p_hist(2,:),'g-',...
     t,p_hist(3,:),'b-',...
     'LineWidth',2);
legend('$p_x$','$p_y$','$p_z$','Location','Best')
ylabel('position $p$ [m]');

subplot(2,2,4)
plot(t,v_hist(1,:),'r-',...
     t,v_hist(2,:),'g-',...
     t,v_hist(3,:),'b-',...
    'LineWidth',2);
legend('$v_x$','$v_y$','$v_z$','Location','Best')
ylabel('velocity $v$ [m/s] in $\left\{w\right\}$');
xlabel('time $t$ [s]')

end