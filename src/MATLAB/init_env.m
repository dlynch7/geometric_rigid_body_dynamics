%% init_env.m
%
% Description:
%   Initialized MATLAB environment for current user session. Mainly sets
%   LaTeX as the default interpreter and specifies a larger font size for
%   nicer-looking figures.
%
%   Also adds dependencies to PATH.
%
% Inputs:
%   none
%
% Outputs:
%   none


function init_env

    set(groot,'defaultLegendInterpreter','latex');
    set(groot,'defaultTextInterpreter','latex');
    set(groot,'defaultAxesTickLabelInterpreter','latex');
    % make text large enough to read:
    set(groot,'DefaultAxesFontSize',16);
    
    addpath('~/ModernRobotics/packages/MATLAB/mr');

end