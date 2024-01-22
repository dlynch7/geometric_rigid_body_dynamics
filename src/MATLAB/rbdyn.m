%% rbdyn.m
%
% Description:
%   This function computes the forward dynamics of a rigid body whose
%   configuration is an element of SE(3) and whose twist is an element of
%   se(3), the Lie algebra of SE(3). This function computes the dynamics in
%   the body frame rather than the world frame.
%
% Inputs:
%   t: time (scalar)
%   Twb: configuration of the rigid body's CoM frame {b} relative to the
%        world frame {w}, represented as a 4x4 homogeneous transformation
%        matrix, i.e., an element of SE(3).
%   Vb:  body twist, represented by a 6x1 vector whose elements are the
%        components of an element of se(3)
%   Fb:  net external body wrench, represented by a 6x1 vector whose
%        elements are the components of an element of dse(3), the dual
%        space to se(3).
%   params: a struct with many fields, including inertial parameters,
%        generated by calling init_params().
%
% Outputs:
%   dVbdt: derivative of body twist Vb with respect to time (a 6x1 vector)

function dVbdt = rbdyn(t,Twb,Vb,Fb,params)

Gb = params.dyn.spatial_inertia_matrix_CoM; % spatial inertia matrix in {b}
Gb_inv = diag(1./diag(Gb)); % no need to call inv() to compute Gb_inv

% To compute wrench due to gravity, easy to define a frame {c} whose origin
% is that of {b} but whose orientation is that of {w}.
Twc = [eye(3),     Twb(1:3,4);
       zeros(1,3), 1];
Tcw = TransInv(Twc);
Tcb = Tcw*Twb;
Fc = [0;0;0;0;0;-params.dyn.mass*params.dyn.grav];

Fb = Fb + transpose(Adjoint(Tcb))*Fc;


dVbdt = Gb_inv*(transpose(ad(Vb))*Gb*Vb + Fb);

end