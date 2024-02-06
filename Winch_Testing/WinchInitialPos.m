function x0 = WinchInitialPos(l0, L_s, L_min, L_max, d, h0)
% Function for calculating the Initial Linear Displacement x0 of a cable
% on a grooved spool. Made by Magnus Grøterud
%
% - l0      : Initial cable length
% - L_s     : Horizontal Length of spool
% - L_min   : Minimum length of TOTAL cable length
% - L_max   : Maximum length of TOTAL cable length
% - d       : Horizontal Cable Length Between Two Pulleys
% - h0       : Vertical Cable Length Between Pulley and Winch (when angle alpha = 0)
%
% 





% Total Initial Cable Length (Currently
L = l0 + d + h;

% Mapping
L_map = interp1([L_min, L_max], [0, 1], L);

% Initial Linear Displacement 
x0 = L_map*L_s;
x0 = interp1([0,1], [-L_s/2, L_s/2], x0);





