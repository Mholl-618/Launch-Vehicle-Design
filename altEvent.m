function [position,isterminal,direction] = altEvent(t,x)
position = x(1)-40e+05; %want this to be zero
isterminal = 1;
direction = 0; %Approach from any direction
end