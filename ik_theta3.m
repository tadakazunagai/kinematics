function retval = ik_theta3(l1, l2, x, y)
  retval = pi - atan2(sqrt(1 - ((-(x^2 + y^2) + l1^2 + l2^2) / (2*l1*l2))^2), (-(x^2 + y^2) + l1^2 + l2^2) / (2*l1*l2));
end

% 別解
%{
function retval = ik_theta3(l1, l2, x_ddash, z_dash)
  retval = atan2(sqrt(1 - ((x_ddash^2 + z_dash^2 - l1^2 - l2^2) / (2*l1*l2))^2), (x_ddash^2 + z_dash^2 - l1^2 - l2^2) / (2*l1*l2));
end
%}
