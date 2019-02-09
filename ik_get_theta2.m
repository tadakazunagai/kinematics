function retval = ik_get_theta2(l1, l2, x, y)
  retval = atan2(y, x) - atan2(l2 * sin(ik_get_theta3(l1, l2, x, y)), l1 + l2*cos(ik_get_theta3(l1, l2, x, y)));
end

%{
function retval = ik_get_theta2(l1, l2, x_ddash, z_dash)
  retval = atan2(z_dash, x_ddash) - atan2(sqrt(1 - ((x_ddash^2 + z_dash^2 + l1^2 - l2^2) / (2*l1*sqrt(x_ddash^2 + z_dash^2)))^2), (x_ddash^2 + z_dash^2 + l1^2 - l2^2) / (2*l1*sqrt(x_ddash^2 + z_dash^2)));
end
%}
