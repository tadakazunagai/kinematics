function [x_ddash, z_dash] = ik_p_dash(x_dash, z, l3, alpha)
  x_ddash = x_dash - l3*cos(alpha);
  z_dash = z - l3*sin(alpha);
end
