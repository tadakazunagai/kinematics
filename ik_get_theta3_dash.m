function retval = ik_get_theta3_dash(l1, l2, x, y)
  retval = pi + atan2(sqrt(1 - ((-(x^2 + y^2) + l1^2 + l2^2) / (2*l1*l2))^2), (-(x^2 + y^2) + l1^2 + l2^2) / (2*l1*l2));
end
