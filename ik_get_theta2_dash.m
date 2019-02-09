function retval = ik_get_theta2_dash(l1, l2, x, y)
  retval = atan2(y, x) - atan2(l2 * sin(ik_get_theta3_dash(l1, l2, x, y)), l1 + l2*cos(ik_get_theta3_dash(l1, l2, x, y)));
end
