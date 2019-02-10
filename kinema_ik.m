% 2018.6.11 永井 忠一 『逆運動学』

x = get(hX, 'value');
y = get(hY, 'value');
z = get(hZ, 'value');
alpha = get(hAlpha, 'value');

last_th1 = th1;
th1 = ik_theta1(x, y);
[x_ddash, z_dash] = ik_p_dash(ik_x_dash(x, y), z, l3, alpha);

if ((x_ddash == 0.0) && (z_dash == 0.0)) || (sqrt(x_ddash^2 + z_dash^2) > (l1 + l2)) % singularity
  th1 = last_th1; % keep last pose
else
  % multiple solutions
  last_th2 = th2;
  last_th3 = th3;
  % elbow down
  th2 = ik_theta2(l1, l2, x_ddash, z_dash);
  th3 = ik_theta3(l1, l2, x_ddash, z_dash);
  % elbow up
  th2_dash = ik_theta2_dash(l1, l2, x_ddash, z_dash);
  th3_dash = ik_theta3_dash(l1, l2, x_ddash, z_dash);
  if (abs(last_th2 - th2) + abs(last_th3 - th3)) > (abs(last_th2 - th2_dash) + abs(last_th3 - th3_dash)) % cost function: SAD(Sum of Absoluted Difference)
    th2 = th2_dash;
    th3 = th3_dash;
  end
  th4 = ik_theta4(alpha, th2, th3);
end

% call F.K.
set(hTh1, 'value', th1); set(hTh2, 'value', th2); set(hTh3, 'value', th3); set(hTh4, 'value', th4);

kinema_fk;
