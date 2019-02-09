% 2018.6.11 永井 忠一 『逆運動学』

x = get(hX, 'value');
y = get(hY, 'value');
z = get(hZ, 'value');
alpha = get(hAlpha, 'value');

%{
% old Octeve ver code
get_theta1 = inline('atan2(-x, y)');
get_x_dash = inline('sqrt(x^2 + y^2)');
get_p_dash = inline('[(x_dash - l3*cos(alpha)), (z - l3*sin(alpha))]', 'x_dash, z, l3, alpha');
get_theta2 = inline('atan2(z_dash, x_ddash) - atan2(sqrt(1 - ((x_ddash^2 + z_dash^2 + l1^2 - l2^2) / (2*l1*sqrt(x_ddash^2 + z_dash^2)))^2), (x_ddash^2 + z_dash^2 + l1^2 - l2^2) / (2*l1*sqrt(x_ddash^2 + z_dash^2)))',
		    'l1, l2, x_ddash, z_dash');
get_theta3 = inline('atan2(sqrt(1 - ((x_ddash^2 + z_dash^2 - l1^2 - l2^2) / (2*l1*l2))^2), (x_ddash^2 + z_dash^2 - l1^2 - l2^2) / (2*l1*l2))',
		    'l1, l2, x_ddash, z_dash');
get_theta4 = inline('alpha - theta2 - theta3',
		    'alpha, theta2, theta3');

th1 = get_theta1(x, y); set(hTh1, 'value', th1);
p_dash = get_p_dash(get_x_dash(x, y), z, l3, alpha);
th2 = get_theta2(l1, l2, p_dash(1), p_dash(2));
th3 = get_theta3(l1, l2, p_dash(1), p_dash(2));
th4 = get_theta4(alpha, th2, th3);
%}

last_th1 = th1;
th1 = ik_get_theta1(x, y);
[x_ddash, z_dash] = ik_get_p_dash(ik_get_x_dash(x, y), z, l3, alpha);

if ((x_ddash == 0.0) && (z_dash == 0.0)) || (sqrt(x_ddash^2 + z_dash^2) > (l1 + l2)) % singularity
  th1 = last_th1; % keep last pose
else
  % multiple solutions
  last_th2 = th2;
  last_th3 = th3;
  % elbow down
  th2 = ik_get_theta2(l1, l2, x_ddash, z_dash);
  th3 = ik_get_theta3(l1, l2, x_ddash, z_dash);
  % elbow up
  th2_dash = ik_get_theta2_dash(l1, l2, x_ddash, z_dash);
  th3_dash = ik_get_theta3_dash(l1, l2, x_ddash, z_dash);
  if (abs(last_th2 - th2) + abs(last_th3 - th3)) > (abs(last_th2 - th2_dash) + abs(last_th3 - th3_dash)) % cost function: SAD(Sum of Absoluted Difference)
    th2 = th2_dash;
    th3 = th3_dash;
  end
  th4 = ik_get_theta4(alpha, th2, th3);
end

% call F.K.
set(hTh1, 'value', th1); set(hTh2, 'value', th2); set(hTh3, 'value', th3); set(hTh4, 'value', th4);

kinema_fk;
