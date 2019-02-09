% 2018.6.3 永井 忠一 『順運動学』

% Joint variable
th1 = get(hTh1, 'value'); th2 = get(hTh2, 'value'); th3 = get(hTh3, 'value'); th4 = get(hTh4, 'value');
set(hTh1Txt, 'string', strcat(['theta1 = ', num2str(th1*(180/pi)), '[degree]']));
set(hTh2Txt, 'string', strcat(['theta2 = ', num2str(th2*(180/pi)), '[degree]']));
set(hTh3Txt, 'string', strcat(['theta3 = ', num2str(th3*(180/pi)), '[degree]']));
set(hTh4Txt, 'string', strcat(['theta4 = ', num2str(th4*(180/pi)), '[degree]']));

% Homogeneous Transformation matrix
T1 = RotateZ(th1)*RotateX(th2);
T2 = T1*Trans(0, l1, 0)*RotateX(th3);
T3 = T2*Trans(0, l2, 0)*RotateX(th4);
T4 = T3*Trans(0, l3, 0);

% Robot
joint1 = T1*horzcat(local.o, local.x, local.y, local.z);
link1 = horzcat(T1*local.o, T2*local.o);
joint2 = T2*horzcat(local.o, local.x, local.y, local.z);
link2 = horzcat(T2*local.o, T3*local.o);
joint3 = T3*horzcat(local.o, local.x, local.y, local.z);
link3 = horzcat(T3*local.o, T4*local.o);

% update
for i=1:4
  set(hLink(i), 'xdata', [link1(1,1), link1(1,2), nan, link2(1,1), link2(1,2), nan, link3(1,1), link3(1,2)]);
  set(hLink(i), 'ydata', [link1(2,1), link1(2,2), nan, link2(2,1), link2(2,2), nan, link3(2,1), link3(2,2)]);
  set(hLink(i), 'zdata', [link1(3,1), link1(3,2), nan, link2(3,1), link2(3,2), nan, link3(3,1), link3(3,2)]);

  set(hJoint1(i).x, 'xdata', [joint1(1,1) joint1(1,2)]); set(hJoint1(i).x, 'ydata', [joint1(2,1) joint1(2,2)]); set(hJoint1(i).x, 'zdata', [joint1(3,1) joint1(3,2)]);
  set(hJoint1(i).y, 'xdata', [joint1(1,1) joint1(1,3)]); set(hJoint1(i).y, 'ydata', [joint1(2,1) joint1(2,3)]); set(hJoint1(i).y, 'zdata', [joint1(3,1) joint1(3,3)]);
  set(hJoint1(i).z, 'xdata', [joint1(1,1) joint1(1,4)]); set(hJoint1(i).z, 'ydata', [joint1(2,1) joint1(2,4)]); set(hJoint1(i).z, 'zdata', [joint1(3,1) joint1(3,4)]);

  set(hJoint2(i).x, 'xdata', [joint2(1,1) joint2(1,2)]); set(hJoint2(i).x, 'ydata', [joint2(2,1) joint2(2,2)]); set(hJoint2(i).x, 'zdata', [joint2(3,1) joint2(3,2)]);
  set(hJoint2(i).y, 'xdata', [joint2(1,1) joint2(1,3)]); set(hJoint2(i).y, 'ydata', [joint2(2,1) joint2(2,3)]); set(hJoint2(i).y, 'zdata', [joint2(3,1) joint2(3,3)]);
  set(hJoint2(i).z, 'xdata', [joint2(1,1) joint2(1,4)]); set(hJoint2(i).z, 'ydata', [joint2(2,1) joint2(2,4)]); set(hJoint2(i).z, 'zdata', [joint2(3,1) joint2(3,4)]);

  set(hJoint3(i).x, 'xdata', [joint3(1,1) joint3(1,2)]); set(hJoint3(i).x, 'ydata', [joint3(2,1) joint3(2,2)]); set(hJoint3(i).x, 'zdata', [joint3(3,1) joint3(3,2)]);
  set(hJoint3(i).y, 'xdata', [joint3(1,1) joint3(1,3)]); set(hJoint3(i).y, 'ydata', [joint3(2,1) joint3(2,3)]); set(hJoint3(i).y, 'zdata', [joint3(3,1) joint3(3,3)]);
  set(hJoint3(i).z, 'xdata', [joint3(1,1) joint3(1,4)]); set(hJoint3(i).z, 'ydata', [joint3(2,1) joint3(2,4)]); set(hJoint3(i).z, 'zdata', [joint3(3,1) joint3(3,4)]);
end

drawnow

% update I.K.
set(hX, 'value', link3(1,2));
set(hXTxt, 'string', strcat(['x = ', num2str(link3(1,2)), '[m]']));
set(hY, 'value', link3(2,2));
set(hYTxt, 'string', strcat(['y = ', num2str(link3(2,2)), '[m]']));
set(hZ, 'value', link3(3,2));
set(hZTxt, 'string', strcat(['z = ', num2str(link3(3,2)), '[m]']));

alpha = th2 + th3 + th4;
set(hAlpha, 'value', alpha);
set(hAlphaTxt, 'string', strcat(['alpha = ', num2str(alpha*(180/pi)), '[degree]']));
