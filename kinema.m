% 2018.6.2 永井 忠一

clear all; close all;

% Translation, Rx/Ry/Rz
Trans = inline('[1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1]');
RotateZ = inline('[cos(th) -sin(th) 0 0; sin(th) cos(th) 0 0; 0 0 1 0; 0 0 0 1]');
RotateX = inline('[1 0 0 0; 0 cos(th) -sin(th) 0; 0 sin(th) cos(th) 0; 0 0 0 1]');
RotateY = inline('[cos(th) sin(th) 0 0; -sin(th) 1 0 0; 0 0 cos(th) 0; 0 0 0 1]');

% 4DOF manipulator (3 link)
l1 = 1; l2 = 1; l3 = 0.5; % [m]
th1 = 0*(pi/180); th2 = 0*(pi/180); th3 = 0*(pi/180); th4 = 0*(pi/180); % [radian]

% Homogeneous Transformation matrix
T1 = RotateZ(th1)*RotateX(th2); % joint 1 (2DOF)
T2 = T1*Trans(0, l1, 0)*RotateX(th3); % joint 2
T3 = T2*Trans(0, l2, 0)*RotateX(th4); % joint 3
T4 = T3*Trans(0, l3, 0); % end effector

local.o = [0; 0; 0; 1];
local.x = [0.25; 0; 0; 1];
local.y = [0; 0.25; 0; 1];
local.z = [0; 0; 0.25; 1];

% Robot
joint1 = T1*horzcat(local.o, local.x, local.y, local.z);
link1 = horzcat(T1*local.o, T2*local.o);
joint2 = T2*horzcat(local.o, local.x, local.y, local.z);
link2 = horzcat(T2*local.o, T3*local.o);
joint3 = T3*horzcat(local.o, local.x, local.y, local.z);
link3 = horzcat(T3*local.o, T4*local.o);

capture_count = 0;

% GUI
hWindow = figure();
set(hWindow, 'NumberTitle', 'off', 'name', '4DOF manipulator');
%{
myPosition = get(hWindow, 'Position');
myPosition(3:4) = [1024 768];
%}
%myPosition = [0 0 1200 900];
myPosition = [0 0 1280 768];
set(hWindow, 'Position', myPosition);

y = 5; height = 20;
y = y + height; uicontrol(hWindow, 'style', 'pushbutton', 'position', [10 y 160 height], 'string', 'capture', 'callback', 'print(strcat(''out'', num2str(capture_count, ''%04d''), ''.png''), ''-dpng''); capture_count = capture_count + 1;', 'HorizontalAlignment', 'center');
y = y + height;
y = y + height; hAlpha = uicontrol(hWindow, 'style', 'slider', 'min', -pi, 'max', pi, 'position', [10 y 160 height], 'callback', 'kinema_ik');
y = y + height; hAlphaTxt = uicontrol(hWindow, 'style', 'text', 'position', [10 y 160 height], 'string', 'alpha = [degree]', 'HorizontalAlignment', 'left');
y = y + height; hZ = uicontrol(hWindow, 'style', 'slider', 'min', -3, 'max', 3, 'position', [10 y 160 height], 'callback', 'kinema_ik');
y = y + height; hZTxt = uicontrol(hWindow, 'style', 'text', 'position', [10 y 160 height], 'string', 'z = [m]', 'HorizontalAlignment', 'left');
y = y + height; hY = uicontrol(hWindow, 'style', 'slider', 'min', -3, 'max', 3, 'position', [10 y 160 height], 'callback', 'kinema_ik');
y = y + height; hYTxt = uicontrol(hWindow, 'style', 'text', 'position', [10 y 160 height], 'string', 'y = [m]', 'HorizontalAlignment', 'left');
y = y + height; hX = uicontrol(hWindow, 'style', 'slider', 'min', -3, 'max', 3, 'position', [10 y 160 height], 'callback', 'kinema_ik');
y = y + height; hXTxt = uicontrol(hWindow, 'style', 'text', 'position', [10 y 160 height], 'string', 'x = [m]', 'HorizontalAlignment', 'left');
y = y + height; uicontrol(hWindow, 'style', 'text', 'position', [10 y 160 height], 'string', 'endpoint position', 'HorizontalAlignment', 'left');
y = y + height; uicontrol(hWindow, 'style', 'text', 'position', [10 y 160 height], 'string', 'Inverse Kinematics', 'HorizontalAlignment', 'left');
y = y + height;
y = y + height; hTh4 = uicontrol(hWindow, 'style', 'slider', 'min', -pi, 'max', pi, 'value', th4, 'position', [10 y 160 height], 'callback', 'kinema_fk');
y = y + height; hTh4Txt = uicontrol(hWindow, 'style', 'text', 'position', [10 y 160 height], 'string', strcat(['theta4 = ', num2str(th4*(180/pi)), '[degree]']), 'HorizontalAlignment', 'left');
y = y + height; hTh3 = uicontrol(hWindow, 'style', 'slider', 'min', -pi, 'max', pi, 'value', th3, 'position', [10 y 160 height], 'callback', 'kinema_fk');
y = y + height; hTh3Txt = uicontrol(hWindow, 'style', 'text', 'position', [10 y 160 height], 'string', strcat(['theta3 = ', num2str(th3*(180/pi)), '[degree]']), 'HorizontalAlignment', 'left');
y = y + height; hTh2 = uicontrol(hWindow, 'style', 'slider', 'min', -pi, 'max', pi, 'value', th2, 'position', [10 y 160 height], 'callback', 'kinema_fk');
y = y + height; hTh2Txt = uicontrol(hWindow, 'style', 'text', 'position', [10 y 160 height], 'string', strcat(['theta2 = ', num2str(th2*(180/pi)), '[degree]']), 'HorizontalAlignment', 'left');
y = y + height; hTh1 = uicontrol(hWindow, 'style', 'slider', 'min', -pi, 'max', pi, 'value', th1, 'position', [10 y 160 height], 'callback', 'kinema_fk');
y = y + height; hTh1Txt = uicontrol(hWindow, 'style', 'text', 'position', [10 y 160 height], 'string', strcat(['theta1 = ', num2str(th1*(180/pi)), '[degree]']), 'HorizontalAlignment', 'left');
y = y + height; uicontrol(hWindow, 'style', 'text', 'position', [10 y 160 height], 'string', 'joint variable', 'HorizontalAlignment', 'left');
y = y + height; uicontrol(hWindow, 'style', 'text', 'position', [10 y 160 height], 'string', 'Forward Kinematics', 'HorizontalAlignment', 'left');

% View x4
link = [link1(:,1)'; link1(:,2)';
	[nan nan nan nan];
	link2(:,1)'; link2(:,2)';
	[nan nan nan nan];
	link3(:,1)'; link3(:,2)'];

for i = 1:4
  hView(i) = subplot(2, 2, i);
  hLink(i) = plot3(hView(i), link(:,1), link(:,2), link(:,3), '-'); hold on;
  if i == 1
    xlabel('x'); ylabel('y'); grid on; axis equal; axis([-3, 3, -3, 3, -3, 3]); view(0, 90); title('top view');
  elseif i == 2
    xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal; axis([-3, 3, -3, 3, -3, 3]);
  elseif i == 3
    xlabel('x'); zlabel('z'); grid on; axis equal; axis([-3, 3, -3, 3, -3, 3]); view(0, 0); title('front view');
  else
    ylabel('y'); zlabel('z'); grid on; axis equal; axis([-3, 3, -3, 3, -3, 3]); view(90, 0); title('side view');
  end
end

for i = 1:4
  joint = [ joint1(:,1)'; joint1(:,2)' ];
  hJoint1(i).x = plot3(hView(i), joint(:,1), joint(:,2), joint(:,3), 'r-');
  joint = [ joint1(:,1)'; joint1(:,3)' ];
  hJoint1(i).y = plot3(hView(i), joint(:,1), joint(:,2), joint(:,3), 'g-');
  joint = [ joint1(:,1)'; joint1(:,4)' ];
  hJoint1(i).z = plot3(hView(i), joint(:,1), joint(:,2), joint(:,3), 'b-');

  joint = [ joint2(:,1)'; joint2(:,2)' ];
  hJoint2(i).x = plot3(hView(i), joint(:,1), joint(:,2), joint(:,3), 'r-');
  joint = [ joint2(:,1)'; joint2(:,3)' ];
  hJoint2(i).y = plot3(hView(i), joint(:,1), joint(:,2), joint(:,3), 'g-');
  joint = [ joint2(:,1)'; joint2(:,4)' ];
  hJoint2(i).z = plot3(hView(i), joint(:,1), joint(:,2), joint(:,3), 'b-');
  
  joint = [ joint3(:,1)'; joint3(:,2)' ];
  hJoint3(i).x = plot3(hView(i), joint(:,1), joint(:,2), joint(:,3), 'r-');
  joint = [ joint3(:,1)'; joint3(:,3)' ];
  hJoint3(i).y = plot3(hView(i), joint(:,1), joint(:,2), joint(:,3), 'g-');
  joint = [ joint3(:,1)'; joint3(:,4)' ];
  hJoint3(i).z = plot3(hView(i), joint(:,1), joint(:,2), joint(:,3), 'b-');
end

kinema_fk;

%{
function next_count = capture(count)
  next_count = count + 1;
end
%}
