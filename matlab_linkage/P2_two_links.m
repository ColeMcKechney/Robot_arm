function robot_arm_Sumith_v2()
clc;clear;

d1 = 2.2;
d2 = 4;

x0 = [0; 0 ; 0; 0];
x1 = [d1; 0 ; 0; 0];
x2 = [d1+d2; 0; 0; 0];

rotate3d on;


for i = 0:10:360
    x = [x0(1) x1(1) x2(1)];
    y = [x0(2) x1(2) x2(2)];
    z = [x0(3) x1(3) x2(3)];
    
    Rz = rot_z(10);
    x1= Rz*x1;
    
    x2 = rot_z(10)*rot_z(24)*(x2);
    
    plot3(x,y,z,'linewidth',2,'color','blue')
    hold on;
    xlim([-10, 10]);
    ylim([-10, 10]);
    zlim([-10, 10]);
    scatter3(x,y,z, 50,'red','filled');
    view(45,45)
    hold off;
    pause(0.1);
    i
end

disp('stop thats all');

end

function Rx = rot_x(theta)
Rx = [1  0 0 0;...
    0 cosd(theta) -sind(theta) 0;...
    0 sind(theta) cosd(theta) 0; ...
    0 0 0 1];
end

function Ry = rot_y(theta)
Ry = [cosd(theta) 0 -sind(theta) 0;...
    0 1 0 0;...
    sind(theta) 0 cosd(theta) 0; ...
    0 0 0 1];
end

function Rz = rot_z(theta)
Rz = [cosd(theta) -sind(theta) 0 0; ...
    sind(theta) cosd(theta) 0 0; ...
    0 0 1 0; ...
    0 0 0 1];
end
