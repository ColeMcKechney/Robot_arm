function robot_arm_Sumith_v1()
clc;clear;

d1 = 4.2;

x0 = [0; 0 ; 0; 0];
x1 = [d1; 0 ; 0; 0];

rotate3d on;


for i = 0:10:360
    x = [x0(1) x1(1)];
    y = [x0(2) x1(2)];
    z = [x0(3) x1(3)];
    
    x1 = rot_z(x1,10);
        
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

function rotated = rot_z(mat_to_rot, theta)
rotated = [cosd(theta) -sind(theta) 0 0; sind(theta) cosd(theta) 0 0; 0 0 1 0; 0 0 0 1] * mat_to_rot;
end
