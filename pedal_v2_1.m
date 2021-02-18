clear all
clc
home = [0 0];
step = 3;
deadzone = 3;
while 1
    r_temp = 1;
    velocity = input('input velocity(cm/step):');
    angle = input('input angle(degree):');
    %fprintf('%s  %s\n',velocity, angle)
    target = [step*velocity*sind(angle)/100 step*velocity*cosd(angle)/100]; %코싸반대
    displacement = target - home;
    displacement_x = linspace(0,displacement(1),step+1);
    displacement_y = linspace(0,displacement(2),step+1);
    figure(1)
    plot(displacement_x,displacement_y,'--','color', 'b','linewidth',1)
    %quiver(home(1),home(2),displacement(1),displacement(2),'linewidth',2,'MaxHeadSize',0.3)
    hold on
    r = step*velocity/100;
    theta = linspace(0, 2*pi, 100);
    max_x = cos(theta);
    max_y = sin(theta);
    %plot(max_x,max_y,'color','k')
    x = r*cos(theta);
    y = r*sin(theta);
    %plot(x,y,'-.','color','k', 'linewidth', 1)
    home2 = [step*velocity/100/2/sind(angle) 0];
    if(angle ~= 0)
        plot(home2(1),0,'.', 'MarkerSize',5, 'color', 'k')
    end
    
    %plot
    if(abs(angle) > deadzone) %rotation
        r2 = abs(home2(1));
        x2 = r2*cos(theta) + home2(1)*ones(1,100);
        y2 = r2*sin(theta) + home2(2)*ones(1,100);
        plot(x2,y2,'--','color','k','linewidth',1)
        if(velocity > 0)
            if(angle > deadzone)
                traj_theta = linspace(180, 180-2*angle, 100);
                foot_theta = linspace(180, 180-2*angle, step+1);
            elseif(angle < deadzone)
                r_temp = -1*r_temp;
                traj_theta = linspace(0,-1*2*angle, 100);
                foot_theta = linspace(0,-1*2*angle, step+1);
            end
        elseif(velocity < 0)
            r_temp = -1*r_temp;
            if(angle > deadzone)
                traj_theta = linspace(0, -1*2*angle, 100);
                foot_theta = linspace(0, -1*2*angle, step+1);
            elseif(angle < deadzone)
                r_temp = -1*r_temp;
                traj_theta = linspace(180,180-2*angle, 100);
                foot_theta = linspace(180,180-2*angle, step+1);
            end
        end
        x3 = r2*cosd(traj_theta) + home2(1)*ones(1,100);
        y3 = r2*sind(traj_theta) + home2(2)*ones(1,100);
        plot(x3,y3,'--','color','r','linewidth',1)
        
        r3 = r2 + 0.127794;
        tx1 = r3*cosd(traj_theta) + home2(1)*ones(1,100);
        ty1 = r3*sind(traj_theta) + home2(2)*ones(1,100);
        plot(tx1,ty1,'color','g','linewidth',3)
        
        r4 = r2 - 0.127794;
        tx2 = r4*cosd(traj_theta) + home2(1)*ones(1,100);
        ty2 = r4*sind(traj_theta) + home2(2)*ones(1,100);
        plot(tx2,ty2,'color','c','linewidth',3)
        
        for i=1:step+1
           fx= (r2 + 0.127794*r_temp)*cosd(foot_theta(i)) + home2(1);
           fy= (r2 + 0.127794*r_temp)*sind(foot_theta(i)) + home2(2);
           plot(fx,fy,'.','Marker','.','MarkerSize',5, 'color', 'k')
           footdraw(fx,fy,180-foot_theta(i))
           r_temp = -1*r_temp;
        end
    else %straight
        foot_l_x= displacement_x - 0.127794;
        foot_l_y= displacement_y;
        plot(foot_l_x,foot_l_y,'color','g','linewidth',3)
        foot_r_x= displacement_x + 0.127794;
        foot_r_y= displacement_y;        
        plot(foot_r_x,foot_r_y,'color','c','linewidth',3)
        for i=1:step+1
            if(r_temp == 1)
                plot(foot_l_x(i),foot_l_y(i),'.','Marker','.','MarkerSize',5,'color','k')
                footdraw(foot_l_x(i),foot_l_y(i),0)
            else
                plot(foot_r_x(i),foot_r_y(i),'.','Marker','.','MarkerSize',5,'color','k')
                footdraw(foot_r_x(i),foot_r_y(i),0)
            end
            r_temp = -1*r_temp;
        end
    end
    
    grid
    axis equal
    axis([-1.2 1.2 -1.2 1.2])
    hold off
end
