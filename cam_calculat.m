%计算并绘制实际凸轮 | Calculate and plot the actual cam
%所有单位均为mm | All units are in millimeters
%%
clc;
clear;
%%
data_name='Trajectory_A';%需要启用的数据名称，便于快速基于以往数据计算凸轮 | Name of dataset, used to calculate cam based on previous data
save_name='TrajectoryA';%生成凸轮点集的文件名，文件会保存在上方路径(当前文件夹)内 | Filename of generated cam point set, saved in current working directory
format='.txt';%生成的凸轮点集文件的后缀，Creo为'.ibl'，SolidWorks为'.txt'，CAD为'.scr' | File extension for cam point set: '.ibl' for Creo, '.txt' for SolidWorks, '.scr' for CAD
num_route=20;%生成点集时插值点数量 | Number of interpolation points for point set generation

%%
load([data_name,'_max.mat']);
[ps,ps_body,ps_driving,ps_driven,ps_sensor,t_front_wheel,dt_front_wheel,length,length_cp,P_distance_sum,t_cp_sum,dt_cp_sum,t_sum,dt_sum,ddt_sum]=Simulation_cal(P0,route_index,num_route,V_0,driving_wheel,driven_wheel,Z_sensor,index,P);
[cam_theoretic,cam_virtual,displacement,fluctation,~,~,~]=cam_cal_elliptic(t_front_wheel,c,thickness,ps_driving,d_driving,i_total,d,r);
fp=fopen([save_name format],'w+');
s=size(cam_virtual,1);
if strcmp(format,'.ibl')
    fprintf(fp,'%s\n','open','arclength','begin section','begin curve');
    for i=1:s
        fprintf(fp,'%d\t',i);
        fprintf(fp,'%f\t',cam_virtual(i,:));
        fprintf(fp,'%d\n',0);
    end
elseif strcmp(format,'.txt')
    for i=1:s
        fprintf(fp,'%f\t',cam_virtual(i,:));
        fprintf(fp,'%d\n',0);
    end
elseif strcmp(format,'.scr')
    fprintf(fp,'%s\n','line');
    for i=1:s
        fprintf(fp,'%f,%f\n',cam_virtual(i,:));
    end
else
    error('format赋值不正确');
end
fclose(fp);
%%
figure(2);
clf(2);
subplot(3,5,[1,2,3,6,7,8,11,12,13]);
hold on;
for i=1:size(P,1)
    rectangle('Position',[P(i,1)-12.5,P(i,2)-12.5,25,25],'Curvature',[1,1],'FaceColor','r');
end
rectangle('Position',[ps_sensor(1,1)-9,ps_sensor(1,2)-9,18,18],'Curvature',[1,1],'FaceColor','k');
plot([ps_driving(1,1),ps_driven(1,1)],[ps_driving(1,2),ps_driven(1,2)],'k-',[ps(1,1),ps_body(1,1)],[ps(1,2),ps_body(1,2)],'k-');
plot_route=plot(ps(1,1),ps(1,2),'m');
plot_wheel_driving=plot(ps_driving(1,1),ps_driving(1,2),'b');
plot_wheel_driven=plot(ps_driven(1,1),ps_driven(1,2),'b');
plot_route_sensor=plot(ps_sensor(1,1),ps_sensor(1,2),'k');
plot_car=plot([ps_driving(1,1),ps_driven(1,1)],[ps_driving(1,2),ps_driven(1,2)],'k-',[ps(1,1),ps_body(1,1)],[ps(1,2),ps_body(1,2)],'k-');
plot_sensor=rectangle('Position',[ps_sensor(1,1)-9,ps_sensor(1,2)-9,18,18],'Curvature',[1,1],'FaceColor','k');
axis equal;
axis(Venue_size);
xlabel('x/mm');
ylabel('y/mm');
subplot(3,5,[4,5]);
hold on;
plot(displacement,fluctation,'b');
plot([displacement(1),displacement(1)],[fluctation(1),0],'r-o');
plot_t_site=plot([displacement(1),displacement(1)],[fluctation(1),0],'r-o');
title('车辆转角大小');
xlabel('移动距离/mm');
ylabel('转弯角度(左转为正)/mm');
subplot(3,5,[9,10,14,15]);
hold on;
rectangle('Position',[-d*0.7,-d*0.7,d*1.4,d*1.4],'Curvature',[1,1],'EdgeColor','k');
rectangle('Position',[-d*0.5,-d*0.5,d,d],'Curvature',[1,1],'EdgeColor','k');
rectangle('Position',[cam_theoretic(i,1)-r,cam_theoretic(i,2)-r,2*r,2*r],'Curvature',[1,1],'EdgeColor','r');
plot([0,cam_virtual(1,1),cam_theoretic(1,1)],[0,cam_virtual(1,2),cam_theoretic(1,2)],'r-');
plot(cam_theoretic(:,1),cam_theoretic(:,2),'r-',cam_virtual(:,1),cam_virtual(:,2),'b-');
plot_roller=rectangle('Position',[cam_theoretic(i,1)-r,cam_theoretic(i,2)-r,2*r,2*r],'Curvature',[1,1],'EdgeColor','r');
plot_roller_line=plot([0,cam_virtual(1,1),cam_theoretic(1,1)],[0,cam_virtual(1,2),cam_theoretic(1,2)],'r-');
axis equal;
axis([-0.7*d,0.7*d,-0.7*d,0.7*d]);
title('凸轮曲线');
xlabel('x/mm');
ylabel('y/mm');
pause(3);
for j=1:300
    i=round(size(ps,1)/300*j);
    subplot(3,5,[1,2,3,6,7,8,11,12,13]);
    delete(plot_route);
    plot_route=plot(ps(1:i,1),ps(1:i,2),'r');
    delete(plot_wheel_driving);
    plot_wheel_driving=plot(ps_driving(1:i,1),ps_driving(1:i,2),'b');
    delete(plot_wheel_driven);
    plot_wheel_driven=plot(ps_driven(1:i,1),ps_driven(1:i,2),'b');
    delete(plot_route_sensor);
    plot_route_sensor=plot(ps_sensor(1:i,1),ps_sensor(1:i,2),'k');
    delete(plot_car);
    plot_car=plot([ps_driving(i,1),ps_driven(i,1)],[ps_driving(i,2),ps_driven(i,2)],'k-',[ps(i,1),ps_body(i,1)],[ps(i,2),ps_body(i,2)],'k-');
    delete(plot_sensor);
    plot_sensor=rectangle('Position',[ps_sensor(i,1)-9,ps_sensor(i,2)-9,18,18],'Curvature',[1,1],'FaceColor','k','EdgeColor','k');
    axis equal;
    axis(Venue_size);
    xlabel('x/mm');
    ylabel('y/mm');
    subplot(3,5,[4,5]);
    hold on;
    delete(plot_t_site);
    plot_t_site=plot([displacement(i),displacement(i)],[fluctation(i),0],'r-o');
    title('车辆转角大小');
    xlabel('移动距离/mm');
    ylabel('转弯角度(左转为正)/mm');
    subplot(3,5,[9,10,14,15]);
    hold on;
    delete(plot_roller);
    plot_roller=rectangle('Position',[cam_theoretic(i,1)-r,cam_theoretic(i,2)-r,2*r,2*r],'Curvature',[1,1],'EdgeColor','r');
    delete(plot_roller_line);
    plot_roller_line=plot([0,cam_virtual(i,1),cam_theoretic(i,1)],[0,cam_virtual(i,2),cam_theoretic(i,2)],'r-');
    title('凸轮曲线');
    xlabel('x/mm');
    ylabel('y/mm');
    if abs(j/6-round(j/6))<0.01
        subplot(3,5,[1,2,3,6,7,8,11,12,13]);
        plot([ps_driving(i,1),ps_driven(i,1)],[ps_driving(i,2),ps_driven(i,2)],'k-',[ps(i,1),ps_body(i,1)],[ps(i,2),ps_body(i,2)],'k-');
        rectangle('Position',[ps_sensor(i,1)-9,ps_sensor(i,2)-9,18,18],'Curvature',[1,1],'FaceColor','k','EdgeColor','k');
        subplot(3,5,[4,5]);
        plot([displacement(i),displacement(i)],[fluctation(i),0],'r-o');
        subplot(3,5,[9,10,14,15]);
        rectangle('Position',[cam_theoretic(i,1)-r,cam_theoretic(i,2)-r,2*r,2*r],'Curvature',[1,1],'EdgeColor','r');
        plot([0,cam_virtual(i,1),cam_theoretic(i,1)],[0,cam_virtual(i,2),cam_theoretic(i,2)],'r-');
    end
    drawnow;
end