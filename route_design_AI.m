%该代码按照并行运算编码规范编码，能够进行多核多线程并行运算
%%
clc;
clear;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%基础参数 Basic Parameters
data_name='Trajectory_A';%需要启用的数据名称，便于快速基于以往数据继续训练模型，没有数据则自动新建 | Name of dataset for continuing training with historical data; new dataset will be created if not exists
configuration=0;%配置加载0或1，0为最简配置，通过加载的数据配合代码内设置的数据计算，1为满配置，所有数据通过加载得来 | Configuration mode: 0 = minimal config(calculate by internal parameters & loaded data), 1 = full config(all parameters loaded from file)
%路线途经点集，包含始末点坐标 Waypoint set of route, including start and end coordinates
P=[5588,713;4463,375;2925,825;1800,2025;450,2213;375,2738;900,3988;1725,5250;3188,5625;];
%Trajectory_A：P=[5588,713;4463,375;2925,825;1800,2025;450,2213;375,2738;900,3988;1725,5250;3188,5625;];
Venue_size=[0,6000,0,6000];%场地大小[x_min,x_max,y_min,y_max] | Field boundary [x_min,x_max,y_min,y_max]
driving_wheel=[94,-130];%主动轮相对于前轮坐标 | Coordinates of driving wheel relative to front wheel
driven_wheel=[-54,-130];%从动轮相对于前轮坐标 | Coordinates of driven wheel relative to front wheel
Z_sensor=[25,-53];%传感器相对于前轮坐标位置 | Coordinates of sensor relative to front wheel
V_0=[-3,-1];%初始发车向量 | Initial velocity vector at start
d_driving=180;%主动轮直径 | Diameter of driving wheel
i_total=36;%定传动比大小 | Total fixed transmission ratio
c=-33;%前轮中平面与转向推动结构运动平面间距 | Distance between front wheel mid-plane and motion plane of steering push mechanism
thickness=3;%凸轮厚度 | Cam thickness
d=130;%凸轮基准直径：前轮直行时，凸轮轴心到接触曲面轴心距离X2 | Cam reference diameter: distance from cam axis to contact surface axis when front wheel runs straight
r=2;%与凸轮接触的曲面半径 | Radius of curved surface contacting with cam
%高阶参数 Advanced Parameters
traing_times=399;%训练次数 | Number of training iterations
num=20;%训练时插值点数量 | Number of interpolation points during training
control_num=5;
route_index=7;%路线贝塞尔光滑阶数 | Smooth order of Bézier route
gradient_power0=40;%初始化梯度力，后面的实际使用梯度力不得大于该值 | Initial gradient force; actual gradient force shall not exceed this value
power_P0=0.6;%
learning_efficiency=0.3;%学习效率，学习效率越大，学习速度越快，越容易跳出局部最优解，但也容易错过最优解 | learning rate. Larger value speeds up training and helps escape local optima, but may miss global optimum
perturbation_frequency=50;%稳态扰动频率，正整数范围取值，默认值100，即间隔设置的训练次数后，评分机制将小幅调整，打破稳态、扰动评分模型，增强跳出局部最优解的能力 | Steady-state perturbation frequency (positive integer, default=100). Adjust scoring model periodically to break steady state and escape local optima
perturbation_power=2;%稳态扰动力度，实数范围取值，默认值1 | Steady-state perturbation magnitude (real number, default=1)
Scored_index=5+[0;0;3;0;0;0;0;0;0;0;0;0;];%判分指数，可以通过调节大小来调节上面5种期望值的权重，前后顺序为上面5种期望值的顺序,值均为自然数 | Scoring weight vector to tune weights of objective terms; values are natural numbers
E_length=6000;%前轮路线长度的期望值 | Expected value of front wheel path length
E_length_cp=4000;%控制点长度平方和期望值 | Expected value for sum of squared lengths of control points
E_distance_sum=10;%传感器路线偏离点集距离平方和的期望值 | Expected value for sum of squared deviations between sensor path and waypoints
E_t_cp_sum=10;%控制点角度改变大小平方和的期望值 | Expected value for sum of squared angle changes of control points
E_dt_cp_sum=5;%控制点角度改变速度平方和的期望值 | Expected value for sum of squared angular velocity of control points
E_ddt_cp_sum=5;%控制点角度改变加速度平方和的期望值 | Expected value for sum of squared angular acceleration of control points
E_t_sum=300;%前轮角度改变大小平方和的期望值 | Expected value for sum of squared angle changes of front wheel
E_dt_sum=2;%前轮角速度平方和的期望值 | Expected value for sum of squared angular velocity of front wheel
E_ddt_sum=0.05;%前轮角加速度平方和的期望值 | Expected value for sum of squared angular acceleration of front wheel
E_rho_max=3;%理论凸轮曲率最大值的期望值 | Expected maximum theoretical cam curvature
E_pressure_angle_max=0.3;%凸轮压力角的期望值 | Expected maximum cam pressure angle
E_t_virtual_max=3;%凸轮曲线角度变化的期望值 | Expected maximum angular variation of cam profile

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
if exist([data_name,'.mat'],'file')
    if configuration==0
        load([data_name,'.mat']);
    else
        load([data_name,'_max.mat']);
    end
    points=Bezier_superposition(P0,route_index,num);
else
    P0=[P(1,:);zeros((size(P,1)-1)*control_num,2)];
    k=1;
    for i=2:size(P,1)
        V=(P(i,:)-P(i-1,:))/control_num;
        for j=1:(control_num-1)
            k=k+1;
            P0(k,:)=P(i-1,:)+V*j;
        end
        k=k+1;
        P0(k,:)=P(i,:);
    end
    optimal_gradient=zeros(size(P0));
    save([data_name,'.mat'],'P0','optimal_gradient');
    points=Bezier_superposition(P0,route_index,num);
end
gradient_power=gradient_power0;
gradient_power_min=gradient_power;
score_times=zeros(traing_times,1);
perturbation_faction=zeros(size(Scored_index));
perturbation_faction(1)=perturbation_faction(1)+perturbation_power;
scored_index=Scored_index+perturbation_faction;
threads_num=size(P0,1)*4+1;
gdts=zeros([size(P0),threads_num]);
for i=2:size(P0,1)*2+1
    if i>size(P0,1)+1
        gdts(i-size(P0,1)-1,2,i)=1;
    else
        gdts(i-1,1,i)=1;
    end
end
gdts(:,:,size(P0,1)*2+2:size(P0,1)*4+1)=-gdts(:,:,2:size(P0,1)*2+1);
ps_num=size(points,1);
V_0=V_0/norm(V_0);
index=round((num/route_index)^0.5);
perturbation_num=0;
power_P=power_P0;
names={'路线长度：';'控制点长度：';'偏离距离：';'控制点转角和：';'控制点转角速度和：';'控制点转角加速度和：';'前轮转角和：';'前轮转角速度和：';'前轮转角加速度和：';'凸轮曲率最大：';'压力角：';'凸轮曲线角度：';};
E_eigenvalues0=[E_length;E_length_cp;E_distance_sum;E_t_cp_sum;E_dt_cp_sum;E_ddt_cp_sum;E_t_sum;E_dt_sum;E_ddt_sum;E_rho_max;E_pressure_angle_max;E_t_virtual_max;];
E_eigenvalues=E_eigenvalues0;
figure(1);
clf(1);
subplot(3,5,[1,2,3,6,7,8,11,12,13]);
axis(Venue_size);
axis equal;
xlabel('x/mm');
ylabel('y/mm');
subplot(3,5,[4,5]);
title('路线曲率半径');  
xlabel('移动距离/mm');
ylabel('曲率半径/mm');
subplot(3,5,[9,10,14,15]);
axis([-0.7*d,0.7*d,-0.7*d,0.7*d]);
axis equal;
title('凸轮曲线');
xlabel('x/mm');
ylabel('y/mm');
pause(3);
%%
for time=1:traing_times
    if time/perturbation_frequency-round(time/perturbation_frequency)==0
        perturbation_faction=[perturbation_faction(end);perturbation_faction(1:end-1)];
        scored_index=Scored_index+perturbation_faction;
        if gradient_power_min>gradient_power
            P0(1,:)=P0(1,:)+P(1,:)-ps_sensor(1,:);
            P0(end,:)=P0(end,:)+P(end,:)-ps_sensor(end,:);
            P0=Optimize_P0(P0,power_P);
            power_P=power_P*0.9;
            E_eigenvalues=(eigenvalues+E_eigenvalues)/2*0.7;
            gradient_power_min=gradient_power;
            gradient_power=gradient_power0;
        end
    end
    len=norm([optimal_gradient(:,1);optimal_gradient(:,2)]);
    if len>0
        optimal_gradient=optimal_gradient/len;
    end
    gradients=repmat(optimal_gradient,[1 1 threads_num]);
    gradients=gradients+gdts*gradient_power;
    scores=[(1:threads_num)',zeros(threads_num,1)];
    parfor thread=1:threads_num
        control_points=P0+gradients(:,:,thread);
        [~,~,ps_driving,~,~,t_front_wheel,~,length,length_cp,P_distance_sum,t_cp_sum,dt_cp_sum,ddt_cp_sum,t_sum,dt_sum,ddt_sum]=Simulation_cal(control_points,route_index,num,V_0,driving_wheel,driven_wheel,Z_sensor,index,P);
        [~,~,~,~,rho_max,pressure_angle_max,t_virtual_max]=cam_cal_elliptic(t_front_wheel,c,thickness,ps_driving,d_driving,i_total,d,r);
        egv=[length;length_cp;P_distance_sum;t_cp_sum;dt_cp_sum;ddt_cp_sum;t_sum;dt_sum;ddt_sum;rho_max;pressure_angle_max;t_virtual_max];
        [scores(thread,2),~]=score_cal(egv,E_eigenvalues,scored_index);
    end
    scores=sortrows(scores,2);
    if time>1
        n=0;
        o_g=zeros(size(optimal_gradient));
        for i=1:threads_num
            if scores(i,2)>score_times(time-1)
                dn=scores(i,2)-score_times(time-1);
                n=n+dn;
                o_g=o_g+gradients(:,:,scores(i,1))*dn;
            end
        end
        if n>0
            o_g=o_g/n;
        end
        optimal_gradient=(1-learning_efficiency)*optimal_gradient+learning_efficiency*o_g;
        if n==0
            gradient_power=gradient_power/2;
        end
    else
        optimal_gradient=(1-learning_efficiency)*optimal_gradient+learning_efficiency*gradients(:,:,scores(end,1));
    end
    P0=P0+gradients(:,:,scores(end,1));
    if optimal_gradient(1,1)~=real(optimal_gradient(1,1))
        clearvars -except data_name plot_line_CP plot_route plot_sensor plot_cam;
        load([data_name,'_max.mat']);
        P0=Optimize_P0(P0,power_P);
        power_P=power_P0;
        gradient_power_min=gradient_power;
        gradient_power=gradient_power0;
        continue;
    end
    save([data_name,'.mat'],'P0','optimal_gradient');
    save([data_name,'_max.mat'])
    score_times(time)=scores(end,2);
    [ps,~,ps_driving,~,ps_sensor,t_front_wheel,dt,length,length_cp,P_distance_sum,t_cp_sum,dt_cp_sum,ddt_cp_sum,t_sum,dt_sum,ddt_sum]=Simulation_cal(P0,route_index,num,V_0,driving_wheel,driven_wheel,Z_sensor,index,P);
    [cam_theoretic,cam_virtual,~,~,rho_max,pressure_angle_max,t_virtual_max]=cam_cal_elliptic(t_front_wheel,c,thickness,ps_driving,d_driving,i_total,d,r);
    eigenvalues=[length;length_cp;P_distance_sum;t_cp_sum;dt_cp_sum;ddt_cp_sum;t_sum;dt_sum;ddt_sum;rho_max;pressure_angle_max;t_virtual_max];
    [~,score]=score_cal(eigenvalues,E_eigenvalues,scored_index);
    disp('迭代次数：'+string(traing_times)+'  '+string(time));
    disp('梯度力：'+string(gradient_power0)+'  '+string(gradient_power));
    disp('综合分数：'+string(score_times(1))+'  '+string(score_times(time)));
    for i=1:size(E_eigenvalues,1)
        disp(names(i)+string(E_eigenvalues(i))+'  '+string(eigenvalues(i))+'  '+string(score(i)));
    end
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    subplot(3,5,[1,2,3,6,7,8,11,12,13]);
    hold off;
    plot(P0(:,1),P0(:,2),'g','LineWidth',1);
    hold on;
    for i=1:size(P,1)
        rectangle('Position',[P(i,1)-12.5,P(i,2)-12.5,25,25],'Curvature',[1,1],'FaceColor','k');
    end
    plot(ps(:,1),ps(:,2),'r','LineWidth',1);
    plot(ps_sensor(:,1),ps_sensor(:,2),'k','LineWidth',1);
    axis equal;
    axis(Venue_size);
    xlabel('x/mm');
    ylabel('y/mm');
    subplot(3,5,[4,5]);
    plot(1:ps_num,t_front_wheel,'b');
    title('前轮转角');  
    xlabel('移动距离/mm');
    ylabel('转角大小/mm');
    subplot(3,5,[9,10,14,15]);
    hold off;
    plot_cam=plot(cam_theoretic(:,1),cam_theoretic(:,2),'r-',cam_virtual(:,1),cam_virtual(:,2),'b-');
    hold on;
    rectangle('Position',[-d*0.7,-d*0.7,d*1.4,d*1.4],'Curvature',[1,1],'EdgeColor','k');
    rectangle('Position',[-d*0.5,-d*0.5,d,d],'Curvature',[1,1],'EdgeColor','k');
    axis equal;
    title('凸轮曲线');
    xlabel('x/mm');
    ylabel('y/mm');
    drawnow;
end