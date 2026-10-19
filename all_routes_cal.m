function [ps_body,ps_driving,ps_driven,ps_sensor,t_front_wheel]=all_routes_cal(points,V_0,driving_wheel,driven_wheel,Z_sensor,index)
V_y=V_0/norm(V_0);
V_x=[V_y(2),-V_y(1)];
ps_body=points;
ps_body(1,:)=points(1,:)+driving_wheel(2)*V_y;
ps_driving=points;
ps_driving(1,:)=points(1,:)+driving_wheel(1)*V_x+driving_wheel(2)*V_y;
ps_driven=points;
ps_driven(1,:)=points(1,:)+driven_wheel(1)*V_x+driven_wheel(2)*V_y;
ps_sensor=points;
ps_sensor(1,:)=points(1,:)+Z_sensor(1)*V_x+Z_sensor(2)*V_y;
V_front_wheel=[0,0;points(3:end,:)-points(1:end-2,:);0,0];
[V_front_wheel(1,:),V_front_wheel(end,:)]=cal_endpoints(V_front_wheel(2:end-1,:),index);
t_front_wheel=zeros(size(points,1),1);
for i=2:size(points)
    V_y=ps_body(i,:)-ps_body(i-1,:);
    V_y=V_y/norm(V_y);
    V_x=[V_y(2),-V_y(1)];
    ps_body(i,:)=points(i,:)+driving_wheel(2)*V_y;
    ps_driving(i,:)=points(i,:)+driving_wheel(1)*V_x+driving_wheel(2)*V_y;
    ps_driven(i,:)=points(i,:)+driven_wheel(1)*V_x+driven_wheel(2)*V_y;
    ps_sensor(i,:)=points(i,:)+Z_sensor(1)*V_x+Z_sensor(2)*V_y;
    t_front_wheel(i)=acos(dot(V_front_wheel(i,:),V_y)/norm(V_front_wheel(i,:)));
    V_sign=cross([V_y,0],[V_front_wheel(i,:),0]);
    if V_sign(3)<0
        t_front_wheel(i)=-t_front_wheel(i);
    end
end
[t_front_wheel(1,:),~]=cal_endpoints(t_front_wheel(2:end-1,:),index);
end