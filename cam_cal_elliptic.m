function [cam_theoretic,cam_virtual,displacement,fluctation,rho_max,pressure_angle_max,t_virtual_max]=cam_cal_elliptic(t_front_wheel,c,thickness,ps_driving,d_driving,i_total,d,r)
fluctation=-tan(t_front_wheel)*c;
Vs=ps_driving(2:end,:)-ps_driving(1:end-1,:);
Vs_norm=(Vs(:,1).^2+Vs(:,2).^2).^0.5;
displacement=zeros(size(fluctation,1),1);
for i=1:size(displacement)-1
    displacement(i+1)=displacement(i)+Vs_norm(i);
end
i_valid=displacement(end)/pi/d_driving;
if i_valid<i_total
    dpm=[displacement(end),fluctation(end);0,0;0,0;i_total*pi*d_driving,fluctation(1)];
    len=(dpm(end,1)-dpm(1,1))/3;
    V1=[displacement(end)-displacement(end-1),fluctation(end)-fluctation(end-1)];
    V2=[displacement(1)-displacement(2),fluctation(1)-fluctation(2)];
    V1=V1/V1(2);
    V2=V2/V2(2);
    if V1(1)>0
        V1=V1*(max(fluctation)*1.2-dpm(1,2));
    else
        V1=V1*(min(fluctation)*1.2-dpm(1,2));
    end
    if V2(1)<0
        V2=V2*(max(fluctation)*1.2-dpm(end,2));
    else
        V2=V2*(min(fluctation)*1.2-dpm(end,2));
    end
    if V1(1)>len
        V1=V1/V1(1)*len;
    end
    if V2(1)<-len
        V2=V2/V2(1)*-len;
    end
    dpm(2:3,:)=[dpm(1,:)+V1;dpm(end,:)+V2];
    dpm=Bezier_curve(dpm,3,round(size(displacement,1)/displacement(end)*(dpm(end,1)-dpm(1,1))));
    fluctation=[fluctation;dpm(:,2)];
    displacement=[displacement;dpm(:,1)];
end
cam_angle=displacement/displacement(end)*2*pi;
cam_theoretic=[cos(cam_angle),sin(cam_angle)].*([fluctation,fluctation]+d/2);
cam_virtual=cam_theoretic;
cam_temp=[cam_virtual(end,:);cam_virtual;cam_virtual(1,:)];
for i=1:size(cam_virtual,1)
    v0=cam_temp(i+2,:)-cam_temp(i,:);
    v1=cam_temp(i+1,:);
    if c<0
        g=-thickness;
    else
        g=thickness;
    end
    if norm(v1)>d/2
        m=c-g/2;
    else
        m=c+g/2;
    end
    t=atan((d/2-norm(v1))/m);
    rb=r/cos(t);
    t1=acos(sum(v0.*v1)/(norm(v0)*norm(v1)));
    tant1=tan(t1-pi/2);
    x=tant1*r^2/(rb^2+(r*tant1)^2)^0.5;
    y=-rb*(1-(x/r)^2)^0.5;
    v2=[x y];
    t2=acos(v1(1)/norm(v1));
    if v1(2)<0
        t2=-t2;
    end
    t3=acos(-v2(1)/norm(v2));
    t3=pi/2+t2+t3;
    cam_virtual(i,:)=v1+norm(v2)*[cos(t3),sin(t3)];
end
rhos=zeros(size(cam_theoretic,1)-1,1);
for i=1:size(rhos,1)-1
    rhos(i)=Curvature_radius(cam_theoretic(i,:),cam_theoretic(i+1,:),cam_theoretic(i+2,:));
end
rhos(end)=Curvature_radius(cam_theoretic(end,:),cam_theoretic(1,:),cam_theoretic(2,:));
rho_max=1/min(rhos);
Vs_F=(cam_theoretic-cam_virtual)/r;
Vs_norm=(cam_virtual(:,1).^2+cam_virtual(:,2).^2).^0.5;
Vs=cam_virtual./[Vs_norm,Vs_norm];
pressure_angle=acos(Vs_F(:,1).*Vs(:,1)+Vs_F(:,2).*Vs(:,2));
pressure_angle_max=max(pressure_angle);
Vs=[cam_virtual(2:end,:);cam_virtual(1,:)]-[cam_virtual(end,:);cam_virtual(1:end-1,:)];
Vs_norm=(Vs(:,1).^2+Vs(:,2).^2).^0.5;
Vs=Vs./[Vs_norm,Vs_norm];
dot=Vs(2:end,1).*Vs(1:end-1,1)+Vs(2:end,2).*Vs(1:end-1,2);
symbol=dot>1;
dot(symbol)=1;
symbol=dot<-1;
dot(symbol)=-1;
t_virtual=acos(dot);
t_virtual_max=max(t_virtual);
end