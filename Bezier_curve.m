function points=Bezier_curve(P0,route_index,num)
    n=((size(P0,1)-1)/route_index);
    points=[P0(1,:);zeros(n*num,2)];
    YangHui=cal_YangHui(route_index);
    t0=(1/num:1/num:1)';
    t0=[t0,t0];
    t1=1-t0;
    for i=2:(size(P0,1)-1)/route_index+1
        Bezier_P=P0((i-2)*route_index+1:(i-1)*route_index+1,:);
        for j=1:route_index+1
            points((2+(i-2)*num):(1+(i-1)*num),:)=points((2+(i-2)*num):(1+(i-1)*num),:)+YangHui(j)*Bezier_P(j,:).*t0.^(j-1).*t1.^(route_index+1-j);
        end
    end
end