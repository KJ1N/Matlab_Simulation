function [first_point,end_point]=cal_endpoints(points,index)
first_point=zeros(1,size(points,2));
end_point=first_point;
YangHui=cal_YangHui(index);
k=1;
for i=1:index
    first_point=first_point+points(i,:)*k*YangHui(i+1);
    end_point=end_point+points(end-i+1,:)*k*YangHui(i+1);
    k=-k;
end
end