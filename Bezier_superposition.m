function points=Bezier_superposition(control_points,Bezier_index,num)
weight=(-4:4)';
weight=[weight,[0;0;0.2;0.8;1;0.8;0.2;0;0]];
weight=Bezier_curve(weight,8,num*Bezier_index);
curve_num=size(control_points,1)-Bezier_index;
weight_mat=zeros((size(control_points,1)-1)*num,curve_num);
curves=zeros((size(control_points,1)-1)*num,2,curve_num);
for i=1:curve_num
    row=(i-1)*num+1;
    weight_mat(row:row+size(weight,1)-1,i)=weight(:,2);
    curves(row:row+size(weight,1)-1,:,i)=Bezier_curve(control_points(i:i+Bezier_index,:),Bezier_index,num*Bezier_index);
end
weight_mat(1,1)=1;
weight_mat(end,end)=1;
for i=1:size(weight_mat,1)
    weight_mat(i,:)=weight_mat(i,:)/sum(weight_mat(i,:));
end
points=[zeros((size(control_points,1)-1)*num,2);control_points(end,:)];
for i=1:size(control_points,1)-1
    interval=[i-Bezier_index+1,i];
    if interval(1)<1
        interval(1)=1;
    end
    if interval(2)>curve_num
        interval(2)=curve_num;
    end
    for j=interval(1):interval(2)
        width=[(i-1)*num+1,i*num];
        w_mat=[weight_mat(width(1):width(2),j),weight_mat(width(1):width(2),j)];
        points(width(1):width(2),:)=points(width(1):width(2),:)+w_mat.*curves(width(1):width(2),:,j);
    end
    if i==36
        continue;
    end
end
end