function [P_distance_sum,P_distance]=distance_calculate(P,points)
P_distance=zeros(size(P,1),1);
P_distance(1)=norm(P(1,:)-points(1,:));
P_distance(end)=norm(P(end,:)-points(end,:));
n=2;
distance=norm(P(2,:)-points(2,:));
for i=3:size(points,1)-1
    d=norm(P(n,:)-points(i,:));
    if d<distance
        distance=d;
    else
        d1=Point_line_distance(P(n,:),points(i-2,:),points(i-1,:));
        d2=Point_line_distance(P(n,:),points(i,:),points(i-1,:));
        P_distance(n)=min(d1,d2);
        n=n+1;
        distance=norm(P(n,:)-points(i,:));
        if n==size(P,1)
            break;
        end
    end
end
P_distance_sum=Squares_standard_sum(P_distance);
end