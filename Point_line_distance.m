function distance=Point_line_distance(P0,P1,P2)
mat=[P0,1;P1,1;P2,1];
if norm(P1-P2)==0
    distance=norm(P0-P1);
else
    distance=abs(det(mat)/norm(P1-P2));
end
end