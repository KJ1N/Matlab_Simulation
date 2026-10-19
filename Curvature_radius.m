function rho=Curvature_radius(P1,P2,P3)
    k=0;
    a=[P1,1,-P1(1)^2-P1(2)^2;P2,1,-P2(1)^2-P2(2)^2;P3,1,-P3(1)^2-P3(2)^2];
    for i=1:3
        if a(i,i)~=0
            a(i,:)=a(i,:)/a(i,i);
        else
            k=1;
            for j=i+1:3
                if a(j,i)~=0
                    b=a(j,:);
                    a(j,:)=a(i,:);
                    a(i,:)=b;
                    k=0;
                    break;
                end
            end
            if k==1
                rho=Inf;
                break;
            end
        end
        for j=i+1:3
            a(j,:)=a(j,:)-a(i,:)*a(j,i);
        end
    end
    if k==0
        for i=3:-1:2
            for j=i-1:-1:1
                a(j,:)=a(j,:)-a(i,:)*a(j,i);
            end
        end
        rho_quadratic=(a(1,4)^2+a(2,4)^2)/4-a(3,4);
        if rho_quadratic>0
            rho=rho_quadratic^0.5;
        else
            rho=Inf;
        end
    end
end