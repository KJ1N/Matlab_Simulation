function P0=Optimize_P0(P0,power)
    P0_temp=P0;
    for i=2:size(P0,1)-1
        P0(i,:)=(P0_temp(i+1,:)+P0_temp(i-1,:))/2*power+P0_temp(i,:)*(1-power);
    end
end