function YangHui=cal_YangHui(num)
    YangHui=ones(1,num+1);
    for i=2:num
        YangHui(2:i)=YangHui(2:i)+YangHui(1:i-1);
    end
end