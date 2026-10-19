function s=Squares_standard_sum(data)
len=max(size(data));
s=((sum(data.^2)/len)^0.5)*len;
end