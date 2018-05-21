function [m] = media(vector)
m = 0;
k = 0;
    for i = 1:length(vector)
        if ~isnan(vector(i,1))
            m = m + vector(i,1);
            k = k+1;
        end
    end
    
    m = m/k;
end

