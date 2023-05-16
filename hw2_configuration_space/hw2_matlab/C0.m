cs = zeros(13, 13);

for a = 1:length(cs)
    for b = 1:length(cs)
        if (a >= 1 && a <= 4) || (a >= 10 && a <= 13)
            if (b >= 1 && b <= 4) || (b >= 10 && b <= 13)
    
                if (b == a + 1) || (a == b) || (b == a - 1)
                    cs(a, b) = 1;
                end
                
            end
        end
    end
end

imshow(cs);
set(gca, 'YDir', 'normal');
xlabel('pa');
ylabel('pb');