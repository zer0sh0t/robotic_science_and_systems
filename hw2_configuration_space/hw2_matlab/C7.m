% Input: cspace -> NxN matrix: cspace(i,j)
%                  == 1 if [q_grid(i); q_grid(j)] is in collision,
%                  == 0 otherwise
% Output: padded_cspace -> NxN matrix: padded_cspace(i,j)
%                          == 1 if cspace(i,j) == 1, or some neighbor of
%                                  cell (i,j) has value 1 in cspace
%                                  (including diagonal neighbors)
%                          == 0 otherwise

function padded_cspace = C7(cspace)
    padded_cspace = cspace;
    for i = 1:length(cspace)
        for j = 1:length(cspace)
            
            if cspace(i, j) == 1
                for k = i-1:i+1
                    for l = j-1:j+1
                        if k ~= i && l ~= j
                            if k >= 1 && k <= length(cspace) && l >= 1 && l <= length(cspace)
                                  padded_cspace(k, l) = 1;
                            end
                        end
                    end
                end
            end

        end
    end
end