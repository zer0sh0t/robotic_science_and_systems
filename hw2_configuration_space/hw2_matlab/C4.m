% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = C4(distances, q_grid, q_start)
    res_len = length(q_grid);
    
    least_dist = inf;
    for i = 1:res_len
        for j = 1:res_len
            if distances(i, j) ~= 1

                dist = ((q_grid(i) - q_start(1)) ^ 2 + (q_grid(j) - q_start(2)) ^ 2) ^ 0.5;
                if dist < least_dist
                    least_dist = dist;
                    start_i = i;
                    start_j = j;
                end

            end
        end
    end
    
    curr_i = start_i;
    curr_j = start_j;

    % min_dist = inf;
    path = [start_i ,start_j];

    while distances(curr_i, curr_j) > 2
        min_dist = inf;
        for i = curr_i-1: curr_i+1
            for j = curr_j-1: curr_j+1
                if i >= 1 && i <= res_len && j >= 1 && j <= res_len
                    if distances(i, j) < min_dist && distances(i, j) >= 2 
                        min_dist = distances(i,j);
                        
                        min_i = i;
                        min_j = j;
                        
                    end
                end
            end
        end

        path = [path; min_i, min_j];
        curr_i = min_i;
        curr_j = min_j;
    
    end
    
end