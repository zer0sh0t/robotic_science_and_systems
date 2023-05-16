% Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = C3(cspace, q_grid, q_goal)
    distances = cspace;
    res_len = length(q_grid);
    
    least_dist = inf;
    for i = 1:res_len
        for j = 1:res_len
            if distances(i, j) == 0

                dist = ((q_grid(i) - q_goal(1)) ^ 2 + (q_grid(j) - q_goal(2)) ^ 2) ^ 0.5;
                if dist < least_dist
                    least_dist = dist;
                    goal_i = i;
                    goal_j = j;
                end
                
            end
        end
    end
    
    distances(goal_i, goal_j) = 2;

    Lx = goal_i;
    Ly = goal_j;

    while ~isempty(Lx)
        curr_i = Lx(1);
        curr_j = Ly(1);
        Lx(1) = [];
        Ly(1) = [];

        for i = curr_i-1: curr_i+1
            for j = curr_j-1: curr_j+1
                
                if i >= 1 && i <= res_len && j >= 1 && j <= res_len
                    if distances(i, j) == 0
                        distances(i, j) = distances(curr_i, curr_j) + 1;
                        Lx(end + 1) = i;
                        Ly(end + 1) = j;
                    end
                end
            
            end
        end
    end
end