% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = C2(robot, obstacles, q_grid)
    res_len = length(transpose(q_grid));
    cspace = zeros(res_len, res_len);

    for i = 1:res_len
        for j = 1:res_len
            q0 = q_grid(i);
            q1 = q_grid(j);
            q = [q0; q1];
            [poly1, poly2, pivot1, pivot2] = q2poly(robot, q);
            
            for k = 1:length(obstacles)
                if intersect(poly1, obstacles(k)).NumRegions > 0
                    cspace(i , j) = 1;
                    continue
                end
                if intersect(poly2, obstacles(k)).NumRegions > 0
                    cspace(i , j) = 1;
                    continue
                end
            end
        end
    end
end