% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_path -> Mx2 matrix containing a collision-free path from
%                  q_start to q_goal. Each row in q_path is a robot
%                  configuration. The first row should be q_start,
%                  the final row should be q_goal.
% Output: num_collisions -> Number of swept-volume collisions encountered
%                           between consecutive configurations in q_path

function num_collisions = C6(robot, obstacles, q_path)
    num_collisions = 0;

    for i = 1:length(q_path)-1
        q = q_path(i, :);
        q_ = q_path(i+1, :);
        [poly1, poly2, ~, ~] = q2poly(robot, q);
        [poly1_, poly2_, ~, ~] = q2poly(robot, q_);
        
        poly1_hull = convhull(union(poly1, poly1_));
        poly2_hull = convhull(union(poly2, poly2_));
        
        for i = 1:length(obstacles)
            if ~isempty(intersect(poly1_hull, obstacles(i)).Vertices)
                num_collisions = increment_counter(robot, q, q_, num_collisions);
                continue
            end

            if ~isempty(intersect(poly2_hull, obstacles(i)).Vertices)
                num_collisions = increment_counter(robot, q, q_, num_collisions);
                continue
            end
        end
    end
end

function num_collisions = increment_counter(robot, q, q_, num_collisions)
    num_collisions = num_collisions + 1;
    C1(robot, q)
    C1(robot, q_)
end