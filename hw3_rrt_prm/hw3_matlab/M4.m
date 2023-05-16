% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    path_found = false;
    samp_freq = 0.7;
    step_size = 1.5;
    max_nodes = 10000;
    V = [q_start];
    E = [q_start];
    node_i = 1;

    while node_i < max_nodes
        if rand(1) < samp_freq
            q_target = q_goal;
        else
            q_target = q_min + (q_max - q_min) * rand(1, 1);
        end

        q_near_idx = knnsearch(V, q_target);
        q_near = V(q_near_idx, :);
        q_new = q_near + (step_size / norm(q_target - q_near))  * (q_target - q_near);

        if ~check_collision(robot, q_new, link_radius, sphere_centers, sphere_radii)
            if ~check_edge(robot, q_near, q_new, link_radius, sphere_centers, sphere_radii)
                V = [V; q_new];
                E = [E; q_new];
            end
        end
        
        path = E;
        if norm(q_new - q_goal) < 0.8
            path_found = true;
            break
        end

        node_i = node_i + 1;
    end
    path = [path; q_goal];
end