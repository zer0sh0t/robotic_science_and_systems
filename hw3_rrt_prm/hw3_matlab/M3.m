% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
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

function [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    start_idx = knnsearch(samples, q_start);
    goal_idx = knnsearch(samples, q_goal);
 
    G = digraph(adjacency);
    %{
    figure
    plot(G)
    kjasndk
    figure
    %}
    [path, d] = shortestpath(G, start_idx, goal_idx);
    
    if isinf(d)
        path_found = false;
    else
        path_found = true;
    end
    
    path = samples(path, :);
    path = [q_start; path];
    path = [path; q_goal];
end