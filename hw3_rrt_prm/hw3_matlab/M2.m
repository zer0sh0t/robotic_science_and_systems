% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)
    samples = [];
    while length(samples) < 100
        q = M1(q_min, q_max, 1);
        if ~check_collision(robot, q, link_radius, sphere_centers, sphere_radii)
            samples = [samples; q];
        end
    end

    distances = zeros(num_samples, num_samples);

    for i = 1:num_samples
        for j = 1:num_samples
            if i == j
                distances(i, j) = inf;
            else
                distances(i, j) = norm(samples(j) - samples(i));
            end
        end
    end
    
    adjacency = zeros(num_samples, num_samples);
    
    for i = 1:num_samples
        neighbors = distances(i, :);

        for n = 1:num_neighbors
            [d, j] = min(neighbors);
            neighbors(j) = inf;

            if ~check_edge(robot, samples(i, :), samples(j, :), link_radius, sphere_centers, sphere_radii)
                adjacency(i, j) = d;
                adjacency(j, i) = d;
            end
        end
    end
end