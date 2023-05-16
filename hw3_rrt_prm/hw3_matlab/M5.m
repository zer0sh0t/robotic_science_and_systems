% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
    q_start = path(1, :);
    smoothed_path = [q_start];

    q = q_start;
    i = 2;

    while i < length(path)
        while i < length(path)

            curr_q = path(i, :);
            if ~check_edge(robot, q, curr_q, link_radius, sphere_centers, sphere_radii)
                best_q = curr_q;
            end
            i = i + 1;
        end

        smoothed_path = [smoothed_path; best_q];
        q = best_q;
    end

    smoothed_path = [smoothed_path; path(end, :)];
end