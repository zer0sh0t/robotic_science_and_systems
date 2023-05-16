% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to convert to polygons
% Output: poly1 -> A polyshape object denoting the 2-D polygon describing
%                  link 1
%         poly2 -> A polyshape object denoting the 2-D polygon describing
%                  link 2
%         pivot1 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 1 (frame 1 origin), with respect to base frame
%         pivot2 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 2 (frame 2 origin), with respect to base frame

function [poly1, poly2, pivot1, pivot2] = q2poly(robot, q)

    pivot1 = robot.pivot1;
    poly1 = polyshape(transpose(robot.link1));
    q1 = rad2deg(q(1));
    poly1 = rotate(poly1, q1);
    poly1.Vertices = poly1.Vertices + transpose(pivot1);

    rp2 = robot.pivot2;
    pivot2(1) = pivot1(1) + rp2(1) * cos(q(1));
    pivot2(2) = pivot1(2) + rp2(1) * sin(q(1));
    
    % origin2_at0 = origin1_at0 + robot.pivot2;
    q_ = rad2deg(q(1) + q(2));
    poly2 = polyshape(transpose(robot.link2));
    poly2 = rotate(poly2, q_);
    
    poly2.Vertices = poly2.Vertices + pivot2;

end