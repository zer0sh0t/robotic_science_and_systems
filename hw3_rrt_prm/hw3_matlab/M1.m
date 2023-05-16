% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

function qs = M1(q_min, q_max, num_samples)
    for k = 1:length(q_min)
        qs(:,k) = q_min(k) + (q_max(k) - q_min(k)) * rand(num_samples,1);
    end
end