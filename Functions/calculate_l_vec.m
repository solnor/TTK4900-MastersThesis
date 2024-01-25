function l_vec = calculate_l_vec(q, a, b)
    r = q(1:2);
    theta = q(3);
    
    R = R_z(theta);

    l_vec = zeros(2,4);
    for i = 1:4
        l_vec(:,i) = a(:,i) - r - R*b(:,i);
    end
end