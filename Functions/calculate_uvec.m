function u = calculate_uvec(q,a,b)
    r = q(1:2);
    theta = q(3);
    
    R = R_z(theta);

    u = zeros(2,4);
    for i = 1:4
        lvec = a(:,i) - r - R*b(:,i);
        u(:,i) = lvec/norm(lvec);
    end
end