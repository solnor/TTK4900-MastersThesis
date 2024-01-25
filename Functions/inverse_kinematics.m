function ln = inverse_kinematics(a, b, pose, m)
    x     = pose(1);
    y     = pose(2);
    theta = pose(3);

    ln = zeros(m,1);
    for i=1:m
        ln(i) = norm(a(:,i)-[x;y]-R_z(theta)*b(:,i),2);
    end
end