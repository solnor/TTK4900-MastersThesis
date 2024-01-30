function AT = calculate_sm(q, a, b)
    theta = q(3);

    ln = inverse_kinematics(a, b, q, 4);
    l = calculate_l_vec(q, a, b);
    u = [l(:,1)/ln(1) l(:,2)/ln(2) l(:,3)/ln(3) l(:,4)/ln(4)];

    R = R_z(theta);
    bi = [R*b(:,1) R*b(:,2) R*b(:,3) R*b(:,4)];
    
    bcrossu = zeros(1,4);
    for i = 1:4
        biskew = skew([bi(:,i); 0]);
        crossProd = biskew*[u(:,i);0];
        bcrossu(i) = crossProd(3);
    end
    
    AT = [u;bcrossu];
end

function uskew = skew(u)
    u1 = u(1);
    u2 = u(2);
    u3 = u(3);
    uskew = [ 0  -u3  u2;
              u3  0  -u1;
             -u2  u1  0];
end