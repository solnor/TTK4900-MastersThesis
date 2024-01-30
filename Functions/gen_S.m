function S = gen_S(s, nx, nu, N)
    % https://www.mathworks.com/matlabcentral/answers/39838-repeat-a-matrix-as-digonal-element-in-a-new-matrix
    sc = repmat({s}, 1, N);
    sc{1, 1} = 1/2*s;
    sc{end, end} = 1/2*s;
    
    S = blkdiag(sc{:});

    for i=1:(N-1)
      S([(i*nu+1):((i+1)*nu)],[((i-1)*nu+1):(i*nu)])=-s;
    end
    
    S = [zeros(N*nx,N*nx) zeros(N*nx,size(S,2))
         zeros(N*nu,N*nx) S];
end