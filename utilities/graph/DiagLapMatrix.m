%% cycleGL: $\mathbf{Z}^{+} \to \mathbf{Z}^{N \times N}$
% Returns a NxN laplacian matrix with specified diagnoal values of S  
%% Example Usage 
%   L = cycleGL(4, -1)
%% Implementation
function [ L ] = DiagLapMatrix(n, S)
    L = zeros(n,n);
    for i = 1:n
        L(i,i) = S;
    end
end
