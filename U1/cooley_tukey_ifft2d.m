function X_inv = cooley_tukey_ifft2d(X)
    % Cooley-Tukey inverse FFT algorithm for input matrix X (n x m)
    % X must have dimensions that are powers of 2
    [n, m] = size(X);

    % Check if both dimensions are powers of 2
    if mod(log2(n), 1) ~= 0 || mod(log2(m), 1) ~= 0
        error('Both dimensions of input matrix must be powers of 2.');
    end

    % Perform 1D inverse FFT on each row
    X_inv = zeros(n, m);
    for i = 1:n
        X_inv(i, :) = cooley_tukey_ifft(X(i, :));
    end

    % Perform 1D inverse FFT on each column of the result
    for j = 1:m
        X_inv(:, j) = cooley_tukey_ifft(X_inv(:, j).').';
    end

    % Normalize by dividing by the total number of elements
    X_inv = X_inv / (n * m);
end

function x = cooley_tukey_ifft(X)
    % Cooley-Tukey inverse FFT algorithm for input vector X
    % X must have a length that is a power of 2
    N = length(X);

    % Base case
    if N <= 1
        x = X;
        return;
    end

    % Split the input into even and odd indexed elements
    X_even = X(1:2:end);
    X_odd = X(2:2:end);

    % Recursive calls for even and odd parts
    x_even = cooley_tukey_ifft(X_even);
    x_odd = cooley_tukey_ifft(X_odd);

    % Combine the results with positive exponent for inverse
    x = zeros(1, N);
    for k = 1:(N/2)
        t = exp(2i * pi * (k - 1) / N) * x_odd(k);
        x(k) = x_even(k) + t;
        x(k + N/2) = x_even(k) - t;
    end
end
