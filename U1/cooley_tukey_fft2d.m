function X = cooley_tukey_fft2d(x)
    % Cooley-Tukey FFT algorithm for input matrix x (n x m)
    % x must have dimensions that are powers of 2
    [n, m] = size(x);

    % Check if both dimensions are powers of 2
    if mod(log2(n), 1) ~= 0 || mod(log2(m), 1) ~= 0
        error('Both dimensions of input matrix must be powers of 2.');
    end

    % Perform 1D FFT on each row
    X = zeros(n, m);
    for i = 1:n
        X(i, :) = cooley_tukey_fft(x(i, :));
    end

    % Perform 1D FFT on each column of the result
    for j = 1:m
        X(:, j) = cooley_tukey_fft(X(:, j).').';
    end
end

function X = cooley_tukey_fft(x)
    % Cooley-Tukey FFT algorithm for input vector x
    % x must have a length that is a power of 2
    N = length(x);

    % Base case
    if N <= 1
        X = x;
        return;
    end

    % Split the input into even and odd indexed elements
    x_even = x(1:2:end);
    x_odd = x(2:2:end);

    % Recursive calls for even and odd parts
    X_even = cooley_tukey_fft(x_even);
    X_odd = cooley_tukey_fft(x_odd);

    % Combine the results
    X = zeros(1, N);
    for k = 1:(N/2)
        t = exp(-2i * pi * (k - 1) / N) * X_odd(k);
        X(k) = X_even(k) + t;
        X(k + N/2) = X_even(k) - t;
    end
end
