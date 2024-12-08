function [PointClusterAffiliation, T] = kmeans(points, n_clusters)

points_size = size(points);
n_points = points_size(1); dimension = points_size(2);
if dimension<2
    error('Dimension must be 2 or higher!\n')
end
points_min = min(points);
points_max = max(points);
dc = eye(1, dimension); % dcoordinates
for i = 1:dimension
    dc(i) = (points_max(i)-points_min(i))/(n_clusters+1);
end

T = eye(n_clusters, dimension);

for i = 1:n_clusters
    for j = 1:dimension
        T(i, j) = min(j)+dc(j)*(i);
    end
end

if dimension == 2
    figure(1)
    scatter(T(:,1), T(:,2), 18, 'color', '.g')
    hold on
else
    if dimension == 3
        figure(1)
        scatter3(T(:,1), T(:,2), T(:, 3), 18, 'color', '.g')
        hold on
    else
        disp('Unable to create a plot - dimension is too high!\n')
    end
end

d = eye(n_points, n_clusters);
PointClusterAffiliation = eye(n_points, 1);
posun_lim = 0.1;
posun = ones(1, dimension);
iter = 0;

while any(posun>posun_lim)
    iter = iter+1;
    for i = 1:n_points
        for j = 1:n_clusters
            d(i, j) = pdist([points(i, :); T(j, :)]);
        end
    end
    
    for i = 1:n_points
        min_d = 10e6;
        min_idx = 0;
        for j = 1:n_clusters
            new_d = d(i, j);
            if new_d<min_d
                min_d = new_d;
                min_idx = j;
            end
        end
        PointClusterAffiliation(i) = min_idx;
    end
    
    T_prev = T;
    
    for i = 1:n_clusters
        CurrentPointClusterAffiliation = find(PointClusterAffiliation == i);
        P_i = points(CurrentPointClusterAffiliation, :);
        T(i, :) = mean(P_i);
        dT = T(i,:)-T_prev(i, :);
        posun(i) = rssq(dT);
    end
end

if dimension == 2
    scatter(points(:, 1), points(:, 2), 18)
    scatter(T(:,1), T(:,2), 18, 'color', '.r')
    legend('Původní těžiště', 'Množina bodů', 'Nové těžiště', 'Location', 'best')
    hold off
else
    if dimension == 3
        scatter3(points(:, 1), points(:, 2), points(:, 3), 18)
        scatter3(T(:,1), T(:,2), T(:,3), 18, 'color', '.r')
        legend('Původní těžiště', 'Množina bodů', 'Nové těžiště', 'Location', 'best')
        hold off
    end
end
end