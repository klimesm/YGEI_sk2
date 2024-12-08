clc; close all; clear variables; format longg; 

cluster1 = rand([20, 3]).*5 + [4, 2, 2];
cluster2 = rand([20, 3]).*5 + [8, 10, 0];
cluster3 = rand([20, 3]).*5 + [12, 10, 14];

points = [cluster1; cluster2; cluster3];
[cluster_ids, T] = kmeans(points, 3);


