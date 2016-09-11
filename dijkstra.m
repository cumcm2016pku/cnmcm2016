function [path, cost] = dijkstra(road_net, src, dst)

dist = zeros(size(road_net, 1), 1);
dist(:) = inf;

prev = zeros(size(road_net, 1), 1);

dist(src) = 0;
n = size(road_net, 1);
flag = true(n, 1);
for j = 1 : n
    minDist = inf;
    choose = 1;
    for i = 1 : n
        if flag(i) && minDist > dist(i)
            minDist = dist(i);
            choose = i;
        end
    end
    flag(choose) = false;
    for i = 1 : n
        if flag(i) && dist(i) > dist(choose) + road_net(choose, i)
            dist(i) = dist(choose) + road_net(choose, i);
            prev(i) = choose;
        end
    end
end

cost = dist(dst);
cur = dst;
cnt = 1;
while cur ~= src
    path(cnt) = cur;
    cur = prev(cur);
    cnt = cnt + 1;
end
path(cnt) = src;

path = path(cnt:-1:1);