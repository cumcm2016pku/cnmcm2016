function routes = GetAllPosibleRoutes(graph)
routes = {};
n = size(graph, 1);

cnt = 0;

for i = 1 : n
    for j = 1 : n
        if graph(i, j) ~= inf && i ~= j
            cnt = cnt + 1;
            routes{cnt} = struct('Route', [i, j], 'Cost', graph(i, j));
        end
    end
end

prev = 1;
cur = cnt;
flag = true;
while flag
    flag = false;
    for i = prev : cur
        r = routes{i}.Route;
        k = r(length(r));
        for j = 1 : n
            if ~isempty(find(r == j, 1)) || graph(k, j) == inf
                continue
            end
            flag = true;
            cnt = cnt + 1;
            routes{cnt} = struct('Route', [r, j], 'Cost', graph(k, j) + routes{i}.Cost);
        end
    end
    prev = cur + 1;
    cur = cnt;
end