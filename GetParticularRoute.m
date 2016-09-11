function routes = GetParticularRoute(allRoutes, from, to)
routes = {};
cnt = 0;
for i = 1 : length(allRoutes)
    r = allRoutes{i}.Route;
    if r(1) == from && r(length(r)) == to
        cnt = cnt + 1;
        routes(cnt) = allRoutes(i);
    end
end