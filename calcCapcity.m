function road = calcCapcity(road)
if road.Type == 1   % ��Χ��·
    road.Capcity = road.Vmin + road.Vmax;
else
    road.Capcity = road.Vmax - road.Vmin;
end