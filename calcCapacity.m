function road = calcCapacity(road)
if road.Type == 1   % ��Χ��·
    road.Capacity = road.Vmin + road.Vmax;
else
    road.Capacity = road.Vmax - road.Vmin;
end

function c0 = baseRoadCapacity(road)
  

end
