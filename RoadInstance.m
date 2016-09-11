function road = RoadInstance(type, width, length, nCarriageway, v, vmin, waitTime, nCross, startPoint, endPoint)
road = struct('type', type, ...  % 道路类型， 1为小区内道路，2为小区周边道路
            'width', width, ...  % 平均每个车道的宽度，单位为米
            'length', length, ... % 道路总长度，单位为米
            'nCarriageway', nCarriageway, ...  % 该方向的车道数目
            'v', v, ...   % 正常行驶速度，单位为米每秒
            'vmin', vmin, ...   % 通过无信号灯的交叉口时，减速到的最低速度
            'waitTime', waitTime, ...  % 在交叉路口的等待时间，单位为秒
            'nCross', nCross, ... % 交叉口个数
            'Start', startPoint, ....
            'End', endPoint, ...              %% 接下来是一些默认参数
            'alphaNumbers', [1, 0.85,  0.715, 0.575, 0.46], ... % 折减系数，最多支持同一方向有5条道路
            'capacity', 0);
