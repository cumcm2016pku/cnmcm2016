function capacity = calcCapacity(road)
% CALCCAPACITY 计算通行能力
c0 = baseRoadCapacity(road);
alphaNumber = calcAlphaNumber(road);
alphaCross = calcAlphaCross(road);
alphaPeople = calcAlphaPeople(road);
alphaWidth = calcAlphaWidth(road);
capacity = c0 * alphaNumber * alphaCross * alphaPeople  * alphaWidth;
end

%%  计算基本通行能力 C0
function c0 = baseRoadCapacity(road)
  v = road.v;
  c0 = 1000 * v / (0.054 * v * v + 1.2 * v + 7);
end

%% 计算车道数对通行能力的影响
function alphaNumber = calcAlphaNumber(road)
alphaNumber = 0;
for i = 1:road.nCarriageway
  alphaNumber = alphaNumber + road.alphaNumbers(i);
end
end

%% 计算交叉路口的影响
function alphaCross = calcAlphaCross(road)
a = 0.635;  % 减速加速度
b = 1.66;   % 加速加速度
if road.type == 1 % 小区内部道路
  v = road.v;
  va = road.vmin;
  l = road.length;
  n = 1; % 交叉口个数 TODO
  alphaCross = (l / v) / ( (l/v) + (1/a + 1/b) * ( (-1 * n / (2*v)) * (v * v - va * va) + n * (v - va)));
elseif road.type == 2   % 小区周边道路，有信号灯
  v = road.v;
  l = road.length;
  n = 1;
  time = road.waitTime;
  alphaCross =  (l / v) / ( (l/v)  + n * (v / (2*a) + v / (2 * b) + time));
else
  error('Invalid road type!');
end
end

%% 计算行人的影响
function alphaPeople = calcAlphaPeople(road)
if road.type == 1
  alphaPeople = 0.63;
elseif road.type == 2
  alphaPeople = 0.90;
else
  error('Invalid road type!');
end
end

%% 计算道路宽度的影响
function alphaWidth = calcAlphaWidth(road)
  if road.width >= 3.50
    alphaWidth = 1;
  else
    alphaWidth = 0.312 * road.width - 0.085;
  end
end
