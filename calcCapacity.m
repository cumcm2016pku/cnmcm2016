function capacity = calcCapacity(road)
% CALCCAPACITY ����ͨ������
c0 = baseRoadCapacity(road);
alphaNumber = calcAlphaNumber(road);
alphaCross = calcAlphaCross(road);
alphaPeople = calcAlphaPeople(road);
alphaWidth = calcAlphaWidth(road);
capacity = c0 * alphaNumber * alphaCross * alphaPeople  * alphaWidth;
end

%%  �������ͨ������ C0
function c0 = baseRoadCapacity(road)
  v = road.v;
  c0 = 1000 * v / (0.054 * v * v + 1.2 * v + 7);
end

%% ���㳵������ͨ��������Ӱ��
function alphaNumber = calcAlphaNumber(road)
alphaNumber = 0;
for i = 1:road.nCarriageway
  alphaNumber = alphaNumber + road.alphaNumbers(i);
end
end

%% ���㽻��·�ڵ�Ӱ��
function alphaCross = calcAlphaCross(road)
a = 0.635;  % ���ټ��ٶ�
b = 1.66;   % ���ټ��ٶ�
if road.type == 1 % С���ڲ���·
  v = road.v;
  va = road.vmin;
  l = road.length;
  n = 1; % ����ڸ��� TODO
  alphaCross = (l / v) / ( (l/v) + (1/a + 1/b) * ( (-1 * n / (2*v)) * (v * v - va * va) + n * (v - va)));
elseif road.type == 2   % С���ܱߵ�·�����źŵ�
  v = road.v;
  l = road.length;
  n = 1;
  time = road.waitTime;
  alphaCross =  (l / v) / ( (l/v)  + n * (v / (2*a) + v / (2 * b) + time));
else
  error('Invalid road type!');
end
end

%% �������˵�Ӱ��
function alphaPeople = calcAlphaPeople(road)
if road.type == 1
  alphaPeople = 0.63;
elseif road.type == 2
  alphaPeople = 0.90;
else
  error('Invalid road type!');
end
end

%% �����·��ȵ�Ӱ��
function alphaWidth = calcAlphaWidth(road)
  if road.width >= 3.50
    alphaWidth = 1;
  else
    alphaWidth = 0.312 * road.width - 0.085;
  end
end
