%% 画出通行能力��?有关�?�数的�?�化情况

myroad_ori = RoadInstance(2, 3.5, 900, 2, 50/3.6, 0, 30, 1, -1, -1);
myroad = RoadInstance(2, 3.5, 900, 2, 50/3.6, 0, 30, 1, -1, -1);
str = sprintRoad(myroad);
disp(sprintRoad(myroad));

%% 通行能力��?路段长度�?�化的情况

% 路段长度从0.1到1000米
maxlength = 1000;
steplength = 0.1;
capacity = zeros(maxlength / steplength, 1);
lengths =  0.1 : steplength : 1000;
for i = 1:length(lengths)
  myroad.length = lengths(i);
  capacity(i) = calcCapacity(myroad);
end
figure;
hold on;
plot(transpose(lengths), capacity);
xlabel('Length of the road');
ylabel('Road Capacity');

%% 通行能力��?行驶速度的�?�化情况
myroad = RoadInstance(2, 3.5, 900, 2, 50/3.6, 0, 30, 1, -1, -1);
maxv = 120/3.6;
stepv = 0.1;
vs = stepv: stepv : maxv;
capacities = zeros(size(vs));
for i = 1:length(vs)
  myroad.v = vs(i);
  capacities(i) = calcCapacity(myroad);
end
figure;
hold on;
plot(vs, capacities);
xlabel('Speed of the road');
ylabel('Road Capacity');


%% 通行能力��?行驶速度的�?�化情况
myroad = RoadInstance(2, 3.5, 900, 2, 50/3.6, 0, 30, 1, -1, -1);
maxv = 120/3.6;
stepv = 0.1;
vs = stepv: stepv : maxv;
capacities = zeros(size(vs));
for i = 1:length(vs)
  myroad.v = vs(i);
  capacities(i) = calcCapacity(myroad);
end
figure;
hold on;
plot(vs, capacities);
xlabel('Speed of the road');
ylabel('Road Capacity');

%% 通行能力��?交�?��?�个数�?�化的情况
myroad = getDemoRoad();
capacities = zeros(5,1);
for i = 1:5
  myroad.nCross = i;
  capacities(i) = calcCapacity(myroad);
end
figure;
hold on;
plot(1:5, capacities);
xlabel('number of crosses');
ylabel('Road Capacity');
