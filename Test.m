%% 算法符号及程序说明
%说明：本程序为采用美国联邦公路阻抗函数BPR时的frankwolfe算法，考虑了换乘（已经将等待时
%间考虑在内并在K短路的确定过程中计算）及拥挤附加时间，在路网情况已知时配流并求出费用值。
%计算条件：根据K短路算法计算出考虑换乘的路径-路段关系矩阵和边权。
%缺点:BPR函数或许不能准确的描述轨道交通系统的路段阻抗，列车的能力也不能用程序中那么简单
%代替（但是，这无关紧要，暂时可以认为是正确的，后续可以用已知的路段函元胞和能力矩阵替换）
%下一步研究方向：网络配流--所有OD对之间的配流。
%==============================================================================
%符号体系：
%   Q-OD需求量
%  Cs-路段额定能力（列车座位数）
%Cmax-路段极限能力（列车座位数+人均面积达标的时的站立人数）
%   t-路段阻抗函数（值）
%numf-起点到终点的路径数量（自己按照K短路算法进行定义）
%numx-网络中的路段数量（直接连接两节点的边数，包含所有路段）
%   W-边权（0流量时的阻抗）,在该程序中，通过调用K短路换乘算法自动标号索引得出。
%Transfer_time-某路径的路段换乘时间
%   G-路段拥挤附加时间
%   A-一般拥挤放大系数(介于0到等车时间之间），可以看作一个综合系数，指部分人选择挤车，部分人选择下趟列车。
%   B-特别拥挤放大系数（等车时间，与发车频率对应），如果过度拥挤，则无法乘车的人必须等待后车到来，等1趟时间加B，等2趟，时间加2B，等几趟由无法乘车人数与极限能力决定。
%   L-路网路线矩阵（0-1矩阵，表示路线与路段的关系，1为包含，0为不包含）
% Mxf-路径与路段的关系矩阵（每一行表示一个路径，矩阵值为1表示经过该路段，否则不经过该路段）
%cont-当前配流次数
%Ckrs-路径配流结果（更新后的阻抗值）
%  X1-当前配流结果
%  Y1-辅助流量
%   S-搜索方向
%lmbda-搜索步长
%  X2-新的迭代点
%   Z-目标函数（通用配流目标函数，可认为是费用）
%==============================================================================
clear all
format
clc
warning off
disp('========================================================================');
disp('                           《FrankWolfe_BPR配流算法》');
disp('运行环境：MATLAB 8.3.0.532 ');
disp('制 作 人：兰州交通大学   刘志祥');
disp('完成时间：2015-12-30');
disp('Q      Q:531548824');
disp('请提出宝贵的意见!');
disp('=========================================================================');

disp('----------------------------------------------------------------------')
fprintf('问题描述：已知路网数据（Road_Net）和线路数据（Line_Station）及OD需求（Q），\n求指定OD对的配流结果.\n\n');
disp('----------------------------------------------------------------------')

%% 定义变量及常数
syms lambda real
for i=1:100
    syms x(i) real;
end
cont=0;
e=inf;
A=2;
B=5;
cs=1460;
cmax=2070;
Q=7500; 

%% 输入路网数据
%例1
% Road_Net=[0 15 5 inf;15 0 inf 20;5 inf 0 25;inf 20 25 0];             %路网关联矩阵，表示点到点的距离（或时间、费用）
%例2
Road_Net =[
     0     2   Inf     1   Inf   Inf   Inf   Inf   Inf
     2     0     3   Inf     3   Inf   Inf   Inf   Inf
   Inf     3     0   Inf   Inf     2   Inf   Inf   Inf
     1   Inf   Inf     0     3   Inf     5   Inf   Inf
   Inf     3   Inf     3     0     1   Inf     2   Inf
   Inf   Inf     2   Inf     1     0   Inf   Inf     3
   Inf   Inf   Inf     5   Inf   Inf     0     2   Inf
   Inf   Inf   Inf   Inf     2   Inf     2     0     4
   Inf   Inf   Inf   Inf   Inf     3   Inf     4     0];
Line_Station={[1 2 3 6 9 8 7 4],[2 5 8 9],[1 4 5 6]};

%% 调用KSP（K shortest path)计算出W,L,Mxf
[W_Section,Line_Section_01Mat,Mxf]=KSP(Road_Net,Line_Station); 
W=cell2mat(W_Section)
L=Line_Section_01Mat
Mxf=Mxf;
if isempty(Mxf)
    Mxf=zeros(1,length(W));
    disp('警告：无路径可达.');
end
disp('路径：（矩阵Mxf的行表示路径号，列表示路段号）');Mxf

%% 计算路网基础数据及路网表达
numf=size(Mxf,1);
numx=length(W);
Cs=cs.*ones(1,numx);
Cmax=cmax.*ones(1,numx);
x=x(1:numx);
disp('路网介绍')
disp(['总需求：',num2str(Q)]);
disp(['路径数：',num2str(numf)]);
disp(['路段数：',num2str(numx)]);
disp(['初始路段阻抗：',num2str(W)]);
disp(['路段额定能力：',num2str(Cs)]);
disp(['路段极限能力：',num2str(Cmax)]);
delete('FW_BPR算法计算过程.txt');
fid=fopen('FW_BPR算法计算过程.txt','a');

%% step1_初始化
X0=zeros(1,numx);
t=zeros(1,numx);
t=W.*(1+0.15*(x./Cmax).^4);                                             %路段走行时间函数
tt=t;
disp('路段阻抗函数t=');
disp(vpa(t.',2));
t=W.*(1+0.15*(X0./Cmax).^4);                                            %路段的初始阻抗函数
Ckrs=(Mxf*t')';                                                         %路径的走行时间初值
disp(['初始路径阻抗：',num2str(Ckrs)]);
%% step2_更新流量和阻抗
[Min,index]=min(Ckrs);
X1=Mxf(index,:).*Q;                                                     %全有全无法为最短路径上的路段分配流量

fprintf(fid,'\r\n==============================================================\r\n');
while e>1e-3
    cont=cont+1;
    
    %% 计算路段附加拥挤时间G
    G=zeros(1,length(W));
    a1=(X1<=Cs);                                                        %a1-指向流量不大于额定能力的下标
    a2=(X1>Cs&X1<=Cmax);                                                %a2-指向流量介于额定能力和极限能力的下标
    a3=(X1>Cmax);                                                       %a3-指向流量大于极限能力的下标
    G(a1)=0;
    G(a2)=(X1(a2)-Cs(a2))./Cs(a2).*A;
    G(a3)=(Cmax(a3)-Cs(a3))./Cs(a3).*A+(X1(a3)-Cmax(a3))./Cs(a3).*B;
    
    %% 计算路段阻抗函数并求出路径阻抗输出至指定文件
    t=(W+G).*(1+0.15*(X1./Cmax).^4);                                    %路段时间
    fprintf(fid,'                   <第%d次的计算结果>\r\n',cont);
    Ckrs=(Mxf*t')';                                                     %路径时间
    fprintf(fid,'阻 抗 值：');
    fprintf(fid,'%9.2f  ',Ckrs);
    fprintf(fid,'\r\n');
    
    %% 输出当前流量至指定文件
    fprintf(fid,'当前流量：');
    fprintf(fid,'%9.2f  ',X1);
    fprintf(fid,'\r\n');
    
    %% 输出输出拥挤附加时间至指定文件
    fprintf(fid,'拥挤附加：');
    fprintf(fid,'%9.2f  ',G);
    fprintf(fid,'\r\n');
    
    %% step3_计算辅助流量并输出至指定文件
    [Min,index]=min(Ckrs);
    Y1=Mxf(index,:).*Q;                                                 %全有全无法求辅助流量
    fprintf(fid,'辅助流量：');
    fprintf(fid,'%9.2f  ',Y1);
    fprintf(fid,'\r\n');
    
    %% step4_求搜索方向和步长并输出至指定文件
    S=Y1-X1;                                                            %搜索方向
    fprintf(fid,'搜索方向：');
    fprintf(fid,'%9.2f  ',S);
    fprintf(fid,'\r\n');     
    X2=X1+lambda*S;                                                     %先将X2用X1和lambda进行表达
    t=(W+G).*(1+0.15*(X2./Cmax).^4);                                    %含lambda的阻抗表达
    f=sum(S.*t,2);                                                      %2表示按行求和    
    if S==0
        lambda2=0;
    else        
        lambda1=double(solve(f));                                       %求解方程，确定步长。
        k=length(lambda1);                                              %如步长lambda1的解不唯一，取实数，且大于0小于1；
        for m=1:k
            if lambda1(m,1)>=0&&lambda1(m,1)<=1
                lambda2=lambda1(m,1);
            end
        end
    end
    fprintf(fid,'迭代步长：');
    fprintf(fid,'  lambda=%9.2f  ',lambda2);
    fprintf(fid,'\r\n');
    
    %% step5_确定新的迭代点并输出至指定文件
    X2=X1+lambda2*S;                                                    %得到下一步的流量值，且进行下一次迭代
    fprintf(fid,'新迭代点：');
    fprintf(fid,'%9.2f  ',X2);
    fprintf(fid,'\r\n');    
    
    %% step6_收敛性判断
    e=sqrt(sum((X2-X1).^2))/sum(X1);
    X1=X2;
    fprintf(fid,'\r\n==============================================================\r\n');
end

%% 求目标函数值
Xx=zeros(numx,1);                                                       %积分下界
Xn=X1;                                                                  %积分上界
Z=zeros(numx,1);
for i=1:numx
    Z(i)=int(tt(i),Xx(i),Xn(i));                                        %对每一个路径积分
end
Z=sum(Z);

%% step7_输出结果
disp(['     迭代次数：',num2str(cont)]);
disp(['     误 差 值：',num2str(e)]);
disp(['     配流结果：',num2str(Xn)]);
disp(['     路径阻抗：',num2str(Ckrs)]);
disp(['     目 标 值：',num2str(Z)]);
toc
disp('**************************************************************************************')
disp('提示：详细计算过程请打开 “FW_BPR算法计算过程.txt” 查看。');
status=fclose('all');
