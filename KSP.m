function [W_Section,Line_Section_01Mat,Mxf]=KSP(Road_Net,Line_Station)
%% 问题描述
%已知路网数据和线路节点数据，求指定点之间的K短路。

%% 程序功能
%路段自动编号（上下行作为2个路段分别对待）
%将路径-节点转换为路径-路段
%求出一般定义的K短路
%求出各条路径的换乘点
%求出各条路径的换乘费用
%求出各条路径的总费用（不换乘&换乘）
%求出路径与路段的关系矩阵（作为Frank_Wolfe算法的已知条件）
%本程序中的换乘仅指一次换乘，本研究假定不会选择三次以及以上换乘次数

%% 变量说明
%                                         KPaths: K短路路径集合
%                                        KCosts-: K短路费用集合
%                                           Kmax: 指定的K短路路径数量（若大于全部路径数，多余的部分会自动舍去）
%                                              a: 路段编号
%                                              P: 保存当前已找到的路径
%                                              X: P的子集，表示一个路径
%                                    path_number: 路径编号
%                                      current_P: 当前路径编号（P_）
%                                         size_X: X的大小
%                                w_index_in_path: 偏差节点在当前路径中的位置
%                                       KSP.: 结构体，为了集中展现结果
%                                       Road_Net: 路网矩阵，带权的邻接方阵
%                                   Line_Station: 路网路线集合
%                                Section_Station: 路段(Section)是由哪两个节点(Station)组成的
%                                      W_Section: 路段权重
%                                 W_Line_Section: 路线上各路段的权重
%                                       Original: 起点
%                                    Destination: 终点
%                                         path_0: 直达路线（一条直达路线上可能有不只一个的直达路径）
%                               Transfer_Station: 可能的换乘点
%                               Cost_of_Transfer: 换乘一次要增加的费用
%                                              H: 可容忍的绕路倍数
%                                        path_k1: 途经起始点到第一个换乘点的所有点的直达路线集合
%                                        path_k2: 途经所有换乘点的路线集合
%                                        path_k3: 途经最后一个换乘点到终点的所有点路径集合
%     Irrespective_of_the_Transfer_KPaths_number: 不考虑换乘的K短路按升序排列编号
%            Irrespective_of_the_Transfer_KPaths: 不考虑换乘的K短路路径
%            Irrespective_of_the_Transfer_KCosts: 不考虑换乘的K短路费用
%                       Transfer_Station_of_Path: 路径上的换乘点
%                              Times_of_Transfer: 换乘次数
%                             KCosts_of_Transfer: 换乘的额外费用
%                   Consider_the_Transfer_KCosts: 考虑换乘的费用
%            Consider_not_consider_KPaths_number: 考虑换乘后的路径排序（对应于不考虑换乘的排序）
%                   Consider_the_Transfer_KPaths: 考虑换乘后升序的K短路费用


%% 模块1:输入并处理相关的路网数据
%例
% Road_Net =[
%      0     2   Inf     1   Inf   Inf   Inf   Inf   Inf
%      2     0     2   Inf     3   Inf   Inf   Inf   Inf
%    Inf     3     0   Inf   Inf     2   Inf   Inf   Inf
%      1   Inf   Inf     0     3   Inf     5   Inf   Inf
%    Inf     3   Inf     3     0     1   Inf     2   Inf
%    Inf   Inf     2   Inf     1     0   Inf   Inf     3
%    Inf   Inf   Inf     5   Inf   Inf     0     2   Inf
%    Inf   Inf   Inf   Inf     2   Inf     2     0     4
%    Inf   Inf   Inf   Inf   Inf     3   Inf     4     0];
% Line_Station={[1 2 3 6 9 8 7 4],[2 5 8 9],[1 4 5 6]};
Original=input('Original=');
Destination=input('Destination=');
tic

%% 定义变量及常量
Cost_of_Transfer=2.5;                                             %预定义换乘时间，本文的换乘时间指平均换乘时间，只要发生换乘就会多话费这么多时间
H=1.5;                                                            %预定义的最大绕路倍数
a=0;                                                              %路段编号初始化
Kmax=100;                                                         %预定义的路径数量，这里尽可能取大，因为绕路倍数能够自动过滤不需要计算的k(启发式算法）

%% 根据路网对路段进行自动标号并求出每个路段的距离
for i=1:length(Road_Net)
    for j=1:length(Road_Net)
        if Road_Net(i,j)~=0&Road_Net(i,j)~=inf
            a=a+1;
            [Section_Station{a},W_Section{a}]=dijkstra(Road_Net,i,j);
        end
    end
end

%% 根据路段标号和路线-节点关系，求出路线-路段的0-1矩阵
Line_Section_01Mat=zeros(length(Line_Station),length(Section_Station));           %线路-路径0-1矩阵，表示某线路是否包含该路段
m=1;
while m<=length(Line_Station)
    for i=2:length(Line_Station{m})
        for k=1:length(Section_Station)
            if isequal(Section_Station{k},Line_Station{m}([i-1 i]))
                Line_Section_01Mat(m,k)=1;
            end
        end
    end
    m=m+1;
end

%% 路线-节点改为路线-路段
for i=1:length(Line_Station)
    a=0;
    for j=2:length(Line_Station{i})
        for k=1:length(Section_Station)
            if Line_Station{i}([j-1,j])==Section_Station{k}
                a=a+1;
                Line_Section{i}(a)=k;
                W_Line_Section{i}(a)=W_Section(k);
            end
        end
    end
end
%% 模块2:K短路算法
%% Step_0判断可行性
if Original > size(Road_Net,1)|| Destination > size(Road_Net,1)
    warning('起点或终点不在指定路网中！');
    KPaths=[];
    KCosts=[];
else
    
    %% Step_1调用Dijkstra算法求出最短路路径及费用
    k=1;
    [path, costs]=dijkstra(Road_Net, Original, Destination);
    if isempty(path)
        KPaths=[];
        KCosts=[];
    else
        
        %% Step_2初始化
        path_number = 1;
        P{path_number,1}= path;
        P{path_number,2}= costs;
        current_P = path_number;
        size_X=1;
        X{size_X}={path_number; path; costs};
        S(path_number)= path(1);
        KPaths{k}= path;
        KCosts{k}= costs;
        
        
        while (k<Kmax && size_X ~=0)
            %% 删除超出的路径和值
            if  KCosts{k}>(H+1)*costs                                  %此处(H+1)表示即便是直达线路，只要超过最短线路的(H+1)倍，也应该放弃，说明该条直达线路设置不合理，该步骤是为了终止过多无用的K路径。
                k=k-1;
                KCosts=KCosts(1:k);
                KPaths=KPaths(1:k);
                break
            end
            
            %% Step_3关闭已搜索的路径
            for i=1:length(X)
                if  X{i}{1}== current_P
                    size_X = size_X - 1;
                    X(i)=[];
                    break;
                end
            end
            P_= P{current_P,1};
            
            %% Step_4找偏差节点w的位置i
            w = S(current_P);
            for i=1:length(P_)
                if w==P_(i)
                    w_index_in_path=i;
                end
            end
            
            %% Step_5更新路网矩阵
            for index_dev_vertex= w_index_in_path:length(P_)- 1
                temp_luwangjuzhen = Road_Net;
                for i = 1: index_dev_vertex-1
                    v = P_(i);
                    temp_luwangjuzhen(v,:)=inf;
                    temp_luwangjuzhen(:,v)=inf;
                end
                SP_sameSubPath=[];
                index =1;
                SP_sameSubPath{index}=P_;
                for i=1:length(KPaths)
                    if length(KPaths{i})>= index_dev_vertex
                        if P_(1:index_dev_vertex)== KPaths{i}(1:index_dev_vertex)
                            index = index+1;
                            SP_sameSubPath{index}=KPaths{i};
                        end
                    end
                end
                v_ = P_(index_dev_vertex);
                for j = 1: length(SP_sameSubPath)
                    next=SP_sameSubPath{j}(index_dev_vertex+1);
                    temp_luwangjuzhen(v_,next)=inf;
                end
                
                %% Step_6计算偏差顶点前的子路径费用
                sub_P=P_(1:index_dev_vertex);
                costs_sub_P=0;
                for i=1:length(sub_P)-1
                    costs_sub_P=costs_sub_P+Road_Net(sub_P(i),sub_P(i+1));
                end
                
                %% Step_7计算偏差顶点到终点的路径及费用
                [dev_p, c]= dijkstra(temp_luwangjuzhen, P_(index_dev_vertex), Destination);
                if ~isempty(dev_p)
                    path_number=path_number+1;
                    P{path_number,1}=[sub_P(1:end-1) dev_p];      %连接起点到终点的路径
                    P{path_number,2}= costs_sub_P + c ;           %计算子路径及偏差定点到终点费用的和（最终费用）
                    S(path_number)= P_(index_dev_vertex);
                    size_X = size_X + 1;
                    X{size_X}={path_number; P{path_number,1};P{path_number,2}};
                    %                                               更新当前数据（路径编号，路径，路径费用）
                end
            end
            
            %% Step_8防错处理，如果指定路径数目大于路网穷举数目，防错，否则最后的结果会发生重复。
            if size_X > 0
                shortestXCosts= X{1}{3};                          %路径费用
                shortestX= X{1}{1};                               %判定路径
                for i=2:size_X
                    if  X{i}{3}< shortestXCosts
                        shortestX= X{i}{1};
                        shortestXCosts= X{i}{3};
                    end
                end
                current_P=shortestX;
                k=k+1;
                KPaths{k}= P{current_P,1};
                KCosts{k}= P{current_P,2};
            else
                k=k+1;
            end
        end
    end
end

%% 模块3:换乘算法
%% Step_0找直达线路
if isempty(KPaths)==0
    for i=1:length(Line_Station)
        pq(i)=ismember(Original,Line_Station{i});                             %起点是否是路线i上的节点
        pz(i)=ismember(Destination,Line_Station{i});                          %终点是否是路线i上的节点
    end
    S=find(pq==1);                                                    %经过起点的线路
    T=find(pz==1);                                                    %经过终点的线路
    path_0=intersect(S,T);                                            %直达线路
    if isempty(path_0)==0
        disp(['起点和终点间有直达线路:',num2str(path_0)]);
    else
        disp('无直达线路，请选择换乘方案');
    end
    
    %% Step_1找路网上的换乘节点
    n=0;
    for i=1:length(Line_Station)
        for j=1:length(Line_Station)
            if i>j
                n=n+1;
                Transfer_Station{n}=intersect(Line_Station{i},Line_Station{j});
            end
        end
    end
    Transfer_Station=setdiff(unique(cell2mat(Transfer_Station)),[Original,Destination]);
    if isempty(Transfer_Station)
        disp('提示：无一次换乘点.');
    else
        disp(['提示：若选择换乘,可能的换乘站有:',num2str(Transfer_Station)]);
    end
    
    %% Step_2确定路径上换乘节点
    for i=1:length(KPaths)
        Transfer_Station_of_Path{i}=intersect(intersect(intersect([Line_Station{S}],[Line_Station{T}]),Transfer_Station),KPaths{i});
    end
    
    %% Step_3初始化路径的前段、中段、末段路径，并标记前中末段位置
    path_k1=cell(1,Kmax);
    path_k2=cell(1,Kmax);
    path_k3=cell(1,Kmax);
    
    index_first_Trasnfer=cell(1,length(KPaths));
    index_Last_Trasnfer=cell(1,length(KPaths));
    for i=1:length(KPaths)
        k1{i}=1;
        k2{i}=length(KPaths{i});
    end
    %标记第一个换乘点
    for i=1:length(KPaths)
        num_HCD=0;
        if isempty(Transfer_Station_of_Path{i})==0
            while k1{i}<length(KPaths{i})
                k1{i}=k1{i}+1;
                for j=1:length(Line_Station)
                    if ismember(KPaths{i}(k1{i}),Transfer_Station_of_Path{i})&all(ismember(KPaths{i}(1:k1{i}),Line_Station{j}))
                        index_first_Trasnfer{i}=k1{i};
                    end
                end
            end
        else
            index_first_Trasnfer{i}=k1{i};
        end
    end
    %标记第二个换乘点
    k1=index_first_Trasnfer;
    for i=1:length(KPaths)
        if isempty(Transfer_Station_of_Path{i})==0
            while k2{i}>=k1{i}
                for j=1:length(Line_Station)
                    if ismember(KPaths{i}(k2{i}),Transfer_Station_of_Path{i})&all(ismember(KPaths{i}(k2{i}:end),Line_Station{j}))
                        index_Last_Trasnfer{i}=k2{i};
                    end
                end
                k2{i}=k2{i}-1;
            end
        else
            index_Last_Trasnfer{i}=k2{i};
        end
    end
    
    %% Step_4求每个路径的前中后段路径集合
    for i=1:length(KPaths)
        n1=0;n2=0;n3=0;
        for j=1:length(Line_Station)
            if all(ismember(KPaths{i}(1:index_first_Trasnfer{i}),Line_Station{j}))
                n1=n1+1;
                path_k1{i}(n1)=j;
            end
            if all(ismember(KPaths{i}(index_first_Trasnfer{i}:index_Last_Trasnfer{i}),Line_Station{j}))
                n2=n2+1;
                path_k2{i}(n2)=j;
            end
            if all(ismember(KPaths{i}(index_Last_Trasnfer{i}:end),Line_Station{j}))
                n3=n3+1;
                path_k3{i}(n3)=j;
            end
        end
    end
    
    %% Step_5求换乘次数和换乘费用
    for i=1:length(KPaths)
        if isempty(Transfer_Station_of_Path{i})
            Times_of_Transfer{i}=0;
            KCosts_of_Transfer{i}=0;
            Transfer_Station_of_Path{i}=[];
        else
            One_time_Line{i}=union(intersect(path_k1{i},path_k2{i}),intersect(path_k2{i},path_k3{i}));
            Direct_Line{i}=intersect(intersect(path_k1{i},path_k2{i}),path_k3{i});
            if isempty(Direct_Line{i})==0
                Times_of_Transfer{i}=0;
                KCosts_of_Transfer{i}=0;
                Transfer_Station_of_Path{i}=[];
            elseif isempty(One_time_Line{i})==0
                Times_of_Transfer{i}=1;
                KCosts_of_Transfer{i}=Cost_of_Transfer*1;
                Transfer_Station_of_Path{i}=KPaths{i}(index_first_Trasnfer{i});
            else
                if isempty(path_k2{i})
                    Times_of_Transfer{i}=3;
                    KCosts_of_Transfer{i}=inf;
                    Transfer_Station_of_Path{i}=intersect(KPaths{i}(index_first_Trasnfer{i}:index_Last_Trasnfer{i}),Transfer_Station);
                else
                    Times_of_Transfer{i}=2;
                    KCosts_of_Transfer{i}=Cost_of_Transfer*2;
                    Transfer_Station_of_Path{i}=[KPaths{i}(index_first_Trasnfer{i}),KPaths{i}(index_Last_Trasnfer{i})];
                end
            end
        end
    end
    
    %% Step_6计算总费用
    for i=1:length(KCosts)
        Consider_the_Transfer_KCosts{i}=KCosts_of_Transfer{i}+KCosts{i};
    end
    
    %% Step_7数据结构体化
    KSP.Road_Net={Road_Net};
    KSP.Line_Station=Line_Station;
    KSP.Section_Station=Section_Station;
    KSP.W_Section=W_Section;
    KSP.Line_Section=Line_Section;
    KSP.Line_Section_01Mat=Line_Section_01Mat;
    KSP.W_Line_Section=W_Line_Section;
    KSP.Original={Original};
    KSP.Destination={Destination};
    KSP.Kmax={Kmax};
    KSP.Cost_of_Transfer={Cost_of_Transfer};
    KSP.H={H};
    KSP.Transfer_Station={Transfer_Station};
    KSP.Irrespective_of_the_Transfer_KPaths=KPaths;
    KSP.Irrespective_of_the_Transfer_KCosts=KCosts;
    KSP.Transfer_Station_of_Path=Transfer_Station_of_Path;
    KSP.Times_of_Transfer=Times_of_Transfer;
    KSP.KCosts_of_Transfer=KCosts_of_Transfer;
    KSP.Consider_the_Transfer_KCosts=Consider_the_Transfer_KCosts;
    [feiyong,paixu]=sort(cell2mat(KSP.Consider_the_Transfer_KCosts));
    Direct_path =find(cell2mat(Times_of_Transfer)==0);
    KSP.Consider_not_consider_KPaths_number={paixu((feiyong<=H*min(feiyong)) | (ismember(paixu,Direct_path )==1))};
    KSP.Consider_not_consider_KPaths_Costs={feiyong((feiyong<=H*min(feiyong))| (ismember(paixu,Direct_path )==1))};
    KSP.Consider_the_Transfer_KPaths=KPaths(paixu((feiyong<=H*min(feiyong))  | (ismember(paixu,Direct_path )==1)));
    for i=1:length(KSP.Irrespective_of_the_Transfer_KCosts)
        KSP.Consider_the_Transfer_KCosts{i}=KSP.Irrespective_of_the_Transfer_KCosts{i}+KSP.KCosts_of_Transfer{i};
    end
    Last_KPaths=KSP.Consider_the_Transfer_KPaths;
    Last_KCosts=KSP.Consider_the_Transfer_KCosts;
    
    %% Step_8路径路段矩阵的自动铺画
    Mxf=zeros(length(Last_KPaths),length(Section_Station));
    m=1;
    while m<=length(Last_KPaths)
        for i=2:length(Last_KPaths{m})
            for k=1:length(Section_Station)
                if isequal(Section_Station{k},Last_KPaths{m}([i-1 i]))
                    Mxf(m,k)=1;
                end
            end
        end
        m=m+1;
    end
    KSP.Mxf={Mxf};
    
    %% 模块4:数据输出及提示
    %% ——》条件输出
    if Kmax>length(KPaths)
        fprintf('提示：满足距离要求的路径最多只有%d条，其中直达线路%d条，1次换乘可达的%d条，2次换乘可达的%d条，3次及以上换乘可达的%d条(已舍去）。\n',...
            length(Times_of_Transfer),sum(cell2mat(Times_of_Transfer)==0),sum(cell2mat(Times_of_Transfer)==1),...
            sum(cell2mat(Times_of_Transfer)==2),sum(cell2mat(Times_of_Transfer)==3));
    end
    %% ——》保存数据
    %     tic
    %     disp('正在保存数据，请耐心等待（Ctrl+C放弃保存）...');
    %     warning off
    %     save_to_excel
    %     disp('保存完成！');
    %     open('E:\MATLAB\自己的算法\Kduanlu_shuju.xls');
    %     disp('保存数据耗时:')
    %     toc
else
    KSP=[];
    Mxf=[];
end
KSP
disp('计算耗时:')
toc
end