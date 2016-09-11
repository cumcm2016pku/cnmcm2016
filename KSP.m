function [W_Section,Line_Section_01Mat,Mxf]=KSP(Road_Net,Line_Station)
%% ��������
%��֪·�����ݺ���·�ڵ����ݣ���ָ����֮���K��·��

%% ������
%·���Զ���ţ���������Ϊ2��·�ηֱ�Դ���
%��·��-�ڵ�ת��Ϊ·��-·��
%���һ�㶨���K��·
%�������·���Ļ��˵�
%�������·���Ļ��˷���
%�������·�����ܷ��ã�������&���ˣ�
%���·����·�εĹ�ϵ������ΪFrank_Wolfe�㷨����֪������
%�������еĻ��˽�ָһ�λ��ˣ����о��ٶ�����ѡ�������Լ����ϻ��˴���

%% ����˵��
%                                         KPaths: K��··������
%                                        KCosts-: K��·���ü���
%                                           Kmax: ָ����K��··��������������ȫ��·����������Ĳ��ֻ��Զ���ȥ��
%                                              a: ·�α��
%                                              P: ���浱ǰ���ҵ���·��
%                                              X: P���Ӽ�����ʾһ��·��
%                                    path_number: ·�����
%                                      current_P: ��ǰ·����ţ�P_��
%                                         size_X: X�Ĵ�С
%                                w_index_in_path: ƫ��ڵ��ڵ�ǰ·���е�λ��
%                                       KSP.: �ṹ�壬Ϊ�˼���չ�ֽ��
%                                       Road_Net: ·�����󣬴�Ȩ���ڽӷ���
%                                   Line_Station: ·��·�߼���
%                                Section_Station: ·��(Section)�����������ڵ�(Station)��ɵ�
%                                      W_Section: ·��Ȩ��
%                                 W_Line_Section: ·���ϸ�·�ε�Ȩ��
%                                       Original: ���
%                                    Destination: �յ�
%                                         path_0: ֱ��·�ߣ�һ��ֱ��·���Ͽ����в�ֻһ����ֱ��·����
%                               Transfer_Station: ���ܵĻ��˵�
%                               Cost_of_Transfer: ����һ��Ҫ���ӵķ���
%                                              H: �����̵���·����
%                                        path_k1: ;����ʼ�㵽��һ�����˵�����е��ֱ��·�߼���
%                                        path_k2: ;�����л��˵��·�߼���
%                                        path_k3: ;�����һ�����˵㵽�յ�����е�·������
%     Irrespective_of_the_Transfer_KPaths_number: �����ǻ��˵�K��·���������б��
%            Irrespective_of_the_Transfer_KPaths: �����ǻ��˵�K��··��
%            Irrespective_of_the_Transfer_KCosts: �����ǻ��˵�K��·����
%                       Transfer_Station_of_Path: ·���ϵĻ��˵�
%                              Times_of_Transfer: ���˴���
%                             KCosts_of_Transfer: ���˵Ķ������
%                   Consider_the_Transfer_KCosts: ���ǻ��˵ķ���
%            Consider_not_consider_KPaths_number: ���ǻ��˺��·�����򣨶�Ӧ�ڲ����ǻ��˵�����
%                   Consider_the_Transfer_KPaths: ���ǻ��˺������K��·����


%% ģ��1:���벢������ص�·������
%��
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

%% �������������
Cost_of_Transfer=2.5;                                             %Ԥ���廻��ʱ�䣬���ĵĻ���ʱ��ָƽ������ʱ�䣬ֻҪ�������˾ͻ�໰����ô��ʱ��
H=1.5;                                                            %Ԥ����������·����
a=0;                                                              %·�α�ų�ʼ��
Kmax=100;                                                         %Ԥ�����·�����������ﾡ����ȡ����Ϊ��·�����ܹ��Զ����˲���Ҫ�����k(����ʽ�㷨��

%% ����·����·�ν����Զ���Ų����ÿ��·�εľ���
for i=1:length(Road_Net)
    for j=1:length(Road_Net)
        if Road_Net(i,j)~=0&Road_Net(i,j)~=inf
            a=a+1;
            [Section_Station{a},W_Section{a}]=dijkstra(Road_Net,i,j);
        end
    end
end

%% ����·�α�ź�·��-�ڵ��ϵ�����·��-·�ε�0-1����
Line_Section_01Mat=zeros(length(Line_Station),length(Section_Station));           %��·-·��0-1���󣬱�ʾĳ��·�Ƿ������·��
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

%% ·��-�ڵ��Ϊ·��-·��
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
%% ģ��2:K��·�㷨
%% Step_0�жϿ�����
if Original > size(Road_Net,1)|| Destination > size(Road_Net,1)
    warning('�����յ㲻��ָ��·���У�');
    KPaths=[];
    KCosts=[];
else
    
    %% Step_1����Dijkstra�㷨������··��������
    k=1;
    [path, costs]=dijkstra(Road_Net, Original, Destination);
    if isempty(path)
        KPaths=[];
        KCosts=[];
    else
        
        %% Step_2��ʼ��
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
            %% ɾ��������·����ֵ
            if  KCosts{k}>(H+1)*costs                                  %�˴�(H+1)��ʾ������ֱ����·��ֻҪ���������·��(H+1)����ҲӦ�÷�����˵������ֱ����·���ò������ò�����Ϊ����ֹ�������õ�K·����
                k=k-1;
                KCosts=KCosts(1:k);
                KPaths=KPaths(1:k);
                break
            end
            
            %% Step_3�ر���������·��
            for i=1:length(X)
                if  X{i}{1}== current_P
                    size_X = size_X - 1;
                    X(i)=[];
                    break;
                end
            end
            P_= P{current_P,1};
            
            %% Step_4��ƫ��ڵ�w��λ��i
            w = S(current_P);
            for i=1:length(P_)
                if w==P_(i)
                    w_index_in_path=i;
                end
            end
            
            %% Step_5����·������
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
                
                %% Step_6����ƫ���ǰ����·������
                sub_P=P_(1:index_dev_vertex);
                costs_sub_P=0;
                for i=1:length(sub_P)-1
                    costs_sub_P=costs_sub_P+Road_Net(sub_P(i),sub_P(i+1));
                end
                
                %% Step_7����ƫ��㵽�յ��·��������
                [dev_p, c]= dijkstra(temp_luwangjuzhen, P_(index_dev_vertex), Destination);
                if ~isempty(dev_p)
                    path_number=path_number+1;
                    P{path_number,1}=[sub_P(1:end-1) dev_p];      %������㵽�յ��·��
                    P{path_number,2}= costs_sub_P + c ;           %������·����ƫ��㵽�յ���õĺͣ����շ��ã�
                    S(path_number)= P_(index_dev_vertex);
                    size_X = size_X + 1;
                    X{size_X}={path_number; P{path_number,1};P{path_number,2}};
                    %                                               ���µ�ǰ���ݣ�·����ţ�·����·�����ã�
                end
            end
            
            %% Step_8���������ָ��·����Ŀ����·�������Ŀ�������������Ľ���ᷢ���ظ���
            if size_X > 0
                shortestXCosts= X{1}{3};                          %·������
                shortestX= X{1}{1};                               %�ж�·��
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

%% ģ��3:�����㷨
%% Step_0��ֱ����·
if isempty(KPaths)==0
    for i=1:length(Line_Station)
        pq(i)=ismember(Original,Line_Station{i});                             %����Ƿ���·��i�ϵĽڵ�
        pz(i)=ismember(Destination,Line_Station{i});                          %�յ��Ƿ���·��i�ϵĽڵ�
    end
    S=find(pq==1);                                                    %����������·
    T=find(pz==1);                                                    %�����յ����·
    path_0=intersect(S,T);                                            %ֱ����·
    if isempty(path_0)==0
        disp(['�����յ����ֱ����·:',num2str(path_0)]);
    else
        disp('��ֱ����·����ѡ�񻻳˷���');
    end
    
    %% Step_1��·���ϵĻ��˽ڵ�
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
        disp('��ʾ����һ�λ��˵�.');
    else
        disp(['��ʾ����ѡ�񻻳�,���ܵĻ���վ��:',num2str(Transfer_Station)]);
    end
    
    %% Step_2ȷ��·���ϻ��˽ڵ�
    for i=1:length(KPaths)
        Transfer_Station_of_Path{i}=intersect(intersect(intersect([Line_Station{S}],[Line_Station{T}]),Transfer_Station),KPaths{i});
    end
    
    %% Step_3��ʼ��·����ǰ�Ρ��жΡ�ĩ��·���������ǰ��ĩ��λ��
    path_k1=cell(1,Kmax);
    path_k2=cell(1,Kmax);
    path_k3=cell(1,Kmax);
    
    index_first_Trasnfer=cell(1,length(KPaths));
    index_Last_Trasnfer=cell(1,length(KPaths));
    for i=1:length(KPaths)
        k1{i}=1;
        k2{i}=length(KPaths{i});
    end
    %��ǵ�һ�����˵�
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
    %��ǵڶ������˵�
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
    
    %% Step_4��ÿ��·����ǰ�к��·������
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
    
    %% Step_5�󻻳˴����ͻ��˷���
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
    
    %% Step_6�����ܷ���
    for i=1:length(KCosts)
        Consider_the_Transfer_KCosts{i}=KCosts_of_Transfer{i}+KCosts{i};
    end
    
    %% Step_7���ݽṹ�廯
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
    
    %% Step_8·��·�ξ�����Զ��̻�
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
    
    %% ģ��4:�����������ʾ
    %% �������������
    if Kmax>length(KPaths)
        fprintf('��ʾ���������Ҫ���·�����ֻ��%d��������ֱ����·%d����1�λ��˿ɴ��%d����2�λ��˿ɴ��%d����3�μ����ϻ��˿ɴ��%d��(����ȥ����\n',...
            length(Times_of_Transfer),sum(cell2mat(Times_of_Transfer)==0),sum(cell2mat(Times_of_Transfer)==1),...
            sum(cell2mat(Times_of_Transfer)==2),sum(cell2mat(Times_of_Transfer)==3));
    end
    %% ��������������
    %     tic
    %     disp('���ڱ������ݣ������ĵȴ���Ctrl+C�������棩...');
    %     warning off
    %     save_to_excel
    %     disp('������ɣ�');
    %     open('E:\MATLAB\�Լ����㷨\Kduanlu_shuju.xls');
    %     disp('�������ݺ�ʱ:')
    %     toc
else
    KSP=[];
    Mxf=[];
end
KSP
disp('�����ʱ:')
toc
end