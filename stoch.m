function X = stoch(T, Q, thita, r, s) 
n=size(T,1); 

%初始化 
L=zeros(n,n); 
W=zeros(n,n); 
X=zeros(n,n); 

Tmin=floyd(T); 
R=Tmin(r,:) ;
S=Tmin(:,s)';%注意因为方向性，这里作转置处理 

%找上游节点和下游节点（显然up和down互为转置——对称，因为若j是i的下游节点，则i必是j的上游节点） 
for i=1:n 
    for j=1:n 
        if T(i,j)~=0&&T(i,j)~=inf 
            down(i,j)=1; 
            up(j,i)=1; 
        else 
            down(i,j)=0; 
            up(j,i)=0; 
        end 
    end 
end 

%计算边权 
for i=1:n 
    for j=1:n 
        if down(i,j)~=0 
            if R(i)<R(j)&&S(i)>S(j) 
                P=1; 
            else 
                P=0; 
            end 
            L(i,j)=P*exp(thita*(R(j)-R(i)-T(i,j))); 
        end 
    end 
end 

%计算路权  
for i=1:n 
    for j=1:n 
        if down(i,j)~=0 
            if R(i)<R(j)&&S(i)>S(j) 
                if i==r 
                    W(i,j)=L(i,j); 
                else 
                    W(i,j)=L(i,j)*(up(i,:)*W(:,i)); 
                    %这是核心句,先找到上游节点，再写出上游节点到i的W值（若还没有，则递归直到能够算出）,请仔细查阅路权的算法好好理解。 
                end 
            end 
        end 
    end 
end 

%配流  
for i=n:-1:1 
    for j=n:-1:1 
        if down(i,j)~=0 
            if R(i)<R(j)&&S(i)>S(j) 
                if j==s 
                    X(i,j)=Q*W(i,j)/((up(j,:)*W(:,j))); 
                else 
                    X(i,j)=X(j,:)*down(j,:)'*W(i,j)/(up(j,:)*W(:,j)); 
                    %注意X（j,:)是1*n行向量，down(j,:)是1*n行向量,因此要将down向量转置为1*n的列向量才能相乘。 
                end 
            end 
        end 
    end 
end 