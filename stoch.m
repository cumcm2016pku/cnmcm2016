function X = stoch(T, Q, thita, r, s) 
n=size(T,1); 

%��ʼ�� 
L=zeros(n,n); 
W=zeros(n,n); 
X=zeros(n,n); 

Tmin=floyd(T); 
R=Tmin(r,:) ;
S=Tmin(:,s)';%ע����Ϊ�����ԣ�������ת�ô��� 

%�����νڵ�����νڵ㣨��Ȼup��down��Ϊת�á����Գƣ���Ϊ��j��i�����νڵ㣬��i����j�����νڵ㣩 
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

%�����Ȩ 
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

%����·Ȩ  
for i=1:n 
    for j=1:n 
        if down(i,j)~=0 
            if R(i)<R(j)&&S(i)>S(j) 
                if i==r 
                    W(i,j)=L(i,j); 
                else 
                    W(i,j)=L(i,j)*(up(i,:)*W(:,i)); 
                    %���Ǻ��ľ�,���ҵ����νڵ㣬��д�����νڵ㵽i��Wֵ������û�У���ݹ�ֱ���ܹ������,����ϸ����·Ȩ���㷨�ú���⡣ 
                end 
            end 
        end 
    end 
end 

%����  
for i=n:-1:1 
    for j=n:-1:1 
        if down(i,j)~=0 
            if R(i)<R(j)&&S(i)>S(j) 
                if j==s 
                    X(i,j)=Q*W(i,j)/((up(j,:)*W(:,j))); 
                else 
                    X(i,j)=X(j,:)*down(j,:)'*W(i,j)/(up(j,:)*W(:,j)); 
                    %ע��X��j,:)��1*n��������down(j,:)��1*n������,���Ҫ��down����ת��Ϊ1*n��������������ˡ� 
                end 
            end 
        end 
    end 
end 