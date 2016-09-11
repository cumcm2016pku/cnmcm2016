function D=floyd(graph)
n=length(graph);
D=graph;
m=1;
while m<=n
    for i=1:n
        for j=1:n
            if D(i,j)>D(i,m)+D(m,j)
                D(i,j)=D(i,m)+D(m,j);
            end
        end
    end
    m=m+1;
end