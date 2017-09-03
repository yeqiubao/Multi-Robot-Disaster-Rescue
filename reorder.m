function [fr,co,info,ut] = reorder(fro,cost,infor)
ut=infor-cost;
[utt,ind]=sort(ut);
indf=fliplr(ind);
ut=fliplr(utt);
fr=zeros(size(exa));co=zeros(size(cos));info=zeros(size(fro));
for j=1:n1
info(j)=infor(indf(j));
co(j)=cos(indf(j));
fr(:,j)=fro(:,indf(j));
end
end

