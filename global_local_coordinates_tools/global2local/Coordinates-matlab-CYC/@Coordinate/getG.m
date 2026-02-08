function g=getG(obj,blh)
%获取重力，blh的输入为纬经高，角度
%支持列向量操作，即3*n的向量，每一列为一个纬经高
assert(size(blh,1)==3);

B=blh(1,:)*obj.D2R;
h=blh(3,:);

si2=sin(B).^2;
co2=cos(B).^2;

m=obj.wie.^2*obj.a.^2.*obj.b./obj.GM;
k=2*h.*(1+obj.f+m-2.*obj.f.*si2)./obj.a;
gamma=obj.a*obj.grav_equ.*co2+obj.b*obj.grav_pol*si2;
gamma=gamma./(sqrt(obj.a.^2.*co2+obj.b.^2.*si2));
tmpgh=gamma.*(1-k+3*h.^2./(obj.a^2));

g=zeros(3,length(tmpgh));
g(3,:)=tmpgh;

end