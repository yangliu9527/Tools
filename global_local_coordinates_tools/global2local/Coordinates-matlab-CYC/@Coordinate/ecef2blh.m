function rBLH=ecef2blh(obj,ecef)
%通过ecef坐标获取blh坐标系的坐标，blh的输出为纬经高，角度
%列向量表示
assert(size(ecef,1)==3);
X(1,:)=ecef(1,:);
Y(1,:)=ecef(2,:);
Z(1,:)=ecef(3,:);
s=sqrt(X.*X+Y.*Y);
theta=atan2(Z.*obj.a,s.*obj.b);
s3=sin(theta).*sin(theta).*sin(theta);
c3=cos(theta).*cos(theta).*cos(theta);

R2D=180/pi;
rBLH(1,:)=atan2(Z+obj.e_nd2.*obj.b*s3,s-obj.e2*obj.a.*c3);
rBLH(2,:)=atan2(Y,X);
rBLH(3,:)=s./cos(rBLH(1,:))-obj.a./sqrt(1-obj.e2.*sin(rBLH(1,:)).*sin(rBLH(1,:)));
rBLH(1:2,:)=rBLH(1:2,:)*R2D;




end