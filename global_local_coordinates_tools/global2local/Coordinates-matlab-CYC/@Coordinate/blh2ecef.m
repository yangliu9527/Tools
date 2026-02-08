function ecef=blh2ecef(obj,blh)
%获取ecef坐标系的坐标，blh的输入为纬经高，角度
%支持列向量操作，即3*n的向量，每一列为一个纬经高
assert(size(blh,1)==3);

RN=obj.getRN(blh);

B=blh(1,:)*obj.D2R;
L=blh(2,:)*obj.D2R;
H=blh(3,:);

x=(RN+H).*cos(B).*cos(L);
y=(RN+H).*cos(B).*sin(L);
z=(RN.*(1-obj.e2)+H).*sin(B);

ecef=[x;y;z];



end