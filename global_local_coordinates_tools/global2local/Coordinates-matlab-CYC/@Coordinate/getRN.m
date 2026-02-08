function RN=getRN(obj,blh)
%获取RN，blh的输入为纬经高，角度
%支持列向量操作，即3*n的向量，每一列为一个值，返回行向量
assert(size(blh,1)==3);

lat=blh(1,:)*obj.D2R;

RN=obj.a./sqrt(1-obj.e2*sin(lat).^2);





end