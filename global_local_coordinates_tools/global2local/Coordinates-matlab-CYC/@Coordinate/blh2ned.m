function local=blh2ned(obj,blh0,blh)
%将经纬高坐标位置转换为以某个点建立的n系为本地坐标系之中
%获取坐标系的坐标，blh的输入为纬经高，角度
%支持列向量操作，即3*n的向量，每一列为一个纬经高
assert(size(blh,1)==3);

len=size(blh,2);
rXYZ0=repmat(obj.blh2ecef(blh0),1,len);
qne0=pos2quat(blh0);
qen0=qinv(qne0);
local=zeros(3,len);
rXYZt(1:3,1:len)=obj.blh2ecef(blh);
del_rXYZ=rXYZt-rXYZ0;
for i=1:len
    local(:,i)=qmultvec(qen0,del_rXYZ(:,i));
end



% B=blh(1,:)*obj.D2R;
% L=blh(2,:)*obj.D2R;
% H=blh(3,:);
% 
% RNH=obj.getRN(blh)+H;
% RMH=obj.getRM(blh)+H;
% 
% B0=blh0(1,1)*obj.D2R;
% L0=blh0(2,1)*obj.D2R;
% H0=blh0(3,1);
% 
% e=(B-B0).*RMH;
% n=(L-L0).*RNH.*cos(B0);
% u=H-H0;
% 
% local=[e;n;u];
% local=[n;e;-u];







end