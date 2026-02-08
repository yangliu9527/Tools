% 类型：函数
% 功能：根据n系经纬坐标求取此时n系相对于e系的姿态（四元数表示），qen为经纬度的单一映射满足函数关系，反之成立详见quat2pos。
% out：qne 四元数，n系相对于e系的姿态
% n是下标！
% in：BLH 纬经高 角度，角度，米

function qne = pos2quat( BLH )
D2R=pi/180;
Phi=BLH(1)*D2R;
Lambda=BLH(2)*D2R;
cL = cos(Lambda/2);%c1
sL = sin(Lambda/2);%s1
cB = cos(-pi/4 - Phi/2);%c2
sB = sin(-pi/4 - Phi/2);%s2

qne = [cB*cL; -sB*sL; sB*cL; cB*sL];

qne = (quatnormalize(qne'))';%归一化

end

