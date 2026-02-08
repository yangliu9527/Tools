% 类型：函数
% 功能：欧拉角转四元数
% out：q 四元数
% in：euler 欧拉角(unit:deg)

function q = euler2quat(euler)
% if  ~(euler(1)>=-pi/2&&euler(1)<=pi/2 || euler(2)>=-pi/2&&euler(2)<=pi/2)
%     error('函数euler2quat中欧拉角输入超出范围输入有误:-pi<=roll<=pi、-pi/2<pitch<pi/2');
% end
% if euler(3)>=-pi&&euler(1)<0
%     tmp=euler(3);
%     euler(3)=tmp+360;%转换到0-360度内
% end
    euler=euler*pi/180;
if size(euler,1) == 3
    roll = euler(1); 
    pitch = euler(2);
    yaw = euler(3);  
    
    sr = sin(roll/2); sp = sin(pitch/2);  sy = sin(yaw/2);
    cr = cos(roll/2); cp = cos(pitch/2); cy = cos(yaw/2);
    
    q(1, 1) = cr*cp*cy + sr*sp*sy;%w
    q(2, 1) = sr*cp*cy - cr*sp*sy;%x
    q(3, 1) = cr*sp*cy + sr*cp*sy;%y
    q(4, 1) = cr*cp*sy - sr*sp*cy;%z
    
    q = (quatnormalize(q'))';%归一化
else
    fprintf('请输入列向量表示的欧拉角！');
end 
end