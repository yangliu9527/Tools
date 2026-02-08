% 类型：函数
% 功能：四元数求逆
% out：q 四元数
% in：q1 四元数

function q = qinv( q1 )
    q(1,1) = q1(1,1);
    q(2:4,1) = -q1(2:4,1); %共轭
end

