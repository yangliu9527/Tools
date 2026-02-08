% 类型：函数
% 功能：欧拉角转方向余弦矩阵
% in：euler 角度
% out：Cbn 方向余弦矩阵
function Cbn = euler2matrix( euler )
euler=euler*pi/180;
if size(euler,1) == 3
    s_phi=sin(euler(1,1));
    c_phi=cos(euler(1,1));
    s_theta=sin(euler(2,1));
    c_theta=cos(euler(2,1));
    s_psi=sin(euler(3,1));
    c_psi=cos(euler(3,1));
    
    Cbn = [c_psi*c_theta, (-s_psi*c_phi)+c_psi *(s_theta*s_phi), (s_psi*s_phi)+c_psi*(s_theta*c_phi);
           s_psi*c_theta, (c_psi*c_phi)+s_psi*(s_theta*s_phi), (-c_psi*s_phi)+s_psi*(s_theta*c_phi);
           -s_theta, c_theta*s_phi, c_theta * c_phi;];
else
    fprintf('请输入列向量表示的欧拉角！');
end 
end

