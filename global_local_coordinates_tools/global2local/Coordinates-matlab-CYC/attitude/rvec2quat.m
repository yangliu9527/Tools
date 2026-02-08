% 类型：函数
% 功能：等效旋转向量转四元数
% out：q 四元数
% in：rvec 等效旋转矢量,其大小表示单位为弧度的角度
% 说明：求取的四元数为qms，即m描述动坐标系，s描述静坐标系
function q = rvec2quat( rvec )
    nm2 = rvec'*rvec;  
    if nm2<1.0e-8  
        q0 = 1-nm2*(1/8-nm2/384); s = 1/2-nm2*(1/48-nm2/3840);%取傅里叶级数展开的前3项
    else
        nm = sqrt(nm2);
        q0 = cos(nm/2); s = sin(nm/2)/nm;
    end
    q = [q0; s*rvec];
end

