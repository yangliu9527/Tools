% 类型：函数
% 功能：四元数转欧拉角
% out：euler 欧拉角(unit:deg)
% in：q 四元数

function euler = quat2euler(q)
euler=zeros(3,1);
if size(q,1) == 4
    R2D=180/pi;
    
    q11 = q(1,1)*q(1,1); q12 = q(1,1)*q(2,1); q13 = q(1,1)*q(3,1); q14 = q(1,1)*q(4,1); 
	q22 = q(2,1)*q(2,1); q23 = q(2,1)*q(3,1); q24 = q(2,1)*q(4,1);     
	q33 = q(3,1)*q(3,1); q34 = q(3,1)*q(4,1);  
	q44 = q(4,1)*q(4,1);
    
	euler(1,1) = atan2(2*(q12+q34), q11-q22-q33+q44);
	euler(2,1) = asin(2*(q13-q24));
	euler(3,1) = atan2(2*(q14+q23), q11+q22-q33-q44); 
    
    euler = euler*R2D;
    
    if euler(3,1)<0
        euler(3,1)=euler(3,1)+360;%将航向角设置到0-360度以内
    end
    
else
    fprintf('请输入列向量表示的四元数！');
end 
end

