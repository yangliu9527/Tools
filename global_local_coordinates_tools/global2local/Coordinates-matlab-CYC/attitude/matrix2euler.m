% 类型：函数
% 功能：方向余弦矩阵转欧拉角
% out：euler 欧拉角（unit：deg）
% in：R 方向余弦矩阵

function euler = matrix2euler( R )
    %NED
    euler(1,1) = atan2(R(3,2),R(3,3));	
	euler(2,1) = atan(-R(3,1)/sqrt(R(3,2) * R(3,2) + R(3,3) * R(3,3)));		
	euler(3,1) = atan2(R(2,1), R(1,1));
    euler = euler*180/pi;
    if(euler(3,1)<0)
        euler(3,1)=euler(3,1)+360;
    end
    %ENU
%     euler(1,1) = atan2(-R(3,1),R(3,3));
%     euler(2,1) = asin(R(3,2));
%     euler(3,1) = atan2(-R(1,2), R(2,2));
%     euler = euler*180/pi;
end

