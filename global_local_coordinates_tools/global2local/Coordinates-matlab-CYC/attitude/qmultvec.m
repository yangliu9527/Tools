% 类型：函数
% 功能：实现四元数直接和3D向量乘法，即通过使用四元数直接利用乘法对3D向量进行旋转
% out：v 旋转后向量
% in：q/v1 四元数/旋转前列向量

function v = qmultvec( q,v1 )    
    tmp=sqrt(v1'*v1);%因为在进行四元数运算结束后，都归一化了，所以先将向量模长保存
    if tmp~=0   
        q1 = [0;v1];
        q2 = qmult(qmult(q,q1),qinv(q));
        v = q2(2:4,1).*tmp;
    else 
        v=[0; 0; 0];
    end
end

