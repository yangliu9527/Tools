function result = euler_n2w(blh0,blh,euler0)
%EULER_N2W 转换欧拉角，b系相对于n系转换为b系相对于w系
%blh0 初始纬经高
%blh  当前纬经高
%euler0 待转换的欧拉角
%按行操作,blh和euler0的行数要对应
%角度
line=size(blh,1);
assert(size(euler0,1)==line);
result=zeros(line,3);
qne0=pos2quat(blh0);
qen0=qinv(qne0);
for i=1:line

    qbn=euler2quat(euler0(i,:)');
    qne=pos2quat(blh(i,:));
    qtmp=qmult(qen0,qne);
    q=qmult(qtmp,qbn);

    result(i,:)=quat2euler(q)';
end

end

