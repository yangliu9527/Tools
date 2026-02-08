function Cne=getCne(obj,blh)%获得C_n^e
%角度
D2R=pi/180;
phi=blh(1)*D2R;
lambda=blh(2)*D2R;


Cne=[-sin(phi)*cos(lambda),-sin(lambda),-cos(phi)*cos(lambda);
    -sin(phi)*sin(lambda),cos(lambda),-cos(phi)*sin(lambda);
    cos(phi),0,-sin(phi)];
end

