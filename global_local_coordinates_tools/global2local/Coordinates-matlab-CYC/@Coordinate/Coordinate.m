classdef Coordinate< handle &matlab.mixin.Copyable
    %COORDINATE 用于坐标转化
    %所用纬经高表示的，纬度和经度的输入输出都用角度！！！

    properties
        %具体含义看构造函数
        D2R=pi/180;

        a
        b
        f

        e
        e2

        e_nd
        e_nd2

        GM
        wie
        grav_equ
        grav_pol
        cul_grav_para

        R_ned2enu=[0 1 0;
            1 0 0
            0 0 -1];%ned转enu
    end

    methods
        %构造函数
        function obj = Coordinate()
            obj.a=6378137.0;                    %m  地球长半轴Rp
            obj.b=6356752.314245179;            %m  地球短半轴Re
            obj.f=1/298.257223563;              %地球扁率

            obj.e=sqrt(2*obj.f-obj.f^2);          %第一偏心率
            obj.e2=obj.e*obj.e;                   %第一偏心率的平方
            %obj.e2=0.00669437999013;
            obj.e_nd=sqrt(obj.a^2-obj.b^2)/obj.b;  %第二偏心率
            obj.e_nd2=obj.e_nd*obj.e_nd;          %第二偏心率的平方
            %obj.e_nd2=0.006739496742227;
            obj.GM=3.986004418e14;              %m^3/s^2 地球引力常数
            %     obj.wie=7.292115e-5;                %rad/s  地球自转角速度
            obj.wie=7.2921151467e-5;
            obj.grav_equ=9.7803267715;          %赤道处重力值
            obj.grav_pol=9.8321863685;          %极点处重力值
            obj.cul_grav_para = [9.7803267715, 0.0052790414, 0.0000232718, -0.000003087691089,...
                0.000000004397731, 0.000000000000721]; %简洁计算重力参数
        end

        RM=getRM(obj,blh);%本代码涉及经纬度和欧拉角的，输入输出参数全部为角度
        RN=getRN(obj,blh);
        ecef=blh2ecef(obj,blh);
        local=blh2ned(obj,blh0,blh);
        g=getG(obj,blh);
        blh=ecef2blh(obj,ecef);
        Cne=getCne(obj,blh);%获得C_n^e
        ned=ecef2ned(obj,ecef,blh0);

    end
end

