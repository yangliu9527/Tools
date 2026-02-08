import numpy as np
import math

class Coordinate:
    def __init__(self):
        # 地球参数
        self.D2R = math.pi / 180  # 角度转弧度
        self.a = 6378137.0  # 地球长半轴 (m)
        self.b = 6356752.314245179  # 地球短半轴 (m)
        self.f = 1 / 298.257223563  # 地球扁率

        self.e = math.sqrt(2 * self.f - self.f**2)  # 第一偏心率
        self.e2 = self.e * self.e  # 第一偏心率的平方

        self.e_nd = math.sqrt(self.a**2 - self.b**2) / self.b  # 第二偏心率
        self.e_nd2 = self.e_nd * self.e_nd  # 第二偏心率的平方

        self.GM = 3.986004418e14  # 地球引力常数 (m^3/s^2)
        self.wie = 7.2921151467e-5  # 地球自转角速度 (rad/s)
        self.grav_equ = 9.7803267715  # 赤道处重力值 (m/s^2)
        self.grav_pol = 9.8321863685  # 极点处重力值 (m/s^2)
        
        # 简洁计算重力参数
        self.cul_grav_para = np.array([
            9.7803267715, 0.0052790414, 0.0000232718, -0.000003087691089,
            0.000000004397731, 0.000000000000721
        ])

        # NED to ENU matrix
        self.R_ned2enu = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])

    def getRM(self, blh):
        """
        获取RM，blh的输入为纬经高，角度
        支持列向量操作，即3*n的向量，每一列为一个值，返回行向量
        """
        # 确保输入的 blh 是 3xn 的矩阵
        assert blh.shape[0] == 3
        
        # 将纬度从度转换为弧度
        lat = blh[0, :] * self.D2R
        
        # 计算 k 和 RM
        k = 1 - self.e2 * np.sin(lat)**2
        RM = self.a * (1 - self.e2) / np.sqrt(k) / k
        
        return RM

    def getRN(self, blh):
        """
        获取RN，blh的输入为纬经高，角度
        支持列向量操作，即3*n的向量，每一列为一个值，返回行向量
        """
        # 确保输入的 blh 是 3xn 的矩阵
        assert blh.shape[0] == 3
        
        # 将纬度从度转换为弧度
        lat = blh[0, :] * self.D2R
        
        # 计算 RN
        RN = self.a / np.sqrt(1 - self.e2 * np.sin(lat)**2)
        
        return RN

    def blh2ecef(self, blh):
        """
        获取ECEF坐标系的坐标，blh的输入为纬经高，角度
        支持列向量操作，即3*n的向量，每一列为一个纬经高
        """
        # 确保输入的 blh 是 3xn 的矩阵
        assert blh.shape[0] == 3
        
        # 获取 RN（法向半径）
        RN = self.getRN(blh)
        
        # 纬度、经度、高度
        B = blh[0, :] * self.D2R  # 纬度转换为弧度
        L = blh[1, :] * self.D2R  # 经度转换为弧度
        H = blh[2, :]  # 高度
        
        # 计算 ECEF 坐标
        x = (RN + H) * np.cos(B) * np.cos(L)
        y = (RN + H) * np.cos(B) * np.sin(L)
        z = (RN * (1 - self.e2) + H) * np.sin(B)
        
        # 返回 ECEF 坐标
        ecef = np.vstack((x, y, z))
        
        return ecef

    def blh2ned(self, blh0, blh):
        """
        将经纬高坐标位置转换为以某个点建立的NED系为本地坐标系之中
        获取坐标系的坐标，blh的输入为纬经高，角度
        支持列向量操作，即3*n的向量，每一列为一个纬经高
        """
        # 确保输入的 blh 是 3xn 的矩阵
        assert blh.shape[0] == 3
        
        len = blh.shape[1]
        
        # 获取参考点的 ECEF 坐标
        rXYZ0 = np.tile(self.blh2ecef(blh0), (1, len))
        
        # 计算参考点的四元数
        qne0 = self.pos2quat(blh0)
        qen0 = self.qinv(qne0)
        
        # 初始化 local 为零
        local = np.zeros((3, len))
        
        # 获取目标点的 ECEF 坐标
        rXYZt = self.blh2ecef(blh)
        
        # 计算参考点和目标点的 ECEF 坐标差值
        del_rXYZ = rXYZt - rXYZ0
        
        # 使用四元数旋转坐标差值
        for i in range(len):
            local[:, i] = np.squeeze(self.qmultvec(qen0, del_rXYZ[:, i]))
        
        return local

    def getG(self, blh):
        """
        获取重力，blh的输入为纬经高，角度
        支持列向量操作，即3*n的向量，每一列为一个纬经高
        """
        # 确保输入的 blh 是 3xn 的矩阵
        assert blh.shape[0] == 3
        
        # 获取纬度和高度
        B = blh[0, :] * self.D2R  # 纬度转换为弧度
        h = blh[2, :]  # 高度
        
        # 计算必要的中间量
        si2 = np.sin(B)**2
        co2 = np.cos(B)**2
        
        # 计算 m、k 和 gamma
        m = self.wie**2 * self.a**2 * self.b / self.GM
        k = 2 * h * (1 + self.f + m - 2 * self.f * si2) / self.a
        gamma = self.a * self.grav_equ * co2 + self.b * self.grav_pol * si2
        gamma = gamma / np.sqrt(self.a**2 * co2 + self.b**2 * si2)
        
        # 计算重力加速度
        tmpgh = gamma * (1 - k + 3 * h**2 / (self.a**2))
        
        # 返回重力加速度（NED坐标系的z轴分量）
        g = np.zeros((3, len(tmpgh)))
        g[2, :] = tmpgh
        
        return g

    def ecef2blh(self, ecef):
        """
        通过ecef坐标获取blh坐标系的坐标，blh的输出为纬经高，角度
        列向量表示
        """
        # 确保输入的 ecef 是 3xn 的矩阵
        assert ecef.shape[0] == 3
        
        # 获取 ECEF 坐标的分量
        X = ecef[0, :]
        Y = ecef[1, :]
        Z = ecef[2, :]
        
        # 计算 s 和 theta
        s = np.sqrt(X**2 + Y**2)
        theta = np.arctan2(Z * self.a, s * self.b)
        
        # 计算立方的 sin 和 cos
        s3 = np.sin(theta)**3
        c3 = np.cos(theta)**3
        
        # 计算经纬高（BLH）坐标
        rBLH = np.zeros_like(ecef)
        rBLH[0, :] = np.arctan2(Z + self.e_nd2 * self.b * s3, s - self.e2 * self.a * c3)
        rBLH[1, :] = np.arctan2(Y, X)
        rBLH[2, :] = s / np.cos(rBLH[0, :]) - self.a / np.sqrt(1 - self.e2 * np.sin(rBLH[0, :])**2)
        
        # 将纬度和经度从弧度转换为角度
        rBLH[0:2, :] = rBLH[0:2, :] * (180 / np.pi)
        
        return rBLH

    def getCne(self, blh):
        """
        获得 C_n^e，即由经纬度 (blh) 计算得到的方向余弦矩阵 C_n^e
        输入：blh 是经纬高坐标，单位为角度
        输出：返回一个 3x3 的方向余弦矩阵 C_n^e
        """
        # 将角度转换为弧度
        D2R = np.pi / 180
        phi = blh[0] * D2R  # 纬度转换为弧度
        lambda_ = blh[1] * D2R  # 经度转换为弧度
        
        # 计算方向余弦矩阵 C_n^e
        Cne = np.array([
            [-np.sin(phi) * np.cos(lambda_), -np.sin(lambda_), -np.cos(phi) * np.cos(lambda_)],
            [-np.sin(phi) * np.sin(lambda_), np.cos(lambda_), -np.cos(phi) * np.sin(lambda_)],
            [np.cos(phi), 0, -np.sin(phi)]
        ])
        
        return Cne

    def ecef2ned(self, ecef, blh0):
        """
        将ECEF坐标转换为以某个点（blh0）为原点的NED坐标
        输入：ecef 为地心地固坐标系（ECEF）的坐标
               blh0 为参考点的经纬高坐标
        输出：ned 为以 blh0 为原点的NED坐标
        """
        # 1. 将 ECEF 转换为 BLH 坐标
        blh = self.ecef2blh(ecef)
        
        # 2. 将 BLH 坐标转换为 NED 坐标
        ned = self.blh2ned(blh0, blh)
        
        return ned

    def euler_n2w(self, blh0, blh, euler0):
        """
        将 B 系相对于 N 系的欧拉角转换为 B 系相对于 W 系的欧拉角。
        输入：
        - blh0: 初始参考点的经纬高坐标
        - blh: 当前点的经纬高坐标
        - euler0: 待转换的欧拉角，按行操作，blh 和 euler0 的行数要对应
        输出：
        - result: 转换后的欧拉角
        """
        # 确保输入的行数一致
        line = blh.shape[0]
        assert euler0.shape[0] == line
        
        # 初始化结果矩阵
        result = np.zeros((line, 3))
        
        # 计算参考点的四元数
        qne0 = self.pos2quat(blh0)
        qen0 = self.qinv(qne0)
        
        # 遍历每一行，进行四元数和欧拉角转换
        for i in range(line):
            # 将欧拉角转换为四元数
            qbn = self.euler2quat(euler0[i, :])
            qne = self.pos2quat(blh[i, :])
            
            # 计算临时四元数
            qtmp = self.qmult(qen0, qne)
            q = self.qmult(qtmp, qbn)
            
            # 将四元数转换回欧拉角
            result[i, :] = self.quat2euler(q)
        
        return result
    
    def euler2matrix(self, euler):
        """
        将欧拉角转换为旋转矩阵
        输入：euler 为欧拉角，单位为度，包含 3 个角度值（滚转、俯仰、偏航）
        输出：Cbn 为旋转矩阵
        """
        # 将欧拉角从度转换为弧度
        euler = np.radians(euler)
        
        # 确保输入是一个 3x1 的数组
        if euler.shape[0] == 3:
            s_phi = np.sin(euler[0])
            c_phi = np.cos(euler[0])
            s_theta = np.sin(euler[1])
            c_theta = np.cos(euler[1])
            s_psi = np.sin(euler[2])
            c_psi = np.cos(euler[2])

            # 计算旋转矩阵 Cbn
            Cbn = np.array([[c_psi * c_theta, (-s_psi * c_phi) + c_psi * (s_theta * s_phi), (s_psi * s_phi) + c_psi * (s_theta * c_phi)],
                            [s_psi * c_theta, (c_psi * c_phi) + s_psi * (s_theta * s_phi), (-c_psi * s_phi) + s_psi * (s_theta * c_phi)],
                            [-s_theta, c_theta * s_phi, c_theta * c_phi]])
        else:
            print("Error: Euler angles must be a 3x1 vector.")
            Cbn = None
        
        return Cbn

    def euler2quat(self, euler):
        """
        将欧拉角转换为四元数
        输入：euler 为欧拉角（滚转、俯仰、偏航），单位为度
        输出：q 为四元数（w, x, y, z）
        """
        # 将欧拉角从度转换为弧度
        euler = np.radians(euler)

        # 确保输入是一个 3x1 的数组
        if euler.shape[0] == 3:
            roll = euler[0]  # 滚转角
            pitch = euler[1]  # 俯仰角
            yaw = euler[2]  # 偏航角

            # 计算四元数分量
            sr = np.sin(roll / 2)
            sp = np.sin(pitch / 2)
            sy = np.sin(yaw / 2)
            cr = np.cos(roll / 2)
            cp = np.cos(pitch / 2)
            cy = np.cos(yaw / 2)

            # 四元数计算公式
            q = np.zeros(4)
            q[0] = cr * cp * cy + sr * sp * sy  # w
            q[1] = sr * cp * cy - cr * sp * sy  # x
            q[2] = cr * sp * cy + sr * cp * sy  # y
            q[3] = cr * cp * sy - sr * sp * cy  # z

            # 归一化四元数
            q = q / np.linalg.norm(q)
        else:
            print("Error: Euler angles must be a 3x1 vector.")
            q = None
        
        return q
    def invbc(self,M):
        """
        An algorithm for matrix inversion to avoid 'bad condition number' warning.
        
        Input:
            M - input square matrix
        
        Output:
            invM - the inversion of input M
        """
        # 计算对角元素的平方根
        D = np.sqrt(np.diag(M))
        
        # 计算 K 矩阵
        K = np.max(D) / D
        K = np.diag(K)
        
        # 计算逆矩阵
        invM = np.linalg.inv(K @ M @ K) @ K
        
        return invM
    
    def matrix2euler(self,R):
        """
        将旋转矩阵 R 转换为欧拉角
        输入：R - 旋转矩阵（3x3）
        输出：euler - 欧拉角（单位为度）
        """
        # NED 坐标系
        euler = np.zeros(3)
        euler[0] = np.arctan2(R[2, 1], R[2, 2])  # 滚转角
        euler[1] = np.arctan(-R[2, 0] / np.sqrt(R[2, 1]**2 + R[2, 2]**2))  # 俯仰角
        euler[2] = np.arctan2(R[1, 0], R[0, 0])  # 偏航角
        
        # 将弧度转换为度
        euler = euler * 180 / np.pi
        
        # 确保偏航角在0到360度之间
        if euler[2] < 0:
            euler[2] = euler[2] + 360
        
        return euler

    def pos2quat(self,BLH):
        """
        将地理坐标（BLH）转换为四元数（qne）
        
        输入：
            BLH - 经纬度坐标（纬度， 经度， 高度）
        
        输出：
            qne - 旋转四元数（n系到e系的转换）
        """
        D2R = np.pi / 180
        Phi = BLH[0] * D2R  # 纬度
        Lambda = BLH[1] * D2R  # 经度
        
        # 计算四元数的各个分量
        cL = np.cos(Lambda / 2)  # cos(λ/2)
        sL = np.sin(Lambda / 2)  # sin(λ/2)
        cB = np.cos(-np.pi / 4 - Phi / 2)  # cos(-π/4 - φ/2)
        sB = np.sin(-np.pi / 4 - Phi / 2)  # sin(-π/4 - φ/2)
        
        qne = np.array([cB * cL, -sB * sL, sB * cL, cB * sL])
        
        # 归一化四元数
        qne = qne / np.linalg.norm(qne)
        
        return qne
    
    def qinv(self,q1):
        """
        计算四元数的逆（共轭四元数）
        
        输入：
            q1 - 输入四元数，形状为 [w, x, y, z]
        
        输出：
            q - 四元数的逆（共轭四元数）
        """
        q = np.copy(q1)
        q[1:4] = -q[1:4]  # 取负值，计算共轭四元数
        return q
    
    def qmult(self,q1, q2):
        """
        计算两个四元数的乘积
        
        输入：
            q1 - 四元数1，形状为 [w1, x1, y1, z1]
            q2 - 四元数2，形状为 [w2, x2, y2, z2]
        
        输出：
            q - 四元数的乘积，形状为 [w, x, y, z]
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        # 计算四元数的乘积
        q = np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2,
            w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        ])
        
        # 四元数标准化
        q = q / np.linalg.norm(q)
        
        return q
    
    def qmultvec(self,q, v1):
        """
        使用四元数对向量进行旋转
        
        输入：
            q - 旋转四元数，形状为 [w, x, y, z]
            v1 - 向量，形状为 [x, y, z]
        
        输出：
            v - 旋转后的向量，形状为 [x, y, z]
        """
        # 计算 v1 的模长
        tmp = np.linalg.norm(v1)
        
        if tmp != 0:
            # 将向量 v1 表示为四元数 [0; v1_x; v1_y; v1_z]
            q1 = np.concatenate(([0], v1))
            
            # 计算旋转后的四元数 q2 = q * q1 * q^-1
            q_inv = self.qinv(q)  # 计算 q 的逆
            q2 = self.qmult(self.qmult(q, q1), q_inv)
            
            # 返回旋转后的向量 v2，缩放回原来的模长
            v = q2[1:4] * tmp
        else:
            v = np.zeros(3)  # 如果向量长度为0，返回零向量
        
        return v
    
    def quat2euler(self,q):
        """
        将四元数转换为欧拉角
        输入：q - 四元数 [w, x, y, z]
        输出：euler - 欧拉角 [滚转, 俯仰, 偏航] 单位：度
        """
        euler = np.zeros(3)
        
        if q.shape[0] == 4:
            # 弧度到角度的转换因子
            R2D = 180 / np.pi
            
            # 四元数的分量
            q11 = q[0] * q[0]
            q12 = q[0] * q[1]
            q13 = q[0] * q[2]
            q14 = q[0] * q[3]
            
            q22 = q[1] * q[1]
            q23 = q[1] * q[2]
            q24 = q[1] * q[3]
            
            q33 = q[2] * q[2]
            q34 = q[2] * q[3]
            
            q44 = q[3] * q[3]
            
            # 计算欧拉角
            euler[0] = np.arctan2(2 * (q12 + q34), q11 - q22 - q33 + q44)  # 滚转角 (Roll)
            euler[1] = np.arcsin(2 * (q13 - q24))                           # 俯仰角 (Pitch)
            euler[2] = np.arctan2(2 * (q14 + q23), q11 + q22 - q33 - q44)  # 偏航角 (Yaw)
            
            # 转换为角度
            euler = euler * R2D
            
            # 偏航角调整为 [0, 360] 范围
            if euler[2] < 0:
                euler[2] += 360
        else:
            print("输入四元数的维度不正确")
        
        return euler
    
    def quat2matrix(self,q):
        """
        将四元数转换为旋转矩阵
        输入：q - 四元数 [w, x, y, z]
        输出：Cbn - 旋转矩阵
        """
        if q.shape[0] == 4:
            q2 = q * q  # 四元数的平方

            Cbn = np.array([
                [q2[0] + q2[1] - q2[2] - q2[3], 2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[1] * q[3] + 2 * q[0] * q[2]],
                [2 * q[1] * q[2] + 2 * q[0] * q[3], q2[0] - q2[1] + q2[2] - q2[3], 2 * q[2] * q[3] - 2 * q[0] * q[1]],
                [2 * q[1] * q[3] - 2 * q[0] * q[2], 2 * q[2] * q[3] + 2 * q[0] * q[1], q2[0] - q2[1] - q2[2] + q2[3]]
            ])
        else:
            print("输入四元数的维度不正确")
        
        return Cbn
    

    def rvec2quat(self,rvec):
        """
        将旋转向量转换为四元数
        输入：rvec - 旋转向量 [rx, ry, rz]
        输出：q - 四元数 [w, x, y, z]
        """
        nm2 = np.dot(rvec, rvec)  # rvec的平方模
        
        if nm2 < 1.0e-8:
            q0 = 1 - nm2 * (1 / 8 - nm2 / 384)
            s = 1 / 2 - nm2 * (1 / 48 - nm2 / 3840)
        else:
            nm = np.sqrt(nm2)
            q0 = np.cos(nm / 2)
            s = np.sin(nm / 2) / nm
        
        q = np.concatenate(([q0], s * rvec))  # 四元数 [w, x, y, z]
        
        return q