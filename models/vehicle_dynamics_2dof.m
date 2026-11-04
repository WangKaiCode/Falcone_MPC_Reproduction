% =========================================================
% Function: vehicle_dynamics_2dof
%
% Purpose:
%   Calculate the lateral and yaw dynamics of a vehicle
%   using the linear 2-DOF bicycle model.
%
% Reference:
%   Rajamani, Vehicle Dynamics and Control, 2nd Edition
%   Section 2.3
%
% Inputs:
%   state      - Current vehicle states [vy; r]
%   delta_f    - Front wheel steering angle (rad)
%   p          - Vehicle parameter structure
%
% Outputs:
%   dstate     - State derivatives [dvy; dr]
% =========================================================
%
% 【中文说明】
%   本函数实现线性 2 自由度（2-DOF）自行车模型，用于描述车辆
%   在水平面内的横向（侧向）与横摆动力学。它是 Falcone MPC 复现
%   中最基础、最常用的预测模型，后续非线性模型、Pacejka 轮胎模型
%   都会以它为参照。
%
%   状态量 state = [vy; r]：
%     vy —— 车辆质心处的横向速度（m/s）
%     r  —— 车辆横摆角速度（yaw rate, rad/s）
%   输入量 delta_f：
%     前轮转向角（rad），由驾驶员或控制器给出
%   输出量 dstate = [dvy; dr]：
%     dvy —— 横向加速度（m/s^2）
%     dr  —— 横摆角加速度（rad/s^2）
%
%   建模假设：
%     1) 纵向速度 vx 恒定（匀速假设），无纵向动力学；
%     2) 左右车轮对称，合并为前、后各一个等效轮（自行车模型）；
%     3) 轮胎侧偏力与侧偏角成线性关系（小侧偏角）；
%     4) 忽略空气阻力、载荷转移、悬架等影响。
% =========================================================

% ---------------------------------------------------------
% Step 1: Extract current vehicle states
%   Get lateral velocity vy
%   Get yaw rate r
% ---------------------------------------------------------
% 【Step 1：提取当前车辆状态】
%   从状态向量 state 中拆出两个状态量：
%     vy —— 横向速度；
%     r  —— 横摆角速度。
%   写法示意：vy = state(1);  r = state(2);

% ---------------------------------------------------------
% Step 2: Extract vehicle parameters
%   Get mass m
%   Get yaw moment of inertia Iz
%   Get front axle distance lf
%   Get rear axle distance lr
%   Get front cornering stiffness Caf
%   Get rear cornering stiffness Car
%   Get longitudinal velocity vx
% ---------------------------------------------------------
% 【Step 2：提取车辆参数】
%   从参数结构体 p 中读取建模所需参数：
%     m   —— 整车质量（kg）
%     Iz  —— 绕质心 z 轴的横摆转动惯量（kg·m^2）
%     lf  —— 质心到前轴的距离（m）
%     lr  —— 质心到后轴的距离（m）
%     Caf —— 前轴等效侧偏刚度（N/rad）
%     Car —— 后轴等效侧偏刚度（N/rad）
%     vx  —— 纵向车速（m/s），此处按匀速处理
%   这些参数由 parameters/vehicle_parameters.m 统一给出。

% ---------------------------------------------------------
% Step 3: Calculate front and rear wheel velocity angles
%   Calculate front wheel velocity angle theta_vf
%   Calculate rear wheel velocity angle theta_vr
%   Use the small-angle approximation
% ---------------------------------------------------------
% 【Step 3：计算前、后轮的速度方向角】
%   速度方向角（velocity angle）表示该轮所在位置的速度矢量
%   相对车身纵轴的方向，用于后续计算轮胎侧偏角。
%
%   基于运动学关系（小角度近似）：
%     theta_vf ≈ vy/vx + lf*r/vx   （前轮速度方向角）
%     theta_vr ≈ vy/vx - lr*r/vx   （后轮速度方向角）
%   物理含义：
%     vy/vx   —— 车身整体的横向漂移角（侧偏角贡献）；
%     lf*r/vx —— 因前轴距质心更远，横摆带来的附加横向速度；
%     lr*r/vx —— 后轴同理，但方向相反。
%   小角度近似避免了 arctan 运算，使模型保持线性，便于 MPC 使用。

% ---------------------------------------------------------
% Step 4: Calculate front and rear tire slip angles
%   alpha_f = front steering angle - front velocity angle
%   alpha_r = - rear velocity angle
% ---------------------------------------------------------
% 【Step 4：计算前、后轮胎侧偏角】
%   轮胎侧偏角（slip angle）是轮胎实际运动方向与轮胎朝向
%   （即车轮平面方向）之间的夹角，是产生侧偏力的直接原因。
%
%   前轮侧偏角：
%     alpha_f = delta_f - theta_vf
%     —— 前轮朝向由转向角 delta_f 决定，减去前轮速度方向角
%   后轮侧偏角：
%     alpha_r = - theta_vr
%     —— 后轮不转向，朝向与车身纵轴一致，因此只有速度方向角
%   符号约定：
%     侧偏角为正表示轮胎受力方向与正横向方向一致，
%     与 Rajamani 教材一致，便于直接套用线性轮胎公式。

% ---------------------------------------------------------
% Step 5: Calculate lateral tire forces
%   Calculate front axle lateral force Fyf
%   Calculate rear axle lateral force Fyr
%   Use the linear tire cornering stiffness model
% ---------------------------------------------------------
% 【Step 5：计算前、后轴横向轮胎力】
%   采用线性轮胎模型（linear cornering stiffness model）：
%     Fyf = Caf * alpha_f
%     Fyr = Car * alpha_r
%   物理含义：
%     侧偏力与侧偏角成正比，比例系数即侧偏刚度 Caf / Car。
%   适用条件：
%     小侧偏角（一般 |alpha| < 3°~5°），此时轮胎处于线性区。
%   后续扩展：
%     若替换为 Pacejka 魔术公式（models/tire_model_pacejka.m），
%     只需把这两行换成非线性映射 Fy = f(alpha)，其余结构不变。

% ---------------------------------------------------------
% Step 6: Calculate lateral acceleration
%   Apply Newton's second law in the lateral direction
%   Calculate dvy
% ---------------------------------------------------------
% 【Step 6：计算横向加速度 dvy】
%   对车辆质心沿车身横向（y 方向）应用牛顿第二定律：
%     m * (dvy + vx * r) = Fyf + Fyr
%   其中：
%     vx * r —— 因横摆引起的横向惯性加速度（向心项）；
%     Fyf + Fyr —— 前、后轴横向轮胎力的合力。
%   整理得：
%     dvy = (Fyf + Fyr) / m - vx * r
%   说明：
%     若忽略 vx*r 项，则退化为简单的 "质量×加速度=力"，
%     但在车辆模型中此项不可忽略，它体现了旋转参考系的效应。

% ---------------------------------------------------------
% Step 7: Calculate yaw angular acceleration
%   Apply moment balance around the vehicle CG
%   Calculate dr
% ---------------------------------------------------------
% 【Step 7：计算横摆角加速度 dr】
%   对车辆绕质心 z 轴做力矩平衡：
%     Iz * dr = lf * Fyf - lr * Fyr
%   整理得：
%     dr = (lf * Fyf - lr * Fyr) / Iz
%   物理含义：
%     前轮侧偏力产生绕质心的正力矩（lf 力臂），
%     后轮侧偏力产生负力矩（lr 力臂，方向相反），
%     两者之差除以横摆转动惯量即得横摆角加速度。
%   直观理解：
%     前轮抓地强、后轮抓地弱 → dr 偏大 → 转向过度趋势；
%     反之则转向不足趋势。

% ---------------------------------------------------------
% Step 8: Assemble the state derivatives
%   Return [dvy; dr]
% ---------------------------------------------------------
% 【Step 8：组装并返回状态导数】
%   将 Step 6 与 Step 7 的结果组合为列向量：
%     dstate = [dvy; dr]
%   该向量即 2-DOF 模型的连续时间状态方程右端项：
%     d/dt [vy; r] = f([vy; r], delta_f; p)
%   在仿真或 MPC 中，通常再用数值积分（如 ode45、RK4）
%   或离散化（如零阶保持、前向欧拉）推进状态：
%     state(k+1) = state(k) + Ts * dstate(k)