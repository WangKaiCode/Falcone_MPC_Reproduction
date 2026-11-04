# Falcone MPC Reproduction

自动驾驶车辆主动转向与模型预测控制（MPC）文献复现项目。本仓库用于学习车辆动力学建模、路径/轨迹跟踪和
MPC，并逐步复现 Falcone 等人的相关研究。

> **当前状态：** 项目骨架已建立；`vehicle_dynamics_2dof.m`
> 目前为注释与伪代码框架。车辆模型实现、参数配置、测试和论文控制器仍在开发中。本仓库尚不代表已完成
> 2005 或 2007 年论文的完整复现。

## 1. 研究目标

核心参考论文：

-   Falcone et al. (2005), *MPC-based approach to active steering for
    autonomous vehicle systems*.
-   Falcone et al. (2007), *Predictive Active Steering Control for
    Autonomous Vehicle Systems*.

研究任务：

1.  建立并验证车辆动力学模型；
2.  理解轮胎侧偏角、轮胎侧向力及非线性轮胎模型；
3.  实现可重复运行的车辆仿真与轨迹跟踪实验；
4.  按原论文逐步实现 MPC 控制器；
5.  对照论文核验模型、参数、约束、目标函数和实验工况；
6.  记录复现差异、实现假设、实验结果与未解决问题。

**复现原则：**
教材模型用于学习和程序验证，不自动等同于论文模型。最终实现以原论文明确给出的公式、参数和实验设置为依据；论文未明确说明的内容应标注为实现假设。

## 2. 开发路线

  --------------------------------------------------------------------------------------------
  阶段              工作内容          交付结果                               状态
  ----------------- ----------------- -------------------------------------- -----------------
  0                 MATLAB            MATLAB Project、目录、版本管理         进行中
                    工程与代码结构                                           

  1                 Rajamani 教材     动力学函数、参数、基本测试与响应曲线   进行中
                    2-DOF                                                    
                    线性自行车模型                                           

  2                 2005 论文车辆模型 按原文实现并验证车辆动力学模型         待开始

  3                 2005 论文 MPC     目标函数、约束、优化问题与闭环仿真     待开始

  4                 2007 论文改进方法 对照原文实现并验证                     待开始

  5                 复现对比与总结    工况对比、误差分析、复现报告           待开始
  --------------------------------------------------------------------------------------------

## 3. 目录结构

``` text
Falcone_MPC_Reproduction/
├── Falcone_MPC_Reproduction.prj
├── README.md
├── .gitignore
├── .gitattributes
├── models/
│   ├── vehicle_dynamics_2dof.m
│   ├── vehicle_dynamics.m
│   └── tire_model_pacejka.m
├── parameters/
│   └── vehicle_parameters.m
├── controllers/
├── simulations/
├── tests/
│   └── test_2dof_model.m
└── results/
```

-   `models/`：车辆与轮胎动力学函数，不混入绘图和完整仿真流程。
-   `parameters/`：集中管理车辆、轮胎及仿真参数。
-   `controllers/`：控制算法实现。
-   `simulations/`：可重复运行的实验入口与工况。
-   `tests/`：数学关系、边界条件和数值行为测试。
-   `results/`：有价值的实验结果与图表；临时结果不必全部提交。

## 4. 软件环境

当前开发环境：Windows 11、MATLAB R2024b (24.2)、Simulink R2024b (24.2)。

已安装的相关工具箱包括 Control System Toolbox、Optimization
Toolbox、Model Predictive Control Toolbox、Automated Driving Toolbox 和
Vehicle Dynamics
Blockset。工具箱已安装不代表当前阶段必须依赖它们；具体依赖应随算法实现确定。

## 5. 开发约定

### 单位

  物理量     单位
  ---------- --------
  位置       m
  速度       m/s
  加速度     m/s²
  角度       rad
  角速度     rad/s
  角加速度   rad/s²
  质量       kg
  转动惯量   kg·m²
  力         N
  力矩       N·m
  侧偏刚度   N/rad

前、后轴侧偏刚度必须注明是**单轮值**还是**整轴等效值**，避免在轮胎力计算中重复乘以
2。

### 函数规范

主要函数的文件头应说明用途、参考来源、输入输出、单位、状态/符号约定、模型假设和适用范围。函数尽量只承担一个明确任务：模型计算、参数定义、仿真组织和测试彼此分离。

### 测试规范

不能只以"程序没有报错"作为模型正确的依据。至少检查零输入/零初始状态、输入符号与响应方向、方程量纲、数值积分与时间步长影响、理论预期及实验可重复性。

## 6. Git 提交约定

  前缀         用途            示例
  ------------ --------------- ------------------------------------------
  `feat`       新增模型/功能   `feat: implement 2dof vehicle dynamics`
  `fix`        修正错误        `fix: correct rear tire slip angle sign`
  `test`       测试            `test: add zero-input 2dof test`
  `docs`       文档            `docs: update reproduction roadmap`
  `refactor`   代码整理        `refactor: separate vehicle parameters`
  `chore`      工程维护        `chore: configure MATLAB project paths`

每完成一个可解释、可验证的小任务就提交一次。不要提交 MATLAB
缓存、临时文件、个人配置、可重新生成的大型中间文件，或未获授权的出版社论文
PDF。

## 7. 参考资料

-   Falcone et al. (2005), *MPC-based approach to active steering for
    autonomous vehicle systems*.
-   Falcone et al. (2007), *Predictive Active Steering Control for
    Autonomous Vehicle Systems*.
-   Rajesh Rajamani, *Vehicle Dynamics and Control*, 2nd Edition.

期刊、卷期、页码和 DOI 应在核对原始文献后补全。

## 8. 复现记录

每个阶段记录目标、对应文献章节/公式、参数及来源、实现假设与原文差异、测试工况与结果、未解决问题和对应
Git commit。

请区分三个不同层次：**代码已编写、模型已通过测试、论文结果已复现**。它们不能互相替代。
