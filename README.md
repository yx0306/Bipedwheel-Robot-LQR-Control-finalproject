# LQR Control for Bipedal Robot Simulation

This repository contains the implementation of an LQR controller for a two-wheeled bipedal robot simulation using MuJoCo. This project demonstrates the linearization of the robot's dynamics and the design of a state-feedback controller to achieve balance and velocity tracking.
1. System Description

Based on the derivation from the course materials (Decoupled Balance Subsystem), the system state-space matrices used in this simulation are:

A Matrix (System Matrix)
State vector $x = [x, \dot{x}, \theta, \dot{\theta}, \delta, \dot{\delta}]^T$
```text
[[ 1.      0.002   0.      0.      0.      0.    ]
 [ 0.      1.     -0.0399  0.      0.      0.    ]
 [ 0.      0.      1.      0.002   0.      0.    ]
 [ 0.      0.      0.1207  1.      0.      0.    ]
 [ 0.      0.      0.      0.      1.      0.002 ]
 [ 0.      0.      0.      0.      0.      1.    ]]

B Matrix (Control Matrix)Control input $u = [T_L, T_R]^T$Plaintext[[ 0.      0.    ]
 [ 0.0125  0.0125]
 [ 0.      0.    ]
 [-0.0269 -0.0269]
 [ 0.      0.    ]
 [ 0.0392 -0.0392]]
2. Controller ConfigurationThe Linear Quadratic Regulator (LQR) is designed to minimize the cost function $J = \int (x^T Q x + u^T R u) dt$.Weight MatricesQ Matrix (State Penalty):diag([14500, 20, 2500, 100, 1, 1])Note: A high penalty on position error (14500) is applied to ensure precise tracking performance.R Matrix (Control Effort Penalty):diag([5, 5])Calculated Feedback Gain (K)The resulting gain matrix $K$ is:Plaintext[[-34.4468 -22.1623 -59.5951 -13.7672   0.2991   0.3359]
 [-34.4468 -22.1623 -59.5951 -13.7672  -0.2991  -0.3359]]
3. Simulation ResultsThe simulation results demonstrate the robot's ability to balance and track velocity commands.Performance AnalysisPitch Angle Convergence: The pitch angle successfully converges and stabilizes around -0.05 rad (equilibrium point).Angular Velocity Convergence: The angular velocity stabilizes after the initial transient response.Control Torque: The controller generates necessary torque to maintain balance. High-frequency adjustments are observed due to the high Q-matrix weights, ensuring rapid response.Phase Portrait: The trajectory in the phase plane ($\theta$ vs $\dot{\theta}$) shows a stable spiral converging to the equilibrium.Command Velocity (Bonus): The robot accurately tracks the user-commanded velocity profile from the GUI.Result Plots(Plots are generated automatically after closing the simulation window)4. How to RunPrerequisitesPython 3.xmujoconumpymatplotlibtkinter (usually included with Python)ExecutionClone this repository to your local environment.Ensure the scene.xml file path in main.py is correct.Run the simulation:Bashpython main.py
Use the GUI slider to control the robot's target velocity.Close the simulation window to generate the result plots.
***

**操作步驟提示：**
1. 打開你的專案資料夾。
2. 找到 `README.md` 檔案（如果沒有就新增一個）。
3. 將上面的程式碼區塊內容**全選複製**。
4. **直接取代**掉原本檔案裡的所有文字。


如果你需要我針對 **LQR 的 Q 矩陣參數調校** 給予更多建議，或是想加入更多關於 **Bipedal Robot 物理模型** 


