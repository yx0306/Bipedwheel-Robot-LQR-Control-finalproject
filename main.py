import math
import time
import os
import threading
import tkinter as tk
import numpy as np
from numpy.linalg import inv
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R_scipy
import matplotlib.pyplot as plt

# ==========================================
# PART 0: 滑鼠控制介面 (GUI)
# ==========================================
class RobotControllerGUI:
    def __init__(self):
        self.cmd_vel = 0.0
        self.running = True
        self.thread = threading.Thread(target=self._run_gui)
        self.thread.daemon = True
        self.thread.start()

    def _run_gui(self):
        self.root = tk.Tk()
        self.root.title("速度控制器")
        self.root.geometry("200x350")
        self.root.attributes('-topmost', True)
        label = tk.Label(self.root, text="拖拉控制速度 (m/s)", font=("Arial", 12))
        label.pack(pady=10)
        self.slider = tk.Scale(self.root, from_=0.8, to=-0.8, 
                               resolution=0.01, orient='vertical', 
                               length=200, width=20, label="速度",
                               command=self._update_val)
        self.slider.set(0)
        self.slider.pack()
        btn = tk.Button(self.root, text="急停 / 歸零 (Reset)", command=self._reset, bg="red", fg="white", font=("Arial", 10, "bold"))
        btn.pack(pady=10)
        self.root.mainloop()

    def _update_val(self, val):
        self.cmd_vel = float(val)

    def _reset(self):
        self.slider.set(0)
        self.cmd_vel = 0.0

# ==========================================
# PART 1: 工具函數
# ==========================================
def pd_control(target_q, q, kp, target_dq, dq, kd):
    return (target_q - q) * kp + (target_dq - dq) * kd

# ==========================================
# PART 2: 機器人參數 (請務必核對 PDF 上的數值)
# ==========================================
class RobotParameters:
    def __init__(self):
        self.g = 9.81
        self.M = 7.08       # 請核對助教 PDF
        self.m = 0.28       # 請核對助教 PDF
        self.l = 0.37       # 請核對助教 PDF
        self.r = 0.07       
        self.d = 0.36       
        
        self.I = 0.5 * self.m * (self.r ** 2)
        self.Jp = (1/3) * self.M * (self.l ** 2) 
        self.Id = 0.001
        self.J_delta = (1/12) * self.M * (self.d ** 2)

    def calculate_Qeq(self):
        term_wheel = 2 * self.m + 2 * self.I / (self.r ** 2)
        term_pendulum = self.Jp + self.M * (self.l ** 2)
        Qeq = (self.Jp * self.M) + (term_pendulum * term_wheel)
        return Qeq

params = RobotParameters()

# LQR 設定
nx = 6 
nu = 2 
delta_t = 0.002 
Q = np.diag([14500.0, 20.0, 2500.0, 100.0, 1.0, 1.0]) 
R = np.diag([5, 5])

def get_model_matrix():
    p = params
    Qeq = p.calculate_Qeq()
    M, m, l, r = p.M, p.m, p.l, p.r
    I, Jp, Id, J_delta, d = p.I, p.Jp, p.Id, p.J_delta, p.d
    g = p.g

    # 這裡的公式是根據倒單擺模型線性化得出的
    A23 = -(M**2 * l**2 * g) / Qeq
    term_wheel_A43 = M + 2*m + 2*I/(r**2)
    A43 = (M * l * g * term_wheel_A43) / Qeq

    A_continuous = np.zeros((nx, nx))
    A_continuous[0, 1] = 1.0; A_continuous[2, 3] = 1.0; A_continuous[4, 5] = 1.0
    A_continuous[1, 2] = A23; A_continuous[3, 2] = A43

    val_B2 = (Jp + M * l**2 + M * l * r) / (Qeq * r)
    term_B4 = (M * l / r) + M + 2*m + 2*I/(r**2)
    val_B4 = -term_B4 / Qeq
    denom_6 = r * (m * d + Id / (r**2) + 2 * J_delta / d)
    val_B6 = 1.0 / denom_6

    B_continuous = np.zeros((nx, nu))
    B_continuous[1, 0] = val_B2; B_continuous[3, 0] = val_B4; B_continuous[5, 0] = val_B6 
    B_continuous[1, 1] = val_B2; B_continuous[3, 1] = val_B4; B_continuous[5, 1] = -val_B6 

    A = np.eye(nx) + A_continuous * delta_t
    B = B_continuous * delta_t
    return A, B

def dlqr(A, B, Q, R):
    P = Q
    for i in range(1000):
        Pn = A.T @ P @ A - A.T @ P @ B @ inv(R + B.T @ P @ B) @ B.T @ P @ A + Q
        if (abs(Pn - P)).max() < 0.001: break
        P = Pn
    return inv(B.T @ P @ B + R) @ (B.T @ P @ A)

# ==========================================
# PART 3: 狀態讀取
# ==========================================
def get_real_state_from_q(model, data, prev_x_pos, wheel_joint_adr):
    w, x, y, z = data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]
    r_rot = R_scipy.from_quat([x, y, z, w])
    euler = r_rot.as_euler('xyz', degrees=False)
    theta = euler[1] 
    theta_dot = data.qvel[4] 
    delta_dot = data.qvel[5] 
    l_vel_idx = wheel_joint_adr['L_wheel']
    r_vel_idx = wheel_joint_adr['R_wheel']
    wl = data.qvel[l_vel_idx]
    wr = data.qvel[r_vel_idx]
    x_dot = (wl + wr) / 2.0 * params.r
    x_pos = prev_x_pos + x_dot * delta_t 
    return np.array([[x_pos], [x_dot], [theta], [theta_dot], [0.0], [delta_dot]])

# ==========================================
# PART 4: 矩陣列印
# ==========================================
def print_matrix(name, matrix):
    print(f"\n{name} Matrix:")
    print("-" * 30)
    with np.printoptions(precision=4, suppress=True, linewidth=100):
        print(matrix)
    print("-" * 30)

def main():
    # 請修改為你的 XML 路徑
    xml_path = (r'D:\mujoco_course-master\crazydog_urdf\scene.xml')
    
    if not os.path.exists(xml_path):
        print(f"[錯誤] 找不到檔案: {xml_path}")
        return

    try:
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"[錯誤] XML 載入失敗: {e}")
        return

    model.opt.timestep = delta_t
    
    # Actuator Mapping
    try:
        ACT_IDX = {
            'L_wheel': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'L_wheel'),
            'R_wheel': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'R_wheel')
        }
        WHEEL_VEL_ADR = {}
        for name in ['L_wheel', 'R_wheel']:
            act_id = ACT_IDX[name]
            trn_id = model.actuator_trnid[act_id, 0]
            dof_adr = model.jnt_dofadr[trn_id]
            WHEEL_VEL_ADR[name] = dof_adr
    except Exception as e:
        print(f"[錯誤] Actuator 映射失敗: {e}")
        return

    # 計算矩陣
    print("正在計算系統矩陣...")
    A_mat, B_mat = get_model_matrix()
    K = dlqr(A_mat, B_mat, Q, R)
    
    # --- 這裡輸出的數據就是作業 System Description 要貼的 ---
    print("\n" + "="*50)
    print("   [System Description & Controller Configuration]   ")
    print("="*50)
    print("請務必與助教 PDF 核對 A, B 矩陣數值！")
    print_matrix("A Matrix", A_mat)
    print_matrix("B Matrix", B_mat)
    print_matrix("Q Matrix", Q)
    print_matrix("R Matrix", R)
    print_matrix("K Gain", K)
    print("="*50 + "\n")

    print("啟動 GUI 控制面板...")
    gui = RobotControllerGUI()

    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.65 
    data.qpos[3:7] = [1, 0, 0, 0] 

    curr_x = 0.0
    prev_in_air = True
    target_x_ref = 0.0
    current_smooth_vel = 0.0
    MAX_ACCEL = 0.5 * delta_t
    u_wheel_prev = 0.0
    filter_alpha = 0.25
    pitch_offset = -0.012 
    kp_ground = np.array([300.0, 300.0, 0.0, 300.0, 300.0, 0.0])
    kd_ground = np.array([15.0,  15.0,  0.0, 15.0,  15.0,  0.0])
    target_pos = np.array([0.9, -1.5, 0.0, 0.9, -1.5, 0.0])
    target_vel = np.zeros(6)

    # === 資料紀錄 (Data Logging) ===
    log_time = []
    log_pitch = []
    log_pitch_vel = []
    log_torque = []
    log_cmd_vel = []   # 加分項: 命令速度
    log_real_vel = []  # 加分項: 實際速度
    start_time = time.time()

    print("\n=== 模擬開始 ===")
    print("1. 請拖動 GUI 滑桿改變速度。")
    print("2. 觀察機器人運動。")
    print("3. **關閉模擬視窗** 後，將自動彈出 5 張結果圖表。")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            current_sim_time = time.time() - start_time

            # 1. 速度命令平滑化
            target_vel_raw = gui.cmd_vel
            vel_diff = target_vel_raw - current_smooth_vel
            if abs(vel_diff) > MAX_ACCEL:
                current_smooth_vel += np.sign(vel_diff) * MAX_ACCEL
            else:
                current_smooth_vel = target_vel_raw

            # 2. 狀態判斷
            LANDING_HEIGHT = 0.48 
            is_in_air = data.qpos[2] > LANDING_HEIGHT

            current_q = []
            current_dq = []
            for i in range(model.nu):
                trn_id = model.actuator_trnid[i, 0]
                q_adr = model.jnt_qposadr[trn_id]
                dq_adr = model.jnt_dofadr[trn_id]
                current_q.append(data.qpos[q_adr])
                current_dq.append(data.qvel[dq_adr])
            current_q = np.array(current_q)
            current_dq = np.array(current_dq)

            # 取得當前真實狀態
            x_state = get_real_state_from_q(model, data, curr_x, WHEEL_VEL_ADR)
            curr_x = x_state[0, 0]
            real_velocity = x_state[1, 0] # 暫存真實速度用於紀錄

            # 紀錄數據
            log_time.append(current_sim_time)
            log_pitch.append(x_state[2, 0])
            log_pitch_vel.append(x_state[3, 0])
            log_cmd_vel.append(current_smooth_vel)
            log_real_vel.append(real_velocity)

            if is_in_air:
                kp = kp_ground * 0.1
                kd = kd_ground * 0.5
                l_vel = data.qvel[WHEEL_VEL_ADR['L_wheel']]
                r_vel = data.qvel[WHEEL_VEL_ADR['R_wheel']]
                u_raw = -0.1 * (l_vel + r_vel)
                u_wheel = u_raw
                u_wheel_prev = u_wheel 
                curr_x = 0.0
                target_x_ref = 0.0
                prev_in_air = True
            else:
                if prev_in_air:
                    curr_x = 0.0
                    target_x_ref = 0.0
                    prev_in_air = False
                
                kp = kp_ground
                kd = kd_ground
                target_x_ref += current_smooth_vel * delta_t
                
                # 計算誤差狀態 (Error State)
                x_state[0, 0] = curr_x - target_x_ref
                x_state[1, 0] = real_velocity - current_smooth_vel # 速度誤差
                x_state[2, 0] = x_state[2, 0] - pitch_offset 
                
                u_lqr = -K @ x_state
                u_raw = np.clip(u_lqr[0, 0], -35.0, 35.0)
                u_wheel = filter_alpha * u_raw + (1 - filter_alpha) * u_wheel_prev
                u_wheel_prev = u_wheel

            log_torque.append(u_wheel)

            tau = pd_control(target_pos, current_q, kp, target_vel, current_dq, kd)
            data.ctrl[:] = tau 
            data.ctrl[ACT_IDX['L_wheel']] = u_wheel
            data.ctrl[ACT_IDX['R_wheel']] = u_wheel

            mujoco.mj_step(model, data)
            viewer.sync()

            elapsed = time.time() - step_start
            if elapsed < delta_t:
                time.sleep(delta_t - elapsed)

    print("模擬結束，正在產生作業圖表...")

    # === 繪製作業所需的 Result 圖表 ===
    plt.figure(figsize=(15, 10)) # 加大畫布

    # 1. Pitch angle convergence
    plt.subplot(2, 3, 1)
    plt.plot(log_time, log_pitch, label='Pitch', color='blue')
    plt.title('1. Pitch Angle Convergence')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.grid(True)
    plt.legend()

    # 2. Angular velocity convergence
    plt.subplot(2, 3, 2)
    plt.plot(log_time, log_pitch_vel, label='Pitch Rate', color='orange')
    plt.title('2. Ang. Vel. Convergence')
    plt.xlabel('Time (s)')
    plt.ylabel('Rate (rad/s)')
    plt.grid(True)
    plt.legend()

    # 3. Control torque
    plt.subplot(2, 3, 3)
    plt.plot(log_time, log_torque, label='Torque', color='green')
    plt.title('3. Control Torque')
    plt.xlabel('Time (s)')
    plt.ylabel('Nm')
    plt.grid(True)
    plt.legend()

    # 4. Phase portrait
    plt.subplot(2, 3, 4)
    plt.plot(log_pitch, log_pitch_vel, color='purple', linewidth=0.8)
    plt.title('4. Phase Portrait')
    plt.xlabel('Pitch (rad)')
    plt.ylabel('Pitch Rate (rad/s)')
    plt.grid(True)

    # 5. Command velocity (加分項)
    plt.subplot(2, 3, 5)
    plt.plot(log_time, log_cmd_vel, 'r--', label='Command', linewidth=2)
    plt.plot(log_time, log_real_vel, 'b-', label='Actual', alpha=0.6)
    plt.title('5. Velocity Tracking (Bonus)')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()