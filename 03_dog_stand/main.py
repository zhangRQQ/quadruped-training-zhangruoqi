import time
import mujoco
import mujoco.viewer
import numpy as np 


# ======== 配置常量 ========
#文件路径
SCENE_PATH = "scenes/flat_scene.xml"
# 仿真参数
SIMULATE_DT = 0.002   # 物理仿真步长 500Hz
VIEWER_DT = 0.01      # 画面刷新间隔 100fps


# 控制参数
KP = 20
KD = 5
TAU_MAX = 33.5
JOINT_Q_START = 7
JOINT_DQ_START = 6
NUM_JOINTS = 12

# 站立目标角度（输出侧，弧度，按关节顺序排列）
# 示例：4条腿，每条腿3个关节（侧摆/髋/膝），共12个
STAND_Q = np.array([
    0.0, 0.22, 0.9,  
    0.0, -0.22, -0.9,   
    0.0, -0.22, -0.9,   
    0.0, 0.22, 0.9 
])


class Simulator:
    def __init__(self, scene_path):
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = None

        self.state = STATE_DAMPING          # 默认启动为阻尼模式
        self.current_target_q = np.zeros(NUM_JOINTS)  # 插值过程中的目标角度

    def set_timestep(self, timestep):
        self.model.opt.timestep = timestep

    def step(self):
        mujoco.mj_step(self.model, self.data)

    def _compute_mit_torque(self, q_des):
        q = self.data.qpos[JOINT_Q_START : JOINT_Q_START + NUM_JOINTS]
        dq = self.data.qvel[JOINT_DQ_START : JOINT_DQ_START + NUM_JOINTS]

        tau = KP * (q_des - q) + KD * (0 - dq)
        tau = np.clip(tau, -TAU_MAX, TAU_MAX)

        return tau

    def run(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            self.viewer = viewer
            while viewer.is_running():
                step_start = time.time()

                # 读取当前状态
                qpos = self.data.qpos.copy()
                qvel = self.data.qvel.copy()

                # 计算控制量
                tau = self._compute_mit_torque(STAND_Q)

                # 写⼊执⾏器控制输⼊
                if self.model.nu > 0:
                    self.data.ctrl[:] = tau

                #推进仿真
                self.step()

                #更新viewer
                viewer.sync()

                # 让显⽰速度⼤致接近真实时间
                time_left = self.model.opt.timestep - (time.time() - step_start)
                if time_left > 0:
                    time.sleep(time_left)

if __name__ == "__main__":
    sim = Simulator(SCENE_PATH)
    sim.set_timestep(SIMULATE_DT)
    sim.run()


