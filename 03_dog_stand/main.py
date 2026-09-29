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
KP = 35
KD = 4
TAU_MAX = 33.5
JOINT_Q_START = 7
JOINT_DQ_START = 6
NUM_JOINTS = 12
DAMPING_KD = 0.5

INTERP_STEP = 0.005  # 插值步长，放慢站立速度，更稳

# 收膝目标
RETRACT_Q = np.array([
    0.0,  1.0, -1.45,   # 前左
    0.0, -1.0, 1.45,  # 前右
    0.0, -1.0,  1.45,  # 后右
    0.0,  1.0, -1.45    # 后左
])
# 半蹲站立目标
STAND_Q = np.array([
    0.0,  0.8, -1.25,   # 前左
    0.0, -0.8,  1.25,  # 前右
    0.0, -0.8,  1.25,  # 后右
    0.0,  0.8, -1.25    # 后左
])

# 状态机与按键定义 
STATE_DAMPING = 0  # 阻尼模式
STATE_STANDING = 1 # 站立模式
STAGE_RETRACT_KNEE = 2  # 阶段1：收膝盖
STAGE_LIFT = 3           # 阶段2：站起身体
KEY_STAND = 32     # 空格键 → 站立模式
KEY_DAMP = 68      # D键 → 阻尼模式


class Simulator:
    def __init__(self, scene_path):
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = None

        self.state = STATE_DAMPING          # 默认启动为阻尼模式
        self.current_target_q = np.zeros(NUM_JOINTS)  # 预初始化，后续插值用

        self.stage = STAGE_RETRACT_KNEE

    def set_timestep(self, timestep):
        self.model.opt.timestep = timestep

    def step(self):
        mujoco.mj_step(self.model, self.data)

    #站立模式计算
    def _compute_mit_torque(self, q_des):
        q = self.data.qpos[JOINT_Q_START : JOINT_Q_START + NUM_JOINTS]
        dq = self.data.qvel[JOINT_DQ_START : JOINT_DQ_START + NUM_JOINTS]

        tau = KP * (q_des - q) + KD * (0 - dq)
        tau = np.clip(tau, -TAU_MAX, TAU_MAX)

        return tau

    #阻尼模式计算
    def _compute_damping_torque(self):
        dq = self.data.qvel[JOINT_DQ_START : JOINT_DQ_START + NUM_JOINTS]

        tau = DAMPING_KD * (0 - dq)
        tau = np.clip(tau, -TAU_MAX, TAU_MAX)

        return tau

    def _smoothly_stand(self):
        #平滑插值

        if self.stage == STAGE_RETRACT_KNEE:
            diff = RETRACT_Q - self.current_target_q
            if np.max(np.abs(diff)) < 0.02:
                self.stage = STAGE_LIFT
                return
            step = np.clip(diff, -INTERP_STEP, INTERP_STEP)
            self.current_target_q += step
            
        else:
            diff = STAND_Q - self.current_target_q
            if np.max(np.abs(diff)) < 1e-3:
                self.current_target_q = STAND_Q.copy()
                return
            step = np.clip(diff, -INTERP_STEP, INTERP_STEP)
            self.current_target_q += step
        """
        # 单阶段平滑插值：从当前姿态直接过渡到最终站立姿态
        diff = STAND_Q - self.current_target_q

        if np.max(np.abs(diff)) < 1e-3:
            self.current_target_q = STAND_Q.copy()
            return

        step = np.clip(diff, -INTERP_STEP, INTERP_STEP)
        self.current_target_q += step"""

    def run(self):
        #引入键盘切换
        def key_callback(key):
            if key == KEY_STAND:
                self.state = STATE_STANDING
                self.stage = STAGE_RETRACT_KNEE  # 进入站立模式，默认从收膝阶段开始
                self.current_target_q = self.data.qpos[JOINT_Q_START : JOINT_Q_START + NUM_JOINTS].copy()
                print(">>切换至站立")
            elif key == KEY_DAMP:
                self.state = STATE_DAMPING
                print(">>切换至阻尼模式")

        with mujoco.viewer.launch_passive(self.model, self.data, key_callback=key_callback) as viewer:
            self.viewer = viewer
            while viewer.is_running():
                step_start = time.time()

                #两阶段计算
                if self.state == STATE_STANDING:
                    self._smoothly_stand()
                    tau = self._compute_mit_torque(self.current_target_q)
                else:
                    tau = self._compute_damping_torque()
                
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


