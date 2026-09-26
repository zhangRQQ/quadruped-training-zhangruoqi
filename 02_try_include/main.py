import time
import mujoco
import mujoco.viewer

# ======== 配置常量 ========
#文件路径
SCENE_PATH = "scenes/flat_scene.xml"
# 仿真参数
SIMULATE_DT = 0.002   # 物理仿真步长 500Hz
VIEWER_DT = 0.01      # 画面刷新间隔 100fps
# 功能开关
PRINT_ROBOT_INFO = True


class Simulator:
    def __init__(self, scene_path):
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = None

    def print_info(self):
        print(f"nq(广义位置自由度): {self.model.nq}")
        print(f"nv(广义速度自由度): {self.model.nv}")
        print(f"nu(执行器数量，预期等于12): {self.model.nu}")

    def set_timestep(self, timestep):
        self.model.opt.timestep = timestep

    def step(self):
        mujoco.mj_step(self.model, self.data)

    def run(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            self.viewer = viewer
            while viewer.is_running():
                step_start = time.time()

                # 读取当前状态
                qpos = self.data.qpos.copy()
                qvel = self.data.qvel.copy()

                # 计算控制量
                torque = 0.0

                # 写⼊执⾏器控制输⼊
                if self.model.nu > 0:
                    self.data.ctrl[:] = torque

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
    if PRINT_ROBOT_INFO:
        sim.print_info()
    sim.run()





