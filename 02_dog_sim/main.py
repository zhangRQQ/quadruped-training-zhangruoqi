import time
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("scenes/flat_scene.xml")
data = mujoco.MjData(model)

print(f"nq(广义位置自由度): {model.nq}")
print(f"nv(广义速度自由度): {model.nv}")
print(f"nu(执行器数量，预期等于12): {model.nu}")

data.qpos[2] = 0.8

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # 读取当前状态
        qpos = data.qpos.copy()
        qvel = data.qvel.copy()

        # 计算控制量
        torque = 0.0

        # 写⼊执⾏器控制输⼊
        if model.nu > 0:
            data.ctrl[:] = torque

        #推进仿真
        mujoco.mj_step(model, data)

        #更新viewer
        viewer.sync()

        # 让显⽰速度⼤致接近真实时间
        time_left = model.opt.timestep - (time.time() - step_start)
        if time_left > 0:
            time.sleep(time_left)