import mujoco
import mujoco.viewer
import numpy as np

# 1. 加载你的机器人模型
model = mujoco.MjModel.from_xml_path('scene.xml')
data = mujoco.MjData(model)

# 2. 如果你的 g1.xml 确实缺失了地面和光照，
# 你可以在加载后通过 mujoco.mj_addFile 或者手动定义场景
# 但最简单的方法是使用一个包含 floor 的辅助 XML，或者手动配置 viewer

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 【核心代码】强行重置机器人的基座位置（Pelvis 3D位置和姿态）
        # 前3个是x,y,z位置，第4-7个是四元数姿态
        data.qpos[0:3] = [-0.5, -0.15, 0.75]  # 让它悬空站立在 0.75 米高度
        data.qpos[3:7] = [1, 0, 0, 0]  # 保持身体笔直不倾斜
        
        # 也可以顺便把所有的手臂关节、腿部关节锁定在0位（或者特定站立角度）
        # data.qpos[7:model.nq] = 0 

        mujoco.mj_step(model, data)
        viewer.sync()
