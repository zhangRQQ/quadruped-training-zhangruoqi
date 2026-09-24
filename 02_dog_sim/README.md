# 02_dog_sim 四足机器人mujoco仿真任务
## 任务完成内容
1. 将unitree狗urdf通过在线工具转换为mjcf模型；
2. 搭建平坦地面仿真场景；
3. 使用motor力矩模式执行器，仿真循环每一帧设置`data.ctrl[:]=0`零力矩；
4. freejoint自由基座，机器人从高空下落，受重力落到地面；零力矩下落地后倾倒属于正常物理现象。

## 遇到问题说明
1. 按照mujoco讲义分别建立black_description.xml(机器人本体文件)和flat_scene.xml（场景文件）并使用`<include>`组合时：
    1. 遇到`<compiler>`报错，转换出来的`models/black_description.xml`文件带有`<compiler>`，而作为被`<include> `引入的 xml 文件不能出现`<compiler>`。删掉，写mesh file 完整相对路径
    2. schema violation报错，被`<include>`的 xml 文件最外层的`<mujoco>`标签被解析器直接剥掉，导致嵌套混乱
    3. AI引导尝试放弃`<include>`改用`<attach>`，但是属性报错，要加标签？
    4. 最后不再分开，把两个xml文件合并至顶层，能够运行
2. 运行后，机器狗降落至地上站稳。但如果按GUI上RESET，会从地面冒出来、飞向天上再坠落

## 环境依赖
- mujoco python包（3.x）
- python3

## 运行方式
```bash
cd 02_dog_sim
python3 main.py
