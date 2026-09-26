# 02_try_include 实验说明文档

## 版本目的
上一版02_dog_sim将xml合并至顶层，完成任务但单一文件长解耦性差，此次02_try_include验证多XML拆分 + include模块化引入的可行性。

## 实现方案
采用MuJoCo模块化拆分，使用include做文件解耦：
1. 仅主场景`flat_scene.xml`保留`<mujoco>`根标签与全局`<compiler>`，全局编译配置只写一次。
2. 将机器人拆成3个独立xml：asset资源、worldbody刚体结构、actuator执行器。
3. 在主文件对应标签内部分别include子文件：
   - `<asset>`内引入网格材质文件
   - `<worldbody>`内引入机器人body结构
   - `<actuator>`内引入电机motor

## 调试记录
### 1.根层级同时include多块内容，报`Schema violation: unrecognized element 'mesh'`
现象：直接include同时包含asset+worldbody+actuator的子xml，解析失败。
原因：被include的子文件只能有单个顶层标签，不能并列多个不同顶层标签。
解决：按功能拆分为3个独立xml，在主文件对应父标签内分别引入。

### 2.mesh路径报错，找不到stl文件，路径重复叠加
现象：`Error opening file 'robot/meshes/robot/xxx.STL'`
原因：`meshdir`的相对路径以主xml（flat_scene.xml）作为基准，不是子xml。
解决：compiler中设置`meshdir="../robot/meshes/"`；子xml内mesh只写文件名，不要写路径。

### 3.子xml携带`<compiler>`标签，解析报错
原因：MuJoCo只允许一份compiler全局配置，include引入的子文件不能带`<compiler>`、`<mujoco>`根标签。
解决：全部compiler配置放到顶层scene，子文件只保留功能内容。

## 环境依赖
- mujoco python包（3.x）
- python3

## 运行方式
```bash
cd 02_dog_sim
python3 main.py



