# 02_try_include 实验说明文档

## 版本目的
上一版02_dog_sim将xml合并至顶层，完成任务但单一文件长解耦性差，此次02_try_include验证多XML拆分 + include模块化引入的可行性。


## 实现方案
验证了 MuJoCo 中两种不同的 include 方案，均可以正常运行。

### 方案A：片段嵌入拆分式
本质是纯文本替换：子文件没有`<mujoco>`根标签，只是XML功能片段，`<include>`标签会直接被子文件内容替换。
实现要点：
1. 仅主场景 `flat_scene.xml` 保留`<mujoco>`根标签与全局`<compiler>`，全局编译配置唯一。
2. 将机器人按功能拆成3个独立XML片段：asset资源、worldbody刚体结构、actuator执行器。
3. 在主文件`flat_scene.xml`对应标签内部分别include子文件：
   - `<asset>`标签内引入网格材质文件
   - `<worldbody>`标签内引入机器人body结构
   - `<actuator>`标签内引入电机motor配置

### 方案B：完整模型合并式
本质是完整模型结构合并：子文件自带完整`<mujoco>`根标签，本身就是独立合法的模型，MuJoCo解析器会自动合并两个模型的对应模块。
实现要点：
1. `robot/black_description.xml` 保留完整模型结构：自带`<mujoco>`根、`<compiler>`编译配置、asset/worldbody/actuator全模块，原生转换文件几乎无需修改。
2. `scenes/flat_scene.xml` 作为顶层入口，仅包含场景元素（地面、灯光），通过`<include>`引入完整机器人模型。
3. 主场景文件末尾补充全局`<compiler>`，显式指定跨目录网格路径，覆盖子文件的路径配置，解决路径报错问题。

### 方案场景补充说明
两种方案均仅存在于`robot/black_description.xml`和`scenes/flat_scene.xml`按标准存放于不同文件夹情况下，为解决跨文件夹include，mesh路径拼接错误、找不到STL问题。


## 目录结构
1. `robot/`：机器人模型资源目录，为独立完整的模型单元
  - `black_description.xml`：完整机器人模型文件，自带mujoco根标签、compiler配置、asset、worldbody、actuator全模块
  - `robot_asset.xml` / `robot_worldbody.xml` / `robot_actuator.xml`：探索方案A的拆分版功能片段文件
  - `meshes/`：机器人STL网格文件目录
2. `scenes/`：场景文件目录
  - `flat_scene.xml`：顶层主场景入口文件
3. `main.py`：仿真运行主程序
4. `README.md`：说明文档


## 调试记录

### 1. 根层级直接include完整多模块XML，报`Schema violation: unrecognized element 'mesh'`
现象：将`<include>`写在`<worldbody>`标签内部，引入完整机器人XML，解析失败。
原因：写在父标签内部的include会触发「片段嵌入模式」，直接把内容塞进父标签，导致`<asset>`出现在`<worldbody>`里，语法非法。
解决：将`<include>`放在`<mujoco>`根层级（与worldbody同级），触发「完整模型合并模式」，自动按模块合并。

### 2. 跨文件夹include，mesh路径拼接错误、找不到STL
现象：`Error opening file 'meshes/robot/xxx.STL'`，路径层级错误。
原因：
- 子文件的`meshdir`相对路径，最终以顶层主XML文件所在目录为基准；
- 机器人XML与场景XML分属不同目录，子文件内的`meshdir="meshes/"`相对于场景目录路径不成立。
解决：在主场景文件`<include>`之后补充`<compiler meshdir="../robot/meshes/" />`，利用「后出现的compiler优先级更高」的规则，覆盖子文件的路径配置，适配跨目录结构。

### 3. 关于compiler标签的认知修正
- 错误认知：全局只能有一个`<compiler>`标签，子文件不能携带。
- 正确结论：允许多个`<compiler>`标签共存，最后出现的标签的同名属性优先级更高，不同属性会保留全局生效。完整模型合并模式下，子文件可以自带compiler配置。

### 4. 模型初始穿地，reset后弹射飞天
现象：按下reset重现仿真时，机器人初始位置嵌入地面，弹射升空。
原因：只在python代码中设置了初始高度，xml文件中trunk初始z坐标为0，腿部结构向下延伸，与地面产生深度几何穿透。
解决：将trunk初始z轴高度设置为0.6，避免初始碰撞穿插，模型自然下落平稳落地。


## 环境依赖
- mujoco python包（3.x）
- python3


## 运行方式
```bash
cd 02_dog_sim
python3 main.py



