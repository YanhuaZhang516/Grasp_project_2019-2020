## 目标

以main_darias.py为主程序.
生成 [Darias + Cloth] 仿真环境.

## 任务

参照现有主程序,分别配置机器人模型,控制器和操作环境
1. 添加机器人模型Darias
2. 实现Darias控制器：关节力矩控制（含重力补偿），关节位置PD控制，逆运动学解算
3. 在操作环境中添加Cloth

## 参考

1. [Darias机器人描述文件](https://git.ias.informatik.tu-darmstadt.de/ias_ros/darias_core/tree/master/darias_description)
2. [Cloth建模](https://git.ias.informatik.tu-darmstadt.de/ren/ip_graspdeform/tree/master/ClothSimulation)
3. [robosuite(本项目)原始版本](https://github.com/StanfordVL/robosuite.git)
4. [Pybullet机器人控制器实现](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/examples)
5. [逆运动学解算](https://git.ias.informatik.tu-darmstadt.de/ren/robottasksim/blob/master/robosuite/controllers/baxter_ik_controller.py)