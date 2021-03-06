# RM2021哨兵

## 整体介绍
2021年双云台哨兵；来自**深圳大学RobotPilots战队**；
包含上下云台：其中上云台代码文件名为leader，下云台代码文件名为master，doc内有相应技术文档；
![整机图片](https://github.com/Sangqianli/RM2021_Sentry/blob/main/doc/%E6%95%B4%E6%9C%BA%E5%9B%BE%E7%89%87.png)
## 软件架构
### 软件架构说明

![代码整体框架](https://github.com/Sangqianli/RM2021_Sentry/blob/main/doc/%E4%BB%A3%E7%A0%81%E6%95%B4%E4%BD%93%E6%A1%86%E6%9E%B6.png)

### 程序逻辑框图

![程序逻辑框图](https://github.com/Sangqianli/RM2021_Sentry/blob/main/doc/%E7%A8%8B%E5%BA%8F%E9%80%BB%E8%BE%91%E6%A1%86%E5%9B%BE.png)


## 硬件布线框图

![硬件系统框图](https://github.com/Sangqianli/RM2021_Sentry/blob/main/doc/%E7%A1%AC%E4%BB%B6%E7%B3%BB%E7%BB%9F%E6%A1%86%E5%9B%BE.png)

## 主要功能介绍
---
### 双云台控制系统
		详见doc内技术文档相关内容，利用这套系统，可以做到只用一个遥控器就完整控制双云台，实现良好的调试控制效果，也方便比赛检录。此外，在通信链路或某个主控程序失灵的情况下，也能保证另一个云台正常工作，不至于完全瘫痪。
---
### 自瞄开火系统
		逻辑框图如下，具体实现请看doc内技术文档相关内容
![自瞄开火程序逻辑](https://github.com/Sangqianli/RM2021_Sentry/blob/main/doc/%E8%87%AA%E7%9E%84%E5%BC%80%E7%81%AB%E7%A8%8B%E5%BA%8F%E9%80%BB%E8%BE%91.png)
---
### 双环底盘跑轨
		硬件上采用1000线增量编码器和微动开关，软件上使用双环PID进行底盘跑轨控制，极大提高底盘动态性能。全轨往返时间可稳定在4.2到4.5秒。详见doc内技术文档相关内容。
---
## 开发调试工具
---
### 软件环境
**Windows10**
### 硬件环境
**STM32F407VET6**
### 编译方式
**C**
### 具体工具
1. MDK5 + STM32CubeMX开发
2. Zigbee无线调试模块和上位机
3. 自制J-Link和J-Scope软件
4. EventRecoder组件 

---
## 未来优化方向
---
	 在哨兵上进行视控一体化，添加更多维度和方向的传感器，获取精准信息，抗干扰强，提高自动跑轨和打击性能 

