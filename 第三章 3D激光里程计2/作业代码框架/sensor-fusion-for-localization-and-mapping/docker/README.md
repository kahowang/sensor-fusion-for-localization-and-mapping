# Docker Environment for Sensor Fusion

基于Docker的[多传感器融合定位/Sensor Fusion](https://www.shenlanxueyuan.com/my/course/261)学习环境.

---

## Overview

本文档旨在提供**多传感器融合定位**的标准开发环境, 减轻环境配置的工作量

---

## 安装配置Docker以及Docker-Compose

在开始使用前，首先需要在本地配置`Docker`以及`Docker-Compose`环境.

---

### 安装Docker

请参考[Docker官方文档](https://docs.docker.com/engine/install/ubuntu/)完成`Docker`环境的安装

安装完成后, 还需要进行`如下操作`, 以保证环境的易用性:

#### 将当前用户加入Docker Group

为了能在非`sudo`模式下使用`Docker`, 需要将当前用户加入`Docker Group`.

* 执行命令:
    
    ```bash
    sudo usermod -aG docker $USER
    ```

* **为了使上述变更生效，请先Logout，再Login**

---

### 安装Docker-Compose

`Docker-Compose`是基于Docker解决方案的Orchestrator. 

请参考[Docker Compose官方文档](https://docs.docker.com/compose/install/)完成`Docker-Compose`环境的安装

---

## 获取镜像

在安装完成`Docker`以及`Docker-Compose`之后，需要从`阿里云`源上获得所需镜像.

```bash
# login to Sensor Fusion registry -- default password -- shenlansf20210122:
docker login --username=937570601@qq.com registry.cn-shanghai.aliyuncs.com
# download images:
docker pull registry.cn-shanghai.aliyuncs.com/shenlanxueyuan/sensor-fusion-workspace:bionic-cpu-vnc
```

---

### 启动实例

在当前Repo`根目录`下, 启动Terminal, 执行命令, 启动Docker Workspace:

```bash
docker-compose down && docker-compose up
```

成功启动后, 命令行输出如下:

<img src="doc/01-launch-instance.png" width="100%" alt="Launch Workspace"/>

---

### Service Health Check

然后打开`Chrome`浏览器, 访问URL`http://localhost:49001/`, 默认账号/密码为`sensorfusion/sensorfusion`, 确保所有服务成功启动. 

若所有服务成功启动, 系统状态如下图所示:

<img src="doc/02-service-health-check.png" width="100%" alt="Service Health Check"/>

---

### 访问工作空间

接着在`Chrome`浏览器中, 访问URL`http://localhost:40080/`, 默认登录密码为`sensorfusion`, 访问Docker Workspace

<img src="doc/03-access-workspace.png" width="100%" alt="Access Workspace"/>

该Workspace可理解为一个在浏览器中的`Ubuntu 18.04 Bionic`环境. 可在其中进行一切Ubuntu环境下的开发操作.

---

### 编译作业

请将作业所需的`源代码`与`数据`, 分别放到当前Repo`workspace/assignments`与`workspace/data`目录下. Docker Workspace会将当前Repo`workspace`文件夹映射到Docker Instance`/workspace`目录下. 

可在Docker Workspace中执行如下命令, 确保两者--`当前Repo workspace文件夹`与`Docker Instance /workspace`文件夹--的一致性

<img src="doc/04-mount-native-workspace-into-docker.png" width="100%" alt="Mount Native Workspace into Docker"/>

---

### 常见问题

1. Docker运行时默认用户为`root`, 运行过程中可能导致`当前Repo workspace文件夹`的User以及Group变更为`root`, 从而使本地文件IO操作因`Permission Denied`失败. 解决方案: 使用chown命令, 变更User-Group:

```bash
sudo chown [CURRENT_USER]:[CURRENT GROUP] workspace
```
