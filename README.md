# Robô móvel - mobileRobDev
Repositório utilizado pra desenvolver o código-fonte do driver dos robôs móveis presentes no LARIS para a execução do projeto FAPESP 2023/06578-6, sob orientação do Prof. Dr. Roberto Inoue

## Dependências

As dependências se baseiam em pacotes e bibliotecas para o ROS2 Foxy e algumas nativas do sistema operacional Ubuntu 20.04 para a NVIDIA Jetson Xavier NX

* Bibliotecas Ubuntu

```bash
sudo apt-get update
sudo apt-get install -y \
	libmodbus-dev \
	tmux \
	ros-foxy-xacro \
	ros-foxy-joint-state-publisher \
	ros-foxy-joy \
	ros-foxy-teleop-twist-joy \
	ros-foxy-teleop-twist-keyboard \
	ros-foxy-rplidar-ros \
	ros-foxy-laser-filters \
	ros-foxy-rqt \
	ros-foxy-rqt-common-plugins \
	ros-foxy-rmw-cyclonedds-cpp \
	ros-foxy-slam-toolbox \
	net-tools 
```
* Camera ZED
  
Para o uso de câmera ZED, compilar o pacote Wrapper ROS2, encontrado neste [link](https://github.com/stereolabs/zed-ros2-wrapper).

## Instalação

A instalação segue a mesma estrutura de um pacote convencional ROS2 pelos seguintes passos:
```bash
mkdir -p ~/your_ros2_wksp/src/ # create your workspace if it does not exist
cd ~/your_ros2_wksp/src/ #use your current ros2 workspace folder
git clone https://github.com/joaocarloscampi/mobileRobDev.git

cd ..
sudo apt update
colcon build --symlink-install

source ~/your_ros2_wksp/install/setup.bash
```


## estrutura

| Diretório | Descrição |
|-----------|-----------|
| config    | Contém configs para o slam_toolbox e Nav2. |
| docker    | Dockerfile para execução do ambiente com isaac_ros_common. |
| include   | Contém headers para os nodes. |
| launch    | Contém launch files. |
| maps      | Contém mapas salvos mapeados por SLAM. |
| rviz      | Contém configs para visualização no rviz. |
| src       | Contém código fonte dos nodes. |
| urdf      | Contém descrição do robô em urdf, com macros para simulação em Gazebo e macros de inércia. |
| worlds    | Contém mapas salvos para execução no Gazebo. |

## launch files

| Comando | Descrição |
|---------|-----------|
| `ros2 launch robovisor publisher.launch.py` | Inicia robot_state_publisher e rviz para visualização do robô. Inicia simulador caso indicado. |
| `ros2 launch robovisor simulation.launch.py` | Inicia Gazebo com mapa padrão, inicializa robô na simulação. |
| `ros2 launch robovisor slam.launch.py` | Inicia mapeamento SLAM. |
| `ros2 launch robovisor navigation.launch.py` | Inicia navegação com Nav2. |
| `ros2 launch robovisor joystick.launch.py` | Inicia controlador via joystick do XBox. |
| `ros2 run robovisor zlac/robovisor_nome` | Inicia node de controle e comunicação com driver. |
| `ros2 run robovisor odom_dump` | Publica ondas e salva resposta em arquivo `.csv`. |
