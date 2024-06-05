# robovisor
workspace pra desenvolver código ROS2 para o projeto de iniciação científica MAI/DAI Robovisor 
2023 sob supervisão do Prof. Dr. Roberto Inoue

## dependências

as dependências agora são instaladas automaticamente pelo arquivo ```docker/Dockerfile.robovisor```.
novas dependências também devem ser adicionadas nesse arquivo

## instalação

instalar isaac_ros_common para utilização do docker:
```
$ cd ~
$ git clone git@github.com:NVIDIA-ISAAC-ROS/isaac_ros_common.git
$ cd isaac_ros_common/scripts
```

dentro da pasta scripts, criar o arquivo ```.isaac_ros_common-config```:
```
CONFIG_IMAGE_KEY="ros2_humble.robovisor"
CONFIG_DOCKER_SEARCH_DIRS="../../workspaces/isaac_ros-dev/robovisor_ws/src/mobileRobDev/docker"
```

será necessário instalar pacotes do docker, como ```docker.io```ou ```docker.buildx```

criar um workspace ros2:
```
$ mkdir -p ~/workspaces/isaac_ros-dev/robovisor_ws/src
$ cd workspaces/isaac_ros-dev/robovisor_ws
$ colcon build
$ cd src
```
clonar o workspace localmente com ```git clone -b develop git@github.com:rsinoue/mobileRobDev.git```.
compilar novamente o ambiente com ```colcon build --symlink-install``` em ```robovisor_ws```.

para desenvolver o código ou navegar o robô, sempre utilizar o docker, rodando o comando em
```~/isaac_ros_common/scripts```:
```
$ ./run_dev.sh
```
o docker mapeia imediatamente os devices USB já conectados e montados em ```/dev``` para dentro 
do ambiente. as interfaces GUI também funcionam normalmente.

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
