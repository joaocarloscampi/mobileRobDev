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


## Estrutura do repositório

| Diretório | Descrição |
|-----------|-----------|
| config    | Arquvios de configuração para o slam_toolbox, Nav2 (Não utilizado ainda) e do robô |
| docker    | Dockerfile para execução do ambiente com isaac_ros_common. (Não utilizado) |
| include   | Headers para os nodes. |
| launch    | Contém launch files. |
| maps      | Contém mapas salvos gerados pelo pacote slam_toolbox. |
| rviz      | Contém os arquivos de configuração para visualização no rviz. |
| src       | Contém o código fonte dos nodes. |
| urdf      | Contém descrição do robô em urdf, com macros para simulação em Gazebo e macros de inércia. |
| worlds    | Contém mapas salvos para execução no Gazebo. (Não utilizado) |

## Arquivos de execução launch

| Comando | Descrição |
|---------|-----------|
| `ros2 launch robovisor yolo_general.launch.py` | Inicia todas as funcionalidades do robô com base no arquivo de configuração robot_parameters.yaml. |
| `ros2 launch robovisor yolo_robot.launch.py` | Inicia robot_state_publisher e rviz para visualização do robô com driver ZLAC. |
| `ros2 launch robovisor yolo_robot_movebase.launch.py` | Inicia robot_state_publisher e rviz para visualização do robô com driver antigo (movebase). |
| `ros2 launch robovisor yolo_mapping.launch.py` | Inicia o mapeamento com o slam_toolbox e yolo_robot. |
| `ros2 launch robovisor yolo_zed.launch.py` | Inicia o ZED Wrapper para a inicialização da câmera. |
| `ros2 launch robovisor map_saver.launch.py` | Inicia o server para salvar o mapa. |
| `ros2 launch robovisor publisher.launch.py` | Inicia robot_state_publisher e rviz para visualização do robô. Inicia simulador caso indicado. (Não testado nesse pacote) |
| `ros2 launch robovisor simulation.launch.py` | Inicia Gazebo com mapa padrão, inicializa robô na simulação. (Não testado nesse pacote) |
| `ros2 launch robovisor slam.launch.py` | Inicia mapeamento SLAM. (Não testado nesse pacote) |
| `ros2 launch robovisor navigation.launch.py` | Inicia navegação com Nav2. (Não testado nesse pacote) |
| `ros2 launch robovisor joystick.launch.py` | Inicia controlador via joystick do XBox. (Não testado nesse pacote) |
| `ros2 run robovisor zlac/robovisor_nome` | Inicia node de controle e comunicação com driver. (Não testado nesse pacote) |
| `ros2 run robovisor odom_dump` | Publica ondas e salva resposta em arquivo `.csv`. (Não testado nesse pacote) |

## Execução principal do pacote

Este pacote apresenta diversos arquivos launch para diferentes funções, conforme descrito acima. Entretanto, para executar todas as funcionalidades com robô + câmera, existe um script principal

```bash
cd ~/your_ros2_wksp/scripts
./runRobot.bash
```
O script executa dois terminais: O primeiro, principal, que inicia o launch ```yolo_general.launch.py```, que pode iniciar o driver, mapeamento, lidar e RVIz. O segundo, inicia o ZED Wrapper paralelamente ao launch princial com os dois tipos de cameras presentes no laboratório: ZED e ZEDm. O motivo para isso é solucionar o conflito de inicialização do ZED Wrapper com a do RPLidar, não iniciando-o por problemas de TIMEOUT. 

O launch ```yolo_general.launch.py``` foi construido de modo a englobar todas as funcionalidades disponíveis do robô móvel, selecionando qual é a desejada no momento. Para isso, é necessário acessar o arquivo config/robot_parameters.yaml e selecionar a partir dos comentários cada uma das coisas.

## Execução da localização com AMCL

Este pacote conta com a opção de realizar a localização com um mapa utilizando o filtro de particulas AMCL, do pacote NAV2. A execução dele necessita de alguns passos a mais:
- Inicie o RVIZ e abra o arquivo de configuração localizado em `rviz/localization.rviz`;
- Execute o launch `yolo_general.launch.py` (ou o script `./runRobot.bash`) e espere o mapa carregar no rviz;
- Utilize a ferramenta 2D Pose Estimation do rviz para fornecer a posição atual do robô executado. Houve a tentativa de fornecer uma posição ideal a partir dos parâmetros, mas o pacote do AMCL quebra ao movimentar o robô.

O arquivo de mapa por enquanto é fixo, como um arquivo de mapa padrão do 2º andar do Departamento de Computaçao. Caso deseje alterar o mapa, entre no launch `yolo_general.launch.py` e troque manualmente.
