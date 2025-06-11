<center>

# EDROM MAIN REPOSITORY

</center>
<img src='https://static.wixstatic.com/media/9d5617_a4c77748888a4c0e8884d1873f55337d~mv2.png/v1/fill/w_440,h_440,al_c,q_85,usm_1.20_1.00_0.01,enc_auto/SIMBOLO%20EDROM.png' align='center' width='200px' height='200px'>
<center>
This repository contains most of the software needed for the RoboCup Humanoid Soccer League.
</center>

## Docker Workspace (ROS2 Humble)
### Como rodar
*É importante ressaltar que é necessário ter Docker instalado em sua máquina.*

Esteja dentro do diretório root **edrom_main** e execute o seguinte comando para buildar a imagem
```
docker compose up -d
```
Agora, verifique se o container foi iniciado:
```
docker ps
```
Feito isso, execute o container:
```
docker compose exec ros2_dev /bin/bash
```
Com isto, você já deve estar dentro do container, para verificar se o ROS2 está funcionando corretamente:
```
ros2 topic list
```
E caso queira garantir:
```
colcon build --symlink-install
source install/setup.bash
ros2 run demo_nodes_cpp talker
```
**Aviso:** Para remover a instância do Container, execute:
```
docker compose down
```
**Opcional:** Caso precisa buildar o Container, faça:
```
cd docker
docker build --no-cache -t edrom_env .
```
