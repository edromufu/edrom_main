## EDROM Docker Workspace (ROS2 Humble)
### Pré-requisitos
Para executar os passos de configuração, é importante ter o Docker instalado no seu Ubuntu 22.04.

Caso não tenha, acesse o [site oficial](https://docs.docker.com/desktop/setup/install/linux/ubuntu/).
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

