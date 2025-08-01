<center>

# PACOTE DE INTEGRAÇÃO WEBOTS E ROS2

</center>

<p align='center'>
<img src='https://raw.githubusercontent.com/cyberbotics/webots_ros2/master/webots_ros2_tiago/docs/images/tiago_moveit2_example.png' width='20%'>
</p>


<center>
Este guia detalha o processo de instalação do pacote <code>webots_ros2</code> compilando-o a partir do código-fonte. 
Esta abordagem é a mais recomendada para garantir a versão mais recente e todos os componentes necessários, 
evitando os problemas comuns encontrados em instalações via <code>apt</code>.
</center>


### Configuração do pacote <code>webots_ros2</code>

Para compilar pacotes de terceiros o ideal é um workspace separado para manter o ambiente organizado.
```
mkdir -p ~/webots_ros2_ws/src
cd ~/webots_ros2_ws
```
Clonar o repositório do webots_ros2 a partir do GitHub.

```
git clone https://github.com/cyberbotics/webots_ros2.git src/webots_ros2
```

Vá à pasta clonada, inicialize e baixe os submódulos de forma recursiva.

```
cd src/webots_ros2
git submodule update --init --recursive
```

Na raiz do workspace, instale as dependencias com rosdep.

```
cd ~/webots_ros2_ws
sudo apt update
rosdep install --from-paths src -y --ignore-src
```

**Aviso:** Se este passo falhar com um erro de <code>Cannot locate rosdep definition</code>., 
instale os pacotes mencionados manualmente (<code>ex: sudo apt install ros-iron-tf-transformations</code>) 
e execute rosdep novamente.

Por fim, construa o pacote (essa ação pode levar alguns minutos).
```
colcon build
```





