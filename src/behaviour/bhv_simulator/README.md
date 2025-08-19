<center>

# PACOTE DE INTEGRAÇÃO WEBOTS E ROS2

</center>

<table align="center">
  <tr>
    <td align="center" width="50%">
      <a href="https://www.ros.org/">
        <img src="https://raw.githubusercontent.com/fkromer/awesome-ros2/de33fced67310091a68ccf48cc29c990c047fb8f/ros_logo.svg" alt="ROS 2 Logo" width="200"/>
      </a>
      <br>
      <b></b>
    </td>
    <td align="center" width="50%">
      <a href="https://cyberbotics.com/">
        <img src="https://camo.githubusercontent.com/3c846321803781713f9f46cff7379edb386c745293eedb296298d64baa310f7f/68747470733a2f2f7777772e746865636f6e73747275637473696d2e636f6d2f77702d636f6e74656e742f75706c6f6164732f323031352f31302f7765626f74732d312e706e67" alt="Webots Logo" width="200"/>
      </a>
      <br>
      <b></b>
    </td>
  </tr>
</table>


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

**Aviso:** Se este passo falhar com um erro de <code>Cannot locate rosdep definition</code>, 
instale os pacotes mencionados manualmente (<code>ex: sudo apt install ros-humble-tf-transformations</code>) 
e execute rosdep novamente.

Por fim, compile o pacote (essa ação pode levar alguns minutos).
```
colcon build
```

## Executar a simulação

Entre no diretório em questão.
```
cd edrom_main/src/behaviour/bhv_simulator/
```

E por fim, execute
```
source install/setup.bash
ros2 launch bhv_simulator behaviour_simulator.launch.py
```

