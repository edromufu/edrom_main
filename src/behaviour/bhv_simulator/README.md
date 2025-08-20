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

Para comunicação entre o ambiente de simulação e os códigos ROS é necessário instalar o pacote <code>webots_ros2</code>. Para isso digite no terminal

```
sudo apt-get install ros-humble-webots-ros2
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

