import subprocess
import sys
import os

comando_ros = "source ~/edrom_main/install/setup.bash && ros2 launch edrom_bringup robot.launch.py camera_idx:='0' img_output:=True teleop:=False"
log_file = open("ros2_edrom_log.txt", "w")

print("Iniciando o bringup do robô em background...")

processo = subprocess.Popen(
    comando_ros,
    shell=True,
    executable='/bin/bash',
    stdout=log_file,
    stderr=subprocess.STDOUT,
    start_new_session=True
)

# A MÁGICA: Pega o ID do Grupo de Processos inteiro (a árvore toda)
pgid = os.getpgid(processo.pid)

# Salva esse número em um arquivo de texto
with open("robot_pgid.txt", "w") as f:
    f.write(str(pgid))

print(f"✅ Bringup iniciado! PGID ({pgid}) salvo em 'robot_pgid.txt'.")
print("Pode desconectar o cabo de rede!")

sys.exit(0)