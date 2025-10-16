# Mensagem principal publicada pelo nó de visão

bool searching      # Flag para indicar se a visão está ativa
uint8 fps           # Frames por segundo do nó de visão

edrom_msgs/Detection ball
edrom_msgs/Detection robot
edrom_msgs/Detection leftgoal
edrom_msgs/Detection rightgoal

edrom_msgs/DetectionArray x_intersection
edrom_msgs/DetectionArray l_intersection
edrom_msgs/DetectionArray t_intersection
edrom_msgs/DetectionArray center
