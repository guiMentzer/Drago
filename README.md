# Drago 

https://github.com/user-attachments/assets/34009592-8adb-4513-a7bd-33d4de80e65d

# Pacotes ROS2

## 'drago_pkg' 

O pacote 'drago_pkg' contém o arquivo URDF do robô, com a pasta **meshes** contendo os arquivos STL dos elos. Ademais, contém um *launch* para a visualição do robô (Rviz) com o controle das juntas (Joint State Publisher).  

<img width="1366" height="768" alt="Captura de tela 2026-05-08 201146" src="https://github.com/user-attachments/assets/e38932b7-6d1e-4e97-ad76-c8f48fb75222" />
<img width="1366" height="768" alt="Captura de tela 2026-05-08 201346" src="https://github.com/user-attachments/assets/b5553b8e-1732-4c55-ab12-f34992cdac7b" />

## 'drago_moveit_pkg' 

O pacote 'moveit_drago_pkg' contém a configuração e inicialização do **MoveIt**. Contém o *launch* para a visualização do robô com a janela de planejamento de trajetória do MoveIt (Motion Planning). Contém outro *launch* semelhante, mas com o construtor de rotinas do MoveIt (Task Constructor), mas este ainda não funciona. 

<img width="1366" height="768" alt="Captura de tela 2026-05-08 195242" src="https://github.com/user-attachments/assets/a28a0694-7158-4d42-ba10-5523f84902f8" />
<img width="1366" height="768" alt="Captura de tela 2026-05-08 195305" src="https://github.com/user-attachments/assets/5d477ac8-0c83-4faa-907c-0b5ed3869ca3" />
<img width="1366" height="768" alt="Captura de tela 2026-05-08 195428" src="https://github.com/user-attachments/assets/4b941be4-577e-47d7-bb26-47e960e1846b" />

## 'robot_hardware_interface'

Este pacote contém todo o código de interface de hardware para que o ROS2 se comunique com o Arduino. Configura as interfaces de comando (command_interfaces) e de estado (state_interface) e cria um protocolo para enviar os ângulos de juntas via serial ao Arduino.

## 'drago_serial' 

Este não é um pacote ROS2. É o código carregado no Arduino, para a interpretação da mensagem recebida serialmente e atuação dos servos motores. Utiliza a biblioteca **Adaftuit_PCA9685** para controlar os servos pelo driver I²C PCA9685. 
