# Drago 

https://github.com/user-attachments/assets/34009592-8adb-4513-a7bd-33d4de80e65d

# Montagem e design 

## Versão atual  

<img width="596" height="577" alt="Captura de tela 2026-05-27 110014" src="https://github.com/user-attachments/assets/0215915b-2b29-4689-b322-f10d66b289b5" />

## Versão futura 

No futuro, pretendo cobrir os componentes eletrônicos, juntamente com a implementação de uma tela Oled para visualição do estado das juntas 

<img width="545" height="590" alt="Captura de tela 2026-05-28 140028" src="https://github.com/user-attachments/assets/c52ee738-e658-4472-b467-857b11f53169" />

A interface visual já foi testada no site [Wokwi](https://wokwi.com/)

<img width="423" height="471" alt="Captura de tela 2026-05-25 174843" src="https://github.com/user-attachments/assets/c3e07e82-85e9-4eec-9d79-2853cc5530a4" />

Encontre o projeto em [Meu projeto no Wokwi](https://wokwi.com/projects/464991738746474497)

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
