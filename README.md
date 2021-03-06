# gabriel-rmi-rl-turtle

Este repositório contém a implementação de um projeto que explora a utilização do Stage simulator e ROS para treinar robôs utilizando reinforcement learning.

* A pasta "docs" contém o report do trabalho desenvolvido.

## Ambiente utilizado:
- Ubuntu 18.04
- ROS Kinetic
- Stage simulator

## Dependências:
- Turtlebot Stage package

## Instalação:

1. Crie uma pasta no seu catkin_ws chamada "rl_turtle"
2. Clone o conteúdo deste repositório para a pasta criada
3. Execute o comando catkin_make na pasta catkin_ws
4. Source no arquivo devel/setup.bash

* É necessário dar permissão de execução aos arquivos Python. Utilize "chmod +x <nome_arquivo>.py"

## Para executar treinamento:

1. Inicie a simulação do Stage:
   - roslaunch turtlebot_stage turtlebot_in_stage.launch
2. Inicie o treinamento:
   - rosrun rl_turtle environment.py
3. Você pode acelerar a simulação passo-a-passo utilizando a tecla "]", ou pelo menu

* Os comandos devem ser executados em terminais separados

ATENÇÃO: Utilize os mapas contidos na pasta "maps" deste repositório, trocando a referência no arquivo de launch do pacote turtlebot_stage para o mapa desejado.

## Demo vídeos:

* Corridor environment training with Q-Learning

[![Corridor training](https://img.youtube.com/vi/0gP1HPKQ3RE/0.jpg)](https://www.youtube.com/watch?v=0gP1HPKQ3RE)

* Corridor with Obstacle environment training with Q-Learning

[![Corridor with Obstacle training](https://img.youtube.com/vi/uHoZYBAvNPc/0.jpg)](https://www.youtube.com/watch?v=uHoZYBAvNPc)


Gabriel Paludo Licks

Robótica Móvel Inteligente 2018/2
