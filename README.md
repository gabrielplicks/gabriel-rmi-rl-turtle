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

## Para executar treinamento:

1. Inicie a simulação do Stage:
   - roslaunch turtlebot_stage turtlebot_in_stage.launch
2. Inicie o treinamento:
   - rosrun rl_turtle environment.py
3. Você pode acelerar a simulação passo-a-passo utilizando a tecla "]", ou pelo menu

* Os comandos devem ser executados em terminais separados

ATENÇÃO: Utilize os mapas contidos na pasta "maps" deste repositório, trocando a referência no arquivo de launch do Stage simulator para o mapa desejado.


Gabriel Paludo Licks

Robótica Móvel Inteligente 2018/2
