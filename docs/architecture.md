# Arquitetura do UnBall SSL Client

## Visão Geral

O UnBall SSL Client é um sistema modular para controle de robôs na RoboCup Small Size League (SSL). 
A arquitetura é baseada em componentes independentes que se comunicam entre si, permitindo 
flexibilidade e manutenção simplificada.

## Componentes Principais

### 1. Game (main.py)

O `Game` é o componente central que coordena todos os outros subsistemas. Ele:
- Inicializa e gerencia todos os subsistemas
- Executa o loop principal do jogo
- Processa comandos do árbitro
- Gerencia o estado do jogo
- Conecta todos os componentes em um fluxo de trabalho coerente

### 2. VisionClient

Responsável por receber e processar dados do sistema SSL-Vision:
- Posições e orientações dos robôs
- Posição da bola
- Geometria do campo

### 3. GameController

Interface com o SSL Game Controller:
- Recebe comandos do árbitro
- Gerencia o estado oficial do jogo
- Comunica eventos do jogo para outros componentes

### 4. PathPlanning

Implementa o algoritmo A* para navegação dos robôs:
- Calcula trajetórias evitando obstáculos
- Inflaciona obstáculos para considerar o tamanho dos robôs
- Suaviza trajetórias para movimento natural

### 5. RobotBehavior

Define comportamentos dos robôs através de máquinas de estado:
- Implementa papéis específicos (Goleiro, Defensor, Atacante)
- Reage a comandos do árbitro
- Toma decisões táticas baseadas no estado do jogo

### 6. Controladores de Robôs

Interface entre as decisões de alto nível e o hardware:
- GrSimController: Envia comandos para o simulador grSim
- IRLController: Envia comandos para robôs reais

### 7. Interface PyQt

Fornece visualização e controle para o usuário:
- Mostra o campo, robôs e bola em tempo real
- Permite envio manual de comandos
- Oferece depuração visual

## Fluxo de Dados

1. SSL-Vision → VisionClient → Game
2. SSL Game Controller → GameController → Game
3. Game → RobotStateMachines (processamento de comportamento)
4. RobotStateMachines → PathPlanner (cálculo de trajetórias)
5. RobotStateMachines + PathPlanner → Robot Controllers
6. Robot Controllers → grSim ou robôs reais

## Threads e Sincronização

O sistema utiliza múltiplas threads para operação em tempo real:
- Thread principal: Loop do jogo (processamento de estados)
- Thread de visão: Recebe dados do SSL-Vision (VisionClient)
- Thread do árbitro: Recebe comandos do Game Controller
- Thread do planejador: Calcula trajetórias A* (PathPlanner)
- Thread de controle: Envia comandos aos robôs (ThreadedRobotControlClient)
- Thread da UI: Atualiza a interface gráfica (PyQt)

Mecanismos de sincronização (locks, queues) são usados para comunicação segura entre threads.

## Configuração

A configuração é centralizada em `config.json`, permitindo ajustar:
- Parâmetros de rede (IPs, portas)
- Configurações do jogo (time, lado, divisão)
- Flags de depuração

## Inicialização do Sistema

1. Carregamento de configuração
2. Inicialização dos subsistemas
3. Conexão com visão e árbitro
4. Inicialização das máquinas de estado dos robôs
5. Início do loop principal do jogo