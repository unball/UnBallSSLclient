# UnBall SSL Client

<div align="center">

![Python](https://img.shields.io/badge/python-3.10+-blue.svg)
![RoboCup](https://img.shields.io/badge/RoboCup-SSL-red.svg)
![UnB](https://img.shields.io/badge/UnB-UnBall-yellow.svg)

**Cliente SSL completo para RoboCup SSL**

*Desenvolvido pela equipe UnBall da Universidade de Brasília*

</div>

## 📋 Visão Geral (Overview)

O **UnBall SSL Client** é um sistema modular para controle de robôs na **RoboCup Small Size League (SSL)**. O projeto implementa uma arquitetura baseada em threads para comunicação em tempo real com:

- 🎥 **SSL-Vision**: Recebe e processa dados de visão (posições de robôs, bola, geometria do campo).
- 🎮 **SSL Game Controller**: Manipula comandos do árbitro e o estado do jogo.
- 🤖 **Controle de Robôs**: Gerencia ações de robôs via simulação (grSim) ou interfaces para robôs reais (IRL).
- 🧠 **Lógica Comportamental**: Implementa máquinas de estado para diferentes papéis de robôs (Goleiro, Defensor, Atacante).
- 🗺️ **Planejamento de Trajetória**: Utiliza o algoritmo A* para navegação de robôs e desvio de obstáculos.
- 🖥️ **Interface Gráfica**: GUI baseada em PyQt5 para visualização em tempo real, controle e depuração.

## 📦 Dependências e Instalação

Este projeto é compatível com **Python 3.10.12**.

Para criar um ambiente virtual e instalar as dependências, execute:

```bash
python3.10 -m venv env
source env/bin/activate
pip install -r requirements.txt
pip install -e .
```

Para instalar as ferramentas Qt necessárias para a interface gráfica:

```bash
sudo apt-get install qttools5-dev
```

## 🏗️ Arquitetura do Sistema

<!--
The following diagram is in Mermaid syntax. 
If your Markdown renderer does not support Mermaid, please use a tool like https://mermaid.live/ to view it.
-->

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           ENTRADA DE DADOS                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│  SSL-Vision ──┐                                                            │
│               │ UDP Multicast                                               │
│  SSL Game     ├──────────────► Game Logic (Main Loop)                      │
│  Controller ──┘                        │                                   │
│                                         │                                   │
│  config.json ───────────────────────────┘                                   │
└─────────────────────────────────────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      PROCESSAMENTO CENTRAL                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│  Robot State Machines (RobotBehavior)                                      │
│                        │                                                    │
│                        ▼                                                    │
│  Path Planner (PathPlanning - A*)                                         │
│                        │                                                    │
│                        ▼                                                    │
│  Robot Controllers (RobotBehavior - GrSim/IRL)                            │
└─────────────────────────────────────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      SAÍDA E INTERAÇÃO                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│  Robot Controllers ──┬──► grSim Simulator                                  │
│                      │                                                     │
│                      └──► Robôs Reais (IRL)                               │
│                                                                             │
│  Game Logic ─────────────► Interface Gráfica (PyQt)                       │
└─────────────────────────────────────────────────────────────────────────────┘
```


**Fluxo de Dados:**

- SSL-Vision e SSL Game Controller transmitem pacotes via multicast UDP.
- VisionClient e GameController capturam e processam esses pacotes.
- A classe principal `Game` (em main.py) orquestra o fluxo, recebendo os dados processados.
- Máquinas de estado dos robôs usam dados de visão e árbitro para decidir ações.
- O planejador de trajetória (A*) calcula rotas considerando obstáculos.
- Controladores de robôs traduzem comandos para grSim ou robôs reais.
- A interface PyQt fornece visualização, controle manual e depuração.

A configuração principal é gerenciada através do arquivo config.json localizado na raiz do projeto. Se este arquivo não for encontrado, uma configuração padrão será criada. As configurações de rede podem ser modificadas em tempo de execução através da UI (Menu > Configurações > Configurar IPS).

### Exemplo config.json

```json
{
  "network": {
    "multicast_ip": "224.5.23.2",
    "vision_port": 10020,
    "referee_ip": "224.5.23.1",
    "referee_port": 10003,
    "yellow_port": 10004,
    "blue_port": 10005,
    "blue_control_port": 10301,
    "yellow_control_port": 10302
  },
  "match": {
    "team_1": "UnBall",
    "team_2": "Opponent",
    "event": "Test",
    "team_side": "left",
    "team_color": "blue",
    "division": "Entry Level",
    "num_robots_our_team": 3,
    "time_logging": false,
    "control_mode": "grSim",
    "fps": 60
  },
  "debug_flags": {
    "vision": false,
    "referee": false,
    "threads": false,
    "timing": false,
    "path_planning": true,
    "robot_behavior": true,
    "all": false
  }
}
```

### Divisões Suportadas

O cliente suporta diferentes divisões da SSL, com configurações específicas para dimensões de campo, número de robôs e papéis padrão (definidas em `FIELD_DIMENSIONS_BY_DIVISION` em main.py):

| Divisão      | Campo (LxA)    | Robôs/time | Papéis Padrão                |
|--------------|---------------|------------|------------------------------|
| Entry Level  | 4.5m × 3.0m   | 3          | {0: GOLEIRO, 1: DEFENSOR, 2: ATACANTE} |
| Division B   | 9.0m × 6.0m   | 6          | {0: GOLEIRO, 1: DEFENSOR, 2: ATACANTE, ...} |
| Division A   | 12.0m × 9.0m  | 11         | {0: GOLEIRO, 1: DEFENSOR, 2: DEFENSOR, ...} |

A seleção da divisão na UI atualiza a visualização do campo e o número de robôs.

## 🧩 Componentes Detalhados

### 1. main.py - Núcleo do Sistema

- **Classe Principal:** `Game`
- **Responsabilidade:** Coordena visão, árbitro, controle de robôs, planejamento de trajetória e UI.
- **Destaques:** Loop principal a 60 FPS, gerenciamento de configuração, inicialização dinâmica de robôs, manipulação de comandos do árbitro e UI.

### 2. VisionClient - Processamento de Visão

- **Arquivo:** VisionClient/Vision.py
- **Classe:** Vision (threading.Thread)
- **Função:** Recebe e processa pacotes do SSL-Vision, atualizando posições de robôs e bola.

### 3. GameController - Comunicação com Árbitro

- **Arquivo:** GameController/GameController.py
- **Classe:** GameController (threading.Thread)
- **Função:** Recebe comandos do árbitro, atualiza estado do jogo.

### 4. RobotBehavior - Comportamentos dos Robôs

- **Arquivos:** robot_states.py, robot_state_machine.py
- **Função:** Define estados e papéis dos robôs (GOLEIRO, DEFENSOR, ATACANTE), lógica de transição de estados.

### 5. PathPlanning - Planejamento de Trajetória

- **Arquivos:** path_planner.py, astar.py
- **Função:** Algoritmo A* para navegação, desvio de obstáculos, suavização de trajetórias.

### 6. SimulationGrSim / RobotBehavior (Controladores)

- **Função:** Controladores para simulação (grSim) e robôs reais (IRL).

### 7. PyQt Interface - Interface Gráfica

- **Arquivos:** ssl_client.py, field_visualization.py
- **Função:** Visualização do campo, controle manual, depuração.

## 🎮 Como Usar

### Interface Gráfica

- **Seleção de Time:** Escolha entre "Time Azul" ou "Time Amarelo" (comboBox).
- **Seleção de Divisão:** Selecione "Entry Level", "Division B", ou "Division A" (division_combo).
- **Modo de Controle:** Escolha entre "grSim" (simulação) ou "IRL" (robôs reais).
- **Comandos do Árbitro:** Use botões como "HALT", "STOP", "FORCE START" para enviar comandos de jogo.
- **Visualização:** O campo central mostra posições em tempo real dos robôs e da bola. Opções para alternar visibilidade dos times e visualizar trajetórias A*.
- **Status dos Robôs:** Exibe papel e estado atual de cada robô.
- **Seleção de Estado do Jogo:** Permite forçar o sistema a um estado de jogo específico.

### Comandos via Terminal

Para executar testes ou utilitários:

```bash
python -m tests.integration.test_behavior
python -m tests.system.test_cli --role goalkeeper --duration 30
```

## 🧪 Testes e Qualidade

Consulte `tests/tests.md` para comandos de execução de testes.

Exemplo:

```bash
python -m unittest discover tests/unit
python -m tests.integration.test_behavior
```

## 🐛 Debug e Monitoramento

- **Logs:** Utiliza o módulo logging do Python. Ative logs detalhados via `GAME_DEBUG = True` em main.py ou flags em config.json.
- **Interface de Debug:** Menu > Configurações > Debug.
- **Visualizar A\*:** Caixa de seleção na UI principal.

## 🤖 Sistema de IA e Comportamentos

Cada robô utiliza uma máquina de estados para determinar seu comportamento. Principais estados:

- IDLE: Robô parado.
- MOVING_TO_POSITION: Movendo-se para um alvo.
- RETURNING: Retornando à posição inicial.
- AVOIDING_BALL: Afastando-se da bola.
- MOVING_TO_BALL: Indo em direção à bola.
- BALL_PLACEMENT_ACTIVE / BALL_PLACEMENT_AVOIDING: Lidando com posicionamento de bola.

## 🔗 Recursos Relacionados

O UnBall SSL Client integra-se com diversas ferramentas e protocolos do ecossistema RoboCup SSL:

- [ssl-vision](https://github.com/RoboCup-SSL/ssl-vision): Sistema de visão para rastreamento de robôs e bola.
- [ssl-game-controller](https://github.com/RoboCup-SSL/ssl-game-controller): Controlador de partidas e árbitro.
- [ssl-simulation-protocol](https://github.com/RoboCup-SSL/ssl-simulation-protocol): Protocolo de simulação.
- [ssl-simulation-setup](https://github.com/RoboCup-SSL/ssl-simulation-setup): Ferramentas de configuração de simulação.
- [grSim](https://github.com/RoboCup-SSL/grSim): Simulador de jogos SSL.
- [erForceSim](https://github.com/robotics-erlangen/framework): Framework de simulação alternativo.
