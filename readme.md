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

- 🎥 **SSL-Vision**: Recebe e processa dados de visão (posições de robôs, bola, geometria do campo)
- 🎮 **SSL Game Controller**: Manipula comandos do árbitro e o estado do jogo
- 🤖 **Controle de Robôs**: Gerencia ações de robôs via simulação (grSim) ou interfaces para robôs reais (IRL)
- 🧠 **Lógica Comportamental**: Implementa máquinas de estado para diferentes papéis de robôs (Goleiro, Defensor, Atacante)
- 🗺️ **Planejamento de Trajetória**: Utiliza o algoritmo A* para navegação de robôs e desvio de obstáculos
- 🖥️ **Interface Gráfica**: GUI baseada em PyQt5 para visualização em tempo real, controle e depuração
- 📊 **Sistema de Logging**: Sistema centralizado de logs com níveis configuráveis para debug eficiente
- 🐛 **Utilitários de Debug**: Ferramentas avançadas para análise de performance e comportamento

## 📦 Dependências e Instalação

Este projeto é compatível com **Python 3.10.12**.

Para criar um ambiente virtual e instalar as dependências:

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

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           ENTRADA DE DADOS                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│  SSL-Vision ──┐                                                             │
│               │ UDP Multicast                                               │
│  SSL Game     ├──────────────► Game Logic (Main Loop)                       │
│  Controller ──┘                        │                                    │
│                                         │                                   │
│  config.json ───────────────────────────┘                                   │
└─────────────────────────────────────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      PROCESSAMENTO CENTRAL                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│  Robot State Machines (RobotBehavior)                                       │
│                        │                                                    │
│                        ▼                                                    │
│  Path Planner (PathPlanning - A*)                                           │
│                        │                                                    │
│                        ▼                                                    │
│  Robot Controllers (RobotBehavior - GrSim/IRL)                              │
│                        │                                                    │
│                        ▼                                                    │
│  Logger & Debug Utils (utils/)                                              │
└─────────────────────────────────────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      SAÍDA E INTERAÇÃO                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│  Robot Controllers ──┬──► grSim Simulator                                   │
│                      │                                                      │
│                      └──► Robôs Reais (IRL)                                 │
│                                                                             │
│  Game Logic ─────────────► Interface Gráfica (PyQt)                         │
│                      │                                                      │
│  Logger ─────────────┴──► Arquivos de Log / Console                         │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Threads do Sistema

O sistema utiliza múltiplas threads para garantir processamento em tempo real:

1. **Thread Principal (Game)**: Loop principal a 60 FPS
2. **Thread Vision**: Recebe pacotes UDP do SSL-Vision
3. **Thread GameController**: Recebe comandos do árbitro
4. **Thread UI (PyQt)**: Interface gráfica responsiva
5. **Threads de Controle**: Uma thread por robô para envio de comandos

**Sincronização**: Utiliza `threading.Lock()` para acesso seguro aos dados compartilhados.

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
  },
  "logging": {
    "level": "INFO",
    "file_output": true,
    "console_output": true,
    "max_file_size": "10MB",
    "backup_count": 5
  }
}
```

## 🧩 Componentes Detalhados

### 1. main.py - Núcleo do Sistema

- **Classe Principal:** `Game`
- **Responsabilidade:** Coordena visão, árbitro, controle de robôs, planejamento de trajetória e UI
- **Destaques:** 
  - Loop principal a 60 FPS
  - Gerenciamento de configuração
  - Inicialização dinâmica de robôs
  - Manipulação de comandos do árbitro
  - Sistema de logging integrado

### 2. Sistema de Logging (utils/logger.py)

- **Função:** Sistema centralizado de logging com níveis configuráveis
- **Características:**
  - Níveis: DEBUG, INFO, WARNING, ERROR, CRITICAL
  - Output para console e arquivo
  - Rotação automática de logs
  - Formatação consistente com timestamps
  
**Uso:**
```python
from utils.logger import get_logger

logger = get_logger("module_name")
logger.debug("Mensagem de debug")
logger.info("Informação importante")
logger.error("Erro ocorreu", exc_info=True)
```

### 3. Utilitários de Debug (utils/debug_utils.py)

- **Performance Monitor**: Análise de tempo de execução
- **Memory Profiler**: Monitoramento de uso de memória
- **Thread Monitor**: Visualização de threads ativas
- **Data Inspector**: Inspeção de estruturas de dados em tempo real

## 🎮 Como Usar

### Interface Gráfica

#### Controles Principais
- **Seleção de Time:** Escolha entre "Time Azul" ou "Time Amarelo"
- **Seleção de Divisão:** "Entry Level", "Division B", ou "Division A"
- **Modo de Controle:** "grSim" (simulação) ou "IRL" (robôs reais)

#### Comandos de Jogo
- **Comandos Básicos:** HALT, STOP, FORCE START
- **Situações de Jogo:** (Em desenvolvimento)
  - FREE-KICK POSITION
  - KICK-OFF
  - PENALTY
  - GOAL KICK
  - CORNER KICK
  - BALL PLACEMENT

#### Visualização e Debug
- **Campo Central:** Posições em tempo real dos robôs e bola
- **Trajetórias A*:** Visualização de caminhos planejados
- **Status dos Robôs:** Papel e estado atual
- **Console de Debug:** Logs em tempo real (Menu > Debug)

### Comandos via Terminal

```bash
# Executar o cliente principal
python main.py

# Testes específicos
python -m tests.integration.test_behavior
python -m tests.system.test_cli --role goalkeeper --duration 30

# Debug com níveis específicos
LOG_LEVEL=DEBUG python main.py

# Análise de performance
python -m utils.performance_analyzer
```

## 🤖 Sistema de IA e Comportamentos

### Estados dos Robôs

Cada robô utiliza uma máquina de estados finitos (FSM):

#### Estados Básicos
- **IDLE**: Robô parado
- **MOVING_TO_POSITION**: Movendo-se para um alvo
- **RETURNING**: Retornando à posição inicial

#### Estados Específicos por Papel

**Atacante:**
- **MOVING_TO_BALL**: Indo em direção à bola
- **APPROACHING_BALL**: Aproximação final (em desenvolvimento)
- **ALIGNING_TO_GOAL**: Alinhamento para chute (em desenvolvimento)
- **KICKING**: Executando chute

**Goleiro:**
- **DEFENDING_GOAL**: Posicionamento defensivo
- **INTERCEPTING**: Tentando interceptar a bola
- **CLEARING_BALL**: Afastando a bola da área

**Defensor:**
- **MARKING**: Marcando adversário
- **BLOCKING**: Bloqueando linha de passe
- **SUPPORTING**: Apoiando o ataque

### Sistema de Planejamento (A*)

O planejador de trajetória considera:
- Obstáculos estáticos (limites do campo)
- Obstáculos dinâmicos (outros robôs)
- Zonas proibidas (área do goleiro adversário)
- Otimização de caminho (suavização de curvas)

## 🐛 Debug e Monitoramento

### Níveis de Log

```python
# Em config.json
"logging": {
  "level": "DEBUG",  # DEBUG, INFO, WARNING, ERROR, CRITICAL
  "modules": {
    "vision": "INFO",
    "path_planning": "DEBUG",
    "robot_behavior": "DEBUG"
  }
}
```

### Ferramentas de Debug

1. **Performance Profiler**
   ```bash
   python -m utils.profiler --module path_planning
   ```

2. **Thread Monitor**
   ```bash
   python -m utils.thread_monitor
   ```

3. **Data Flow Analyzer**
   ```bash
   python -m utils.data_flow_analyzer
   ```

## 🧪 Testes

### Estrutura de Testes

```
tests/
├── unit/           # Testes unitários
├── integration/    # Testes de integração
├── system/        # Testes de sistema
└── performance/   # Testes de performance
```

### Executar Testes

```bash
# Todos os testes
python -m pytest

# Testes específicos
python -m pytest tests/unit/test_path_planning.py
python -m pytest tests/integration/test_robot_behavior.py -v

# Com cobertura
python -m pytest --cov=. --cov-report=html
```

## 📚 Recursos de Aprendizado

### Para Iniciantes em RoboCup SSL

1. **Documentação Oficial SSL**
   - [SSL Rules](https://ssl.robocup.org/rules/)
   - [SSL Vision Protocol](https://github.com/RoboCup-SSL/ssl-vision/wiki)

2. **Artigos Recomendados**
   - "Multi-Robot Path Planning in Dynamic Environments" (2023)
   - "Coordinated Team Play in Robot Soccer" (2022)
   - "Real-time Motion Planning for SSL Robots" (2021)

3. **Conceitos Importantes**
   - **Threads em Python**: `threading` vs `multiprocessing`
   - **Comunicação UDP**: Sockets e multicast
   - **Máquinas de Estado**: Design patterns para robótica
   - **Algoritmo A***: Pathfinding em grids

### Dicas de Desenvolvimento

1. **Use o Logger, não print()**
   ```python
   # ❌ Evite
   print(f"Debug: {value}")
   
   # ✅ Prefira
   self.logger.debug(f"Value: {value}")
   ```

2. **Thread Safety**
   ```python
   with self.data_lock:
       # Acesso seguro a dados compartilhados
       self.shared_data = new_value
   ```

3. **Tratamento de Exceções**
   ```python
   try:
       risky_operation()
   except SpecificException as e:
       self.logger.error(f"Erro específico: {e}", exc_info=True)
   ```

## 🚧 Status de Desenvolvimento

### ✅ Implementado
- Sistema base de visão e controle
- Planejamento de trajetória A*
- Interface gráfica básica
- Sistema de logging
- Estrutura de máquinas de estado

### 🔄 Em Desenvolvimento
- Melhorias na aproximação da bola
- Lógica avançada do goleiro
- Comportamento defensivo inteligente
- Botões de situações de jogo na UI

### 📋 Planejado
- Sistema de táticas coletivas
- Machine Learning para predição
- Análise pós-jogo
- Modo de treinamento

## 🤝 Contribuindo

1. Fork o projeto
2. Crie sua feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit suas mudanças (`git commit -m 'Add: AmazingFeature'`)
4. Push para a branch (`git push origin feature/AmazingFeature`)
5. Abra um Pull Request

## Suporte

- **Issues**: Use o GitHub Issues para reportar bugs
- **Discussões**: GitHub Discussions para dúvidas
- **Email**: unball@unb.br
