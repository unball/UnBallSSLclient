todo:

# importante

- performance
- reduzir o número de debugs no terminal/arrumar as flags
- arrumar todos os estados do jogo
- talvez criar lógica de estados dos robôs para o jogo 
- ângulos 
- corrigir a maneira que o robô se aproxima da bola
- a classe pathfollower deveria realmente estar aí?
- acho que estou criando muitas variáveis específicas pro ssl-el fazer a flexibilidade pra transferência entre categorias de maneira assídua, evitando variáveis locais tanto em main.py quanto em robot_state_machine.py
- migrar `print()` para `logger`
    - substituir todas as chamadas `print()` usadas para depuração pelo sistema de logging centralizado (`utils/logger.py`)
    - arquivos-alvo: `RobotBehavior/robot_state_machine.py`, `GameController/GameController.py`, `PyQt/ssl_client.py`
    - exemplo:
        ```python
        # Em RobotBehavior/robot_state_machine.py
        # Antes:
        # print(f"Robô {self.robot_id} movendo para {target_pos}")

        # Depois:
        # (no __init__): self.logger = get_logger(f"robot_behavior.sm.{self.robot_id}")
        # (no método): self.logger.debug(f"Movendo para {target_pos}")
        ```
  - melhorar a aproximação e posse de bola do atacante
    - o robô atacante consegue planejar o caminho até a bola, mas a aproximação final e a "tomada de posse" são ineficazes
    - arquivos-alvo: `RobotBehavior/robot_state_machine.py` (dentro da classe `AttackerStateMachine`)
    - sugestão de abordagem:
        1. no estado `MOVING_TO_BALL`, calcular um "ponto de aproximação" atrás da bola, já alinhado com o gol adversário
        2. criar um novo estado `AIMING_TO_GOAL` para pequenos ajustes de orientação
        3. condição para chutar (`kick_flat`) deve ser mais rigorosa
    - prioridade: altíssima
- refatorar lógica do goleiro
    - o goleiro atual apenas segue a bola no eixo Y (nem isso direito). precisa ser mais inteligente
    - arquivos-alvo: `RobotBehavior/robot_state_machine.py` (classe `GoalkeeperStateMachine`)
    - calcular a interseção da linha projetada da bola até o centro do gol?
    - ser agressivo e chutar quando a bola ta no campo
    - prioridade: alta
- refatorar lógica do defensor
    - o defensor não está atuando de forma eficaz
    - arquivos-alvo: `RobotBehavior/robot_state_machine.py` (classe `DefenderStateMachine`)
    - sugestão: implementar lógica de "sombreamento"
    - prioridade: média
- implementar botões da UI
    - conectar os botões da interface que atualmente não possuem funcionalidade
    - arquivos-alvo: `PyQt/ssl_client.py`
    - prioridade: alta
- o a* tá considerando os robôs amarelos na hora do chute? o a* está considerando os robôs no path pllaning? 

# importante mas nem tanto

- revisar todo o código e organizar (pode ser feito por ai)
- revisar todo o código e comentar (pode ser feito por ai)
- melhorar os unit tests e arrumá-los
- pesquisar e ler sobre táticas de futebol de robôs
    - buscar artigos e teses sobre estratégias usadas na RoboCup SSL
    - tópicos: "Coordinated Multi-Agent Path Planning", "RoboCup SSL Strategy", "Dynamic Role Assignment in Robot Soccer"
    - prioridade: contínua

# ui e etc

- melhor atualizar a tela quando nenhum dado de visão é recebido
- colocar o nome dos times no Team A e Team B
- mudar o tamanho da janela de acordo com o botão maximizar
- corrigir outras abas (debug e posicionamento)
- IRL
- Quando muda a resolução o pyqt não faz update conforme o necessário