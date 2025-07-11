#!/usr/bin/env python3
"""
scripts/goalkeeper_demo.py

Script de demonstração do sistema de otimização do goleiro para apresentações.
Este script permite demonstração interativa do sistema em sala de aula,
incluindo visualizações em tempo real e análises comparativas.

Uso:
    python scripts/goalkeeper_demo.py --interactive
    python scripts/goalkeeper_demo.py --generate-report
    python scripts/goalkeeper_demo.py --benchmark

Autor: UnBall Team - Universidade de Brasília
"""

import os
import sys
import argparse
import time
import matplotlib.pyplot as plt
import numpy as np
from typing import Tuple, List

# Adicionar diretório raiz ao path para imports
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, ROOT_DIR)

# Imports do projeto
try:
    from GoalkeeperOptimization.goalkeeper_optimizer import create_default_optimizer
    from GoalkeeperOptimization.goalkeeper_visualization import (
        create_demo_visualization,
    )
    from GoalkeeperOptimization.goalkeeper_risk_model import create_default_risk_model
    from utils.logger import get_logger
except ImportError as e:
    print(f"Erro ao importar módulos: {e}")
    print("Certifique-se de que está executando a partir do diretório raiz do projeto.")
    sys.exit(1)

logger = get_logger("goalkeeper_demo")


class GoalkeeperDemoPresentation:
    """
    Classe principal para demonstração em sala de aula.

    Fornece interface interativa para demonstrar o sistema de otimização
    do goleiro, incluindo visualizações em tempo real e análises.
    """

    def __init__(self):
        print("Inicializando demonstração do sistema de otimização do goleiro...")
        print("=" * 60)

        # Inicializar componentes
        self.optimizer = create_default_optimizer(use_precomputed=True)
        self.visualization = create_demo_visualization(self.optimizer)
        self.risk_model = self.optimizer.risk_model

        # Configurações da demonstração
        self.demo_positions = [
            (1.0, 0.0),  # Centro do campo
            (1.5, 0.3),  # Lateral direita
            (0.8, -0.2),  # Próximo, esquerda
            (2.0, 0.4),  # Longe, direita
            (0.5, 0.0),  # Muito próximo, centro
        ]

        logger.info("GoalkeeperDemoPresentation initialized successfully")
        print("Sistema inicializado com sucesso.")
        print(
            f"Métodos de otimização disponíveis: {', '.join(self.optimizer.get_optimization_methods())}"
        )
        print("=" * 60)

    def interactive_demo(self):
        """Demonstração interativa em tempo real."""
        print("\nDEMONSTRAÇÃO INTERATIVA DO GOLEIRO")
        print("=" * 50)
        print("Esta demonstração permite posicionar a bola interativamente")
        print("e observar o posicionamento ótimo do goleiro em tempo real.")
        print()

        while True:
            try:
                print("\nOpções:")
                print("1. Posicionar bola manualmente")
                print("2. Usar posições pré-definidas")
                print("3. Demonstração automática")
                print("4. Voltar ao menu principal")

                choice = input("\nEscolha uma opção (1-4): ").strip()

                if choice == "1":
                    self._manual_ball_positioning()
                elif choice == "2":
                    self._predefined_positions_demo()
                elif choice == "3":
                    self._automatic_demo()
                elif choice == "4":
                    break
                else:
                    print("Opção inválida. Tente novamente.")

            except KeyboardInterrupt:
                print("\nDemonstração interrompida pelo usuário.")
                break
            except Exception as e:
                logger.error(f"Erro na demonstração interativa: {e}")
                print(f"Erro: {e}")

    def _manual_ball_positioning(self):
        """Permite posicionar a bola manualmente."""
        print("\nPOSICIONAMENTO MANUAL DA BOLA")
        print("-" * 40)
        print(
            f"Campo: {self.risk_model.field_dims.field_length}m x {self.risk_model.field_dims.field_width}m"
        )
        print(
            f"Limites X: [{-self.risk_model.field_dims.field_length/2:.1f}, {self.risk_model.field_dims.field_length/2:.1f}]"
        )
        print(
            f"Limites Y: [{-self.risk_model.field_dims.field_width/2:.1f}, {self.risk_model.field_dims.field_width/2:.1f}]"
        )

        try:
            x = float(input("\nPosição X da bola (metros): "))
            y = float(input("Posição Y da bola (metros): "))

            ball_pos = (x, y)

            # Validar posição
            if not self._validate_ball_position(ball_pos):
                print("Posição inválida. Deve estar dentro dos limites do campo.")
                return

            print(f"\nBola posicionada em: ({x:.2f}, {y:.2f})")

            # Calcular e mostrar resultado
            result = self.optimizer.optimize_position(ball_pos)
            self._display_optimization_result(ball_pos, result)

            # Perguntar se quer visualizar
            if input("\nDeseja ver a visualização? (s/n): ").lower().startswith("s"):
                fig = self.visualization.plot_risk_heatmap(ball_pos)
                plt.title("Mapa de Risco - Posicionamento do Goleiro")
                plt.xlabel("Posição X (m)")
                plt.ylabel("Posição Y (m)")
                plt.tight_layout()
                plt.show()

        except ValueError:
            print("Erro: Digite números válidos para as coordenadas.")
        except Exception as e:
            print(f"Erro: {e}")

    def _predefined_positions_demo(self):
        """Demonstração com posições pré-definidas."""
        print("\nDEMONSTRAÇÃO COM POSIÇÕES PRÉ-DEFINIDAS")
        print("-" * 50)

        for i, ball_pos in enumerate(self.demo_positions, 1):
            print(f"\nTeste {i}: Bola em ({ball_pos[0]:.1f}, {ball_pos[1]:.1f})")

            # Otimizar posição
            start_time = time.time()
            result = self.optimizer.optimize_position(ball_pos)
            total_time = (time.time() - start_time) * 1000

            # Mostrar resultado
            self._display_optimization_result(ball_pos, result, show_detailed=False)

            # Pausa entre demonstrações
            input("Pressione Enter para continuar...")

        # Perguntar se quer ver comparação visual
        if (
            input("\nDeseja ver comparação visual de todos os testes? (s/n): ")
            .lower()
            .startswith("s")
        ):
            fig = self.visualization.plot_optimization_comparison(self.demo_positions)
            plt.title("Comparação de Otimização - Posições Pré-definidas")
            plt.xlabel("Posição X (m)")
            plt.ylabel("Posição Y (m)")
            plt.tight_layout()
            plt.show()

    def _automatic_demo(self):
        """Demonstração automática com movimento da bola."""
        print("\nDEMONSTRAÇÃO AUTOMÁTICA")
        print("-" * 40)
        print("A bola se moverá automaticamente pelo campo.")
        print("Observe como o goleiro se reposiciona otimamente.")

        # Trajetória da bola
        trajectory = self._generate_ball_trajectory()

        print(f"\nTrajetória gerada com {len(trajectory)} pontos")
        print("Pressione Ctrl+C para parar a demonstração.")

        try:
            for i, ball_pos in enumerate(trajectory):
                print(f"\nFrame {i+1}/{len(trajectory)}")
                print(f"Bola: ({ball_pos[0]:.2f}, {ball_pos[1]:.2f})")

                # Otimizar em tempo real
                result = self.optimizer.optimize_position(ball_pos)

                print(
                    f"Goleiro ótimo: ({result.optimal_position[0]:.2f}, {result.optimal_position[1]:.2f})"
                )
                print(f"Score de risco: {result.risk_score:.3f}")
                print(f"Probabilidade de defesa: {result.defense_probability:.3f}")
                print(f"Tempo: {result.solve_time_ms:.1f}ms")

                time.sleep(1.5)  # Pausa para visualização

        except KeyboardInterrupt:
            print("\nDemonstração automática interrompida.")

    def _generate_ball_trajectory(self) -> List[Tuple[float, float]]:
        """Gera trajetória da bola para demonstração automática."""
        trajectory = []

        # Trajetória em direção ao gol
        start_x = self.risk_model.field_dims.field_length / 2 - 0.5
        end_x = 0.5

        num_points = 10

        for i in range(num_points):
            progress = i / (num_points - 1)

            # Movimento em X (linear)
            x = start_x - progress * (start_x - end_x)

            # Movimento em Y (senoidal para simular zigue-zague)
            y_amplitude = 0.4
            y = y_amplitude * np.sin(progress * np.pi * 2)

            trajectory.append((x, y))

        return trajectory

    def benchmark_demo(self):
        """Demonstração de benchmark dos métodos."""
        print("\nBENCHMARK DOS MÉTODOS DE OTIMIZAÇÃO")
        print("=" * 50)

        methods = self.optimizer.get_optimization_methods()

        if len(methods) == 1:
            print(f"Apenas um método disponível: {methods[0]}")
            return

        print(f"Métodos disponíveis: {', '.join(methods)}")
        print(f"Testando com {len(self.demo_positions)} posições...")

        # Executar benchmark
        benchmark_results = self.optimizer.benchmark_methods(self.demo_positions)

        # Mostrar resultados
        print("\nRESULTADOS DO BENCHMARK:")
        print("-" * 40)

        for method, stats in benchmark_results.items():
            print(f"\n{method.upper()}:")
            print(f"   Tempo médio: {stats['avg_time']:.2f}ms")
            print(f"   Tempo mínimo: {stats['min_time']:.2f}ms")
            print(f"   Tempo máximo: {stats['max_time']:.2f}ms")
            print(f"   Tempo total: {stats['total_time']:.2f}ms")

        # Determinar método mais rápido
        fastest_method = min(
            benchmark_results.keys(), key=lambda m: benchmark_results[m]["avg_time"]
        )

        print(f"\nMétodo mais rápido: {fastest_method}")

        # Perguntar se quer visualização
        if input("\nDeseja ver gráfico comparativo? (s/n): ").lower().startswith("s"):
            fig = self.visualization.plot_optimization_comparison(self.demo_positions)
            plt.title("Benchmark dos Métodos de Otimização")
            plt.xlabel("Posição X (m)")
            plt.ylabel("Posição Y (m)")
            plt.tight_layout()
            plt.show()

    def generate_full_report(self):
        """Gera relatório completo para apresentação."""
        print("\nGERANDO RELATÓRIO COMPLETO")
        print("=" * 50)

        output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "goalkeeper_demo_output")

        print(f"Diretório de saída: {output_dir}")
        print("Gerando visualizações...")

        try:
            self.visualization.generate_demo_report(output_dir)

            print("Relatório gerado com sucesso.")
            print(f"Arquivos salvos em: {os.path.abspath(output_dir)}")
            print("\nArquivos gerados:")

            # Listar arquivos gerados
            if os.path.exists(output_dir):
                for file in os.listdir(output_dir):
                    print(f"   {file}")

        except Exception as e:
            logger.error(f"Erro ao gerar relatório: {e}")
            print(f"Erro ao gerar relatório: {e}")

    def performance_analysis(self):
        """Análise de performance do sistema."""
        print("\nANÁLISE DE PERFORMANCE")
        print("=" * 50)

        print("Executando análise de performance...")

        # Gerar dados de teste
        num_tests = 20
        test_positions = []

        for i in range(num_tests):
            x = np.random.uniform(
                0.5, self.risk_model.field_dims.field_length / 2 - 0.5
            )
            y = np.random.uniform(
                -self.risk_model.field_dims.field_width / 2 + 0.5,
                self.risk_model.field_dims.field_width / 2 - 0.5,
            )
            test_positions.append((x, y))

        # Coletar estatísticas
        solve_times = []
        risk_scores = []
        defense_probs = []

        print(f"Testando {num_tests} posições aleatórias...")

        for i, ball_pos in enumerate(test_positions):
            result = self.optimizer.optimize_position(ball_pos)
            solve_times.append(result.solve_time_ms)
            risk_scores.append(result.risk_score)
            defense_probs.append(result.defense_probability)

            if (i + 1) % 5 == 0:
                print(f"   {i + 1}/{num_tests} testes concluídos")

        # Mostrar estatísticas
        print("\nESTATÍSTICAS DE PERFORMANCE:")
        print("-" * 40)
        print(f"Tempo médio de execução: {np.mean(solve_times):.2f}ms")
        print(f"Tempo mínimo: {np.min(solve_times):.2f}ms")
        print(f"Tempo máximo: {np.max(solve_times):.2f}ms")
        print(f"Desvio padrão do tempo: {np.std(solve_times):.2f}ms")
        print()
        print(f"Score de risco médio: {np.mean(risk_scores):.4f}")
        print(f"Probabilidade de defesa média: {np.mean(defense_probs):.4f}")
        print(
            f"Eficiência média: {np.mean(defense_probs) / (np.mean(solve_times) / 1000):.1f} prob/s"
        )

        # Perguntar se quer visualização
        if input("\nDeseja ver análise visual? (s/n): ").lower().startswith("s"):
            fig = self.visualization.plot_performance_analysis()
            plt.title("Análise de Performance do Sistema")
            plt.xlabel("Posição X (m)")
            plt.ylabel("Posição Y (m)")
            plt.tight_layout()
            plt.show()

    def _validate_ball_position(self, ball_pos: Tuple[float, float]) -> bool:
        """Valida se a posição da bola está dentro dos limites do campo."""
        x, y = ball_pos
        dims = self.risk_model.field_dims

        return (
            -dims.field_length / 2 <= x <= dims.field_length / 2
            and -dims.field_width / 2 <= y <= dims.field_width / 2
        )

    def _display_optimization_result(
        self, ball_pos: Tuple[float, float], result, show_detailed: bool = True
    ):
        """Exibe resultado da otimização de forma formatada."""
        if show_detailed:
            print("\nRESULTADO DA OTIMIZAÇÃO:")
            print("-" * 30)

        print(
            f"Posição ótima do goleiro: ({result.optimal_position[0]:.3f}, {result.optimal_position[1]:.3f})"
        )
        print(f"Score de risco: {result.risk_score:.4f}")
        print(f"Probabilidade de defesa: {result.defense_probability:.4f}")
        print(f"Tempo de cálculo: {result.solve_time_ms:.2f}ms")
        print(f"Método utilizado: {result.solver_status}")

        if show_detailed:
            # Análise adicional
            risk_level = (
                "BAIXO"
                if result.risk_score < 0.3
                else "MÉDIO" if result.risk_score < 0.7 else "ALTO"
            )
            prob_level = (
                "ALTA"
                if result.defense_probability > 0.7
                else "MÉDIA" if result.defense_probability > 0.3 else "BAIXA"
            )

            print(f"\nANÁLISE:")
            print(f"   Nível de risco: {risk_level}")
            print(f"   Chance de defesa: {prob_level}")

            # Distância da bola ao gol
            goal_center = self.risk_model.goal_center
            distance = np.sqrt(
                (ball_pos[0] - goal_center[0]) ** 2
                + (ball_pos[1] - goal_center[1]) ** 2
            )
            print(f"   Distância bola-gol: {distance:.2f}m")


def main():
    """Função principal do script de demonstração."""
    parser = argparse.ArgumentParser(
        description="Demonstração do Sistema de Otimização do Goleiro - UnBall",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemplos de uso:
    python goalkeeper_demo.py --interactive     # Demonstração interativa
    python goalkeeper_demo.py --benchmark       # Benchmark dos métodos
    python goalkeeper_demo.py --report          # Gerar relatório completo
    python goalkeeper_demo.py --performance     # Análise de performance
    python goalkeeper_demo.py --quick-demo      # Demonstração rápida
        """,
    )

    parser.add_argument(
        "--interactive", action="store_true", help="Executar demonstração interativa"
    )
    parser.add_argument(
        "--benchmark",
        action="store_true",
        help="Executar benchmark dos métodos de otimização",
    )
    parser.add_argument(
        "--report",
        action="store_true",
        help="Gerar relatório completo com visualizações",
    )
    parser.add_argument(
        "--performance", action="store_true", help="Executar análise de performance"
    )
    parser.add_argument(
        "--quick-demo",
        action="store_true",
        help="Demonstração rápida com posições pré-definidas",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Saída detalhada (verbose)"
    )

    args = parser.parse_args()

    # Configurar nível de logging
    if args.verbose:
        import logging

        logging.getLogger().setLevel(logging.DEBUG)

    try:
        # Inicializar demonstração
        demo = GoalkeeperDemoPresentation()

        # Executar ação solicitada
        if args.interactive:
            demo.interactive_demo()
        elif args.benchmark:
            demo.benchmark_demo()
        elif args.report:
            demo.generate_full_report()
        elif args.performance:
            demo.performance_analysis()
        elif args.quick_demo:
            demo._predefined_positions_demo()
        else:
            # Menu principal se nenhuma opção específica
            main_menu(demo)

    except KeyboardInterrupt:
        print("\nDemonstração interrompida pelo usuário.")
        print("Obrigado por usar o sistema de demonstração.")
    except Exception as e:
        logger.error(f"Erro fatal na demonstração: {e}")
        print(f"Erro fatal: {e}")
        sys.exit(1)


def main_menu(demo: GoalkeeperDemoPresentation):
    """Menu principal interativo."""
    print("\nSISTEMA DE DEMONSTRAÇÃO DO GOLEIRO - UnBall")
    print("Universidade de Brasília")
    print("=" * 60)

    while True:
        print("\nMENU PRINCIPAL:")
        print("1. Demonstração Interativa")
        print("2. Benchmark dos Métodos")
        print("3. Gerar Relatório Completo")
        print("4. Análise de Performance")
        print("5. Demonstração Rápida")
        print("6. Informações do Sistema")
        print("7. Sair")

        try:
            choice = input("\nEscolha uma opção (1-7): ").strip()

            if choice == "1":
                demo.interactive_demo()
            elif choice == "2":
                demo.benchmark_demo()
            elif choice == "3":
                demo.generate_full_report()
            elif choice == "4":
                demo.performance_analysis()
            elif choice == "5":
                demo._predefined_positions_demo()
            elif choice == "6":
                show_system_info(demo)
            elif choice == "7":
                print("\nObrigado por usar o sistema de demonstração.")
                break
            else:
                print("Opção inválida. Escolha um número de 1 a 7.")

        except KeyboardInterrupt:
            print("\nSaindo do sistema...")
            break
        except Exception as e:
            logger.error(f"Erro no menu principal: {e}")
            print(f"Erro: {e}")


def show_system_info(demo: GoalkeeperDemoPresentation):
    """Mostra informações detalhadas do sistema."""
    print("\nINFORMAÇÕES DO SISTEMA")
    print("=" * 50)

    # Informações do campo
    dims = demo.risk_model.field_dims
    print("CONFIGURAÇÃO DO CAMPO:")
    print(f"   Dimensões: {dims.field_length}m x {dims.field_width}m")
    print(f"   Largura do gol: {dims.goal_width}m")
    print(
        f"   Área de defesa: {dims.defense_area_depth}m x {dims.defense_area_width}m"
    )

    # Informações do goleiro
    constraints = demo.risk_model.constraints
    print("\nCONFIGURAÇÃO DO GOLEIRO:")
    print(f"   Velocidade máxima: {constraints.max_speed}m/s")
    print(f"   Tempo de reação: {constraints.reaction_time}s")
    print(f"   Raio do robô: {constraints.robot_radius}m")
    print(f"   Distância segura do gol: {constraints.safe_distance_from_goal_line}m")

    # Informações do otimizador
    methods = demo.optimizer.get_optimization_methods()
    print(f"\nMÉTODOS DE OTIMIZAÇÃO:")
    for method in methods:
        print(f"   {method}")

    # Estatísticas do cache
    cache_stats = demo.optimizer.get_cache_stats()
    print(f"\nCACHE DE OTIMIZAÇÃO:")
    print(f"   Entradas no cache: {cache_stats['cache_size']}")

    # Informações da matriz pré-computada
    if demo.optimizer._risk_matrix:
        grid = demo.optimizer._position_grid
        total_positions = len(grid["ball_x"]) * len(grid["ball_y"]) * len(grid["gk_y"])
        print(f"\nMATRIZ PRÉ-COMPUTADA:")
        print(f"   Posições da bola: {len(grid['ball_x'])} x {len(grid['ball_y'])}")
        print(f"   Posições do goleiro: {len(grid['gk_y'])}")
        print(f"   Total de combinações: {total_positions:,}")

    print("\n" + "=" * 50)
    input("Pressione Enter para continuar...")


def quick_test():
    """Teste rápido para verificar se o sistema está funcionando."""
    print("Executando teste rápido do sistema...")

    try:
        # Criar componentes
        optimizer = create_default_optimizer(
            use_precomputed=False
        )  # Sem pré-computação para teste rápido

        # Testar uma posição
        ball_pos = (1.0, 0.0)
        result = optimizer.optimize_position(ball_pos)

        print("Teste concluído com sucesso.")
        print(f"   Bola em: {ball_pos}")
        print(f"   Goleiro ótimo: {result.optimal_position}")
        print(f"   Tempo: {result.solve_time_ms:.1f}ms")

        return True

    except Exception as e:
        print(f"Erro no teste: {e}")
        return False


if __name__ == "__main__":
    # Banner inicial
    print("=" * 60)
    print("SISTEMA DE OTIMIZAÇÃO DO GOLEIRO - UnBall")
    print("Universidade de Brasília")
    print("RoboCup Small Size League")
    print("=" * 60)

    # Verificar se é chamada com --test para teste rápido
    if len(sys.argv) > 1 and "--test" in sys.argv:
        quick_test()
    else:
        main()
