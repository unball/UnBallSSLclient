"""
GoalkeeperOptimization/goalkeeper_visualization.py

Sistema de visualização para demonstração do otimizador do goleiro.
Este módulo cria mapas de calor, gráficos e demonstrações visuais
para apresentações em sala de aula.

Autor: UnBall Team - Universidade de Brasília
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from typing import Tuple, Dict, List, Optional
import seaborn as sns
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.animation import FuncAnimation
import os
import sys

from GoalkeeperOptimization.goalkeeper_optimizer import (
    GoalkeeperOptimizer,
)

from utils.logger import get_logger

# Adiciona a raiz do projeto ao sys.path para permitir imports absolutos
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

logger = get_logger("goalkeeper_visualization")


class GoalkeeperVisualization:
    """
    Classe principal para visualização do sistema de otimização do goleiro.

    Fornece ferramentas para criar:
    - Mapas de calor de risco
    - Gráficos de posicionamento ótimo
    - Animações de movimento do goleiro
    - Comparações de métodos de otimização
    """

    def __init__(self, optimizer: GoalkeeperOptimizer):
        self.optimizer = optimizer
        self.risk_model = optimizer.risk_model
        self.field_dims = self.risk_model.field_dims
        self.logger = logger

        # Configurações de visualização
        self.figure_size = (12, 8)
        self.dpi = 100

        # Cores personalizadas
        self.colors = {
            "field": "#228B22",  # Verde do campo
            "lines": "white",  # Linhas do campo
            "goal": "#654321",  # Gol (marrom)
            "ball": "#FFD700",  # Bola (dourado)
            "goalkeeper_optimal": "#FF0000",  # Goleiro posição ótima (vermelho)
            "goalkeeper_current": "#0000FF",  # Goleiro posição atual (azul)
            "risk_high": "#FF0000",  # Risco alto (vermelho)
            "risk_low": "#00FF00",  # Risco baixo (verde)
        }

        # Setup do matplotlib
        plt.style.use("default")
        sns.set_palette("husl")

        logger.info("GoalkeeperVisualization initialized")

    def create_field_plot(self, ax=None) -> plt.Axes:
        """
        Cria plot básico do campo de futebol robótico.

        Args:
            ax: Axes existente (opcional)

        Returns:
            Axes configurado com o campo
        """
        if ax is None:
            fig, ax = plt.subplots(figsize=self.figure_size, dpi=self.dpi)

        # Dimensões do campo
        field_length = self.field_dims.field_length
        field_width = self.field_dims.field_width

        # Campo principal
        field_rect = patches.Rectangle(
            (-field_length / 2, -field_width / 2),
            field_length,
            field_width,
            linewidth=2,
            edgecolor=self.colors["lines"],
            facecolor=self.colors["field"],
            alpha=0.8,
        )
        ax.add_patch(field_rect)

        # Linha central
        ax.axvline(x=0, color=self.colors["lines"], linewidth=2)

        # Círculo central
        center_circle = patches.Circle(
            (0, 0),
            0.5,  # Raio do círculo central
            linewidth=2,
            edgecolor=self.colors["lines"],
            facecolor="none",
        )
        ax.add_patch(center_circle)

        # Gols
        goal_width = self.field_dims.goal_width
        goal_depth = self.field_dims.goal_depth

        # Gol esquerdo (nosso gol)
        left_goal = patches.Rectangle(
            (-field_length / 2 - goal_depth, -goal_width / 2),
            goal_depth,
            goal_width,
            linewidth=2,
            edgecolor=self.colors["lines"],
            facecolor=self.colors["goal"],
            alpha=0.7,
        )
        ax.add_patch(left_goal)

        # Gol direito
        right_goal = patches.Rectangle(
            (field_length / 2, -goal_width / 2),
            goal_depth,
            goal_width,
            linewidth=2,
            edgecolor=self.colors["lines"],
            facecolor=self.colors["goal"],
            alpha=0.7,
        )
        ax.add_patch(right_goal)

        # Áreas de defesa
        defense_width = self.field_dims.defense_area_width
        defense_depth = self.field_dims.defense_area_depth

        # Área de defesa esquerda
        left_defense = patches.Rectangle(
            (-field_length / 2, -defense_width / 2),
            defense_depth,
            defense_width,
            linewidth=2,
            edgecolor=self.colors["lines"],
            facecolor="none",
        )
        ax.add_patch(left_defense)

        # Área de defesa direita
        right_defense = patches.Rectangle(
            (field_length / 2 - defense_depth, -defense_width / 2),
            defense_depth,
            defense_width,
            linewidth=2,
            edgecolor=self.colors["lines"],
            facecolor="none",
        )
        ax.add_patch(right_defense)

        # Configurar eixos
        ax.set_xlim(-field_length / 2 - 0.5, field_length / 2 + 0.5)
        ax.set_ylim(-field_width / 2 - 0.5, field_width / 2 + 0.5)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("Posição X (metros)", fontsize=12)
        ax.set_ylabel("Posição Y (metros)", fontsize=12)

        return ax

    def plot_risk_heatmap(
        self,
        ball_pos: Tuple[float, float],
        resolution: float = 0.05,
        save_path: Optional[str] = None,
    ) -> plt.Figure:
        """
        Cria mapa de calor do risco baseado na posição da bola.

        Args:
            ball_pos: Posição da bola (x, y)
            resolution: Resolução do grid de posições
            save_path: Caminho para salvar a figura (opcional)

        Returns:
            Figura do matplotlib
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8), dpi=self.dpi)

        # Gerar mapa de risco
        risk_map = self.risk_model.generate_risk_map(ball_pos, resolution)

        # Plot 1: Campo com posicionamento
        self.create_field_plot(ax1)

        # Plotar bola
        ax1.scatter(
            ball_pos[0],
            ball_pos[1],
            c=self.colors["ball"],
            s=200,
            marker="o",
            edgecolor="black",
            linewidth=2,
            label="Bola",
            zorder=10,
        )

        # Plotar posição ótima do goleiro
        optimal_pos = risk_map["optimal_position"]
        ax1.scatter(
            optimal_pos[0],
            optimal_pos[1],
            c=self.colors["goalkeeper_optimal"],
            s=300,
            marker="s",
            edgecolor="black",
            linewidth=2,
            label="Goleiro (Ótimo)",
            zorder=10,
        )

        # Linha de interceptação
        ax1.axvline(
            x=self.risk_model.goalkeeper_line_x,
            color="red",
            linestyle="--",
            alpha=0.7,
            label="Linha de Defesa",
        )

        ax1.legend(fontsize=12)
        ax1.set_title(
            f"Posicionamento Ótimo do Goleiro\nBola em ({ball_pos[0]:.1f}, {ball_pos[1]:.1f})",
            fontsize=14,
            fontweight="bold",
        )

        # Plot 2: Mapa de calor do risco
        y_positions = np.array(risk_map["y_positions"])
        risk_scores = np.array(risk_map["risk_scores"])

        # Criar grid para o mapa de calor
        y_grid = np.linspace(y_positions.min(), y_positions.max(), len(y_positions))
        risk_grid = risk_scores.reshape(1, -1)

        # Colormap personalizado (verde = baixo risco, vermelho = alto risco)
        colors_risk = ["green", "yellow", "orange", "red"]
        n_bins = 100
        cmap = LinearSegmentedColormap.from_list("risk", colors_risk, N=n_bins)

        im = ax2.imshow(
            risk_grid,
            extent=[y_positions.min(), y_positions.max(), -0.1, 0.1],
            aspect="auto",
            cmap=cmap,
            alpha=0.8,
        )

        # Plotar curva de risco
        ax2_twin = ax2.twinx()
        ax2_twin.plot(
            y_positions, risk_scores, "k-", linewidth=3, label="Score de Risco"
        )
        ax2_twin.scatter(
            optimal_pos[1],
            self.risk_model.calculate_risk_score(ball_pos, optimal_pos),
            c="red",
            s=200,
            marker="*",
            edgecolor="black",
            linewidth=2,
            label="Posição Ótima",
            zorder=10,
        )

        # Configurar eixos
        ax2.set_xlabel("Posição Y do Goleiro (metros)", fontsize=12)
        ax2.set_ylabel("Visualização do Risco", fontsize=12)
        ax2_twin.set_ylabel("Score de Risco", fontsize=12)
        ax2_twin.legend(fontsize=10)

        ax2.set_title("Mapa de Calor do Risco", fontsize=14, fontweight="bold")

        # Colorbar
        cbar = plt.colorbar(im, ax=ax2, fraction=0.046, pad=0.04)
        cbar.set_label("Nível de Risco", fontsize=12)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches="tight")
            logger.info(f"Risk heatmap saved to {save_path}")

        return fig

    def plot_optimization_comparison(
        self, test_positions: List[Tuple[float, float]], save_path: Optional[str] = None
    ) -> plt.Figure:
        """
        Compara diferentes métodos de otimização.

        Args:
            test_positions: Lista de posições da bola para teste
            save_path: Caminho para salvar a figura

        Returns:
            Figura do matplotlib
        """
        # Executar benchmark
        benchmark_results = self.optimizer.benchmark_methods(test_positions)

        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(
            2, 2, figsize=(16, 12), dpi=self.dpi
        )

        methods = list(benchmark_results.keys())
        colors_methods = plt.cm.Set1(np.linspace(0, 1, len(methods)))

        # Plot 1: Tempo de execução médio
        avg_times = [benchmark_results[method]["avg_time"] for method in methods]
        bars1 = ax1.bar(methods, avg_times, color=colors_methods, alpha=0.8)
        ax1.set_ylabel("Tempo Médio (ms)", fontsize=12)
        ax1.set_title("Tempo de Execução por Método", fontsize=14, fontweight="bold")
        ax1.grid(True, alpha=0.3)

        # Adicionar valores nas barras
        for bar, time_val in zip(bars1, avg_times):
            height = bar.get_height()
            ax1.text(
                bar.get_x() + bar.get_width() / 2.0,
                height + 0.01,
                f"{time_val:.1f}ms",
                ha="center",
                va="bottom",
                fontsize=10,
            )

        # Plot 2: Distribuição dos tempos
        for i, method in enumerate(methods):
            times = benchmark_results[method]["solve_times"]
            ax2.hist(times, alpha=0.6, label=method, color=colors_methods[i], bins=10)

        ax2.set_xlabel("Tempo de Execução (ms)", fontsize=12)
        ax2.set_ylabel("Frequência", fontsize=12)
        ax2.set_title(
            "Distribuição dos Tempos de Execução", fontsize=14, fontweight="bold"
        )
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Plot 3: Posições ótimas para diferentes posições da bola
        self.create_field_plot(ax3)

        for i, ball_pos in enumerate(test_positions):
            # Usar método principal para obter posição ótima
            result = self.optimizer.optimize_position(ball_pos, method="auto")
            optimal_pos = result.optimal_position

            # Plotar bola e goleiro
            ax3.scatter(
                ball_pos[0],
                ball_pos[1],
                c=self.colors["ball"],
                s=100,
                marker="o",
                alpha=0.7,
            )
            ax3.scatter(
                optimal_pos[0],
                optimal_pos[1],
                c=colors_methods[i % len(colors_methods)],
                s=150,
                marker="s",
                alpha=0.8,
            )

            # Linha conectando bola e posição ótima
            ax3.plot(
                [ball_pos[0], optimal_pos[0]],
                [ball_pos[1], optimal_pos[1]],
                color=colors_methods[i % len(colors_methods)],
                alpha=0.5,
                linestyle="--",
            )

            # Rótulos
            ax3.annotate(
                f"B{i+1}",
                ball_pos,
                xytext=(5, 5),
                textcoords="offset points",
                fontsize=8,
            )
            ax3.annotate(
                f"G{i+1}",
                optimal_pos,
                xytext=(5, 5),
                textcoords="offset points",
                fontsize=8,
            )

        ax3.set_title(
            "Posicionamento Ótimo para Diferentes Posições da Bola",
            fontsize=14,
            fontweight="bold",
        )

        # Plot 4: Scores de risco
        risk_scores_by_method = {}
        for method in methods:
            scores = []
            for ball_pos in test_positions:
                result = self.optimizer.optimize_position(ball_pos, method=method)
                scores.append(result.risk_score)
            risk_scores_by_method[method] = scores

        x = np.arange(len(test_positions))
        width = 0.8 / len(methods)

        for i, method in enumerate(methods):
            offset = (i - len(methods) / 2) * width + width / 2
            ax4.bar(
                x + offset,
                risk_scores_by_method[method],
                width,
                label=method,
                color=colors_methods[i],
                alpha=0.8,
            )

        ax4.set_xlabel("Posição da Bola (Teste)", fontsize=12)
        ax4.set_ylabel("Score de Risco", fontsize=12)
        ax4.set_title(
            "Comparação de Scores de Risco por Método", fontsize=14, fontweight="bold"
        )
        ax4.set_xticks(x)
        ax4.set_xticklabels([f"Teste {i+1}" for i in range(len(test_positions))])
        ax4.legend()
        ax4.grid(True, alpha=0.3)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches="tight")
            logger.info(f"Optimization comparison saved to {save_path}")

        return fig

    def create_interactive_demo(
        self, ball_positions: List[Tuple[float, float]], save_path: Optional[str] = None
    ) -> plt.Figure:
        """
        Cria demonstração interativa do sistema de otimização.

        Args:
            ball_positions: Sequência de posições da bola
            save_path: Caminho para salvar animação

        Returns:
            Figura com animação
        """
        fig, ax = plt.subplots(figsize=self.figure_size, dpi=self.dpi)

        # Elementos que serão animados
        ball_scatter = None
        goalkeeper_scatter = None
        risk_text = None

        def animate(frame):
            nonlocal ball_scatter, goalkeeper_scatter, risk_text

            ax.clear()
            self.create_field_plot(ax)

            # Posição atual da bola
            ball_pos = ball_positions[frame % len(ball_positions)]

            # Calcular posição ótima do goleiro
            result = self.optimizer.optimize_position(ball_pos)
            optimal_pos = result.optimal_position

            # Plotar bola
            ball_scatter = ax.scatter(
                ball_pos[0],
                ball_pos[1],
                c=self.colors["ball"],
                s=300,
                marker="o",
                edgecolor="black",
                linewidth=3,
                label="Bola",
                zorder=10,
            )

            # Plotar goleiro
            goalkeeper_scatter = ax.scatter(
                optimal_pos[0],
                optimal_pos[1],
                c=self.colors["goalkeeper_optimal"],
                s=400,
                marker="s",
                edgecolor="black",
                linewidth=3,
                label="Goleiro (Ótimo)",
                zorder=10,
            )

            # Linha de interceptação
            ax.axvline(
                x=self.risk_model.goalkeeper_line_x,
                color="red",
                linestyle="--",
                alpha=0.7,
                label="Linha de Defesa",
            )

            # Linha conectando bola e goleiro
            ax.plot(
                [ball_pos[0], optimal_pos[0]],
                [ball_pos[1], optimal_pos[1]],
                color="red",
                alpha=0.6,
                linestyle="-",
                linewidth=2,
            )

            # Informações textuais
            info_text = (
                f"Frame: {frame + 1}/{len(ball_positions)}\n"
                f"Bola: ({ball_pos[0]:.2f}, {ball_pos[1]:.2f})\n"
                f"Goleiro: ({optimal_pos[0]:.2f}, {optimal_pos[1]:.2f})\n"
                f"Risco: {result.risk_score:.3f}\n"
                f"Def. Prob.: {result.defense_probability:.3f}\n"
                f"Método: {result.solver_status}"
            )

            ax.text(
                0.02,
                0.98,
                info_text,
                transform=ax.transAxes,
                fontsize=10,
                verticalalignment="top",
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
            )

            ax.legend(loc="upper right")
            ax.set_title(
                "Demonstração do Sistema de Otimização do Goleiro",
                fontsize=16,
                fontweight="bold",
            )

            return ball_scatter, goalkeeper_scatter

        # Criar animação
        anim = FuncAnimation(
            fig,
            animate,
            frames=len(ball_positions),
            interval=1000,
            blit=False,
            repeat=True,
        )

        if save_path:
            anim.save(save_path, writer="pillow", fps=1)
            logger.info(f"Interactive demo saved to {save_path}")

        return fig

    def plot_performance_analysis(self, save_path: Optional[str] = None) -> plt.Figure:
        """
        Análise de performance do sistema de otimização.

        Args:
            save_path: Caminho para salvar a figura

        Returns:
            Figura do matplotlib
        """
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(
            2, 2, figsize=(16, 12), dpi=self.dpi
        )

        # Gerar dados de teste
        test_positions = []
        num_tests = 50

        # Distribuir posições da bola pelo campo ofensivo
        for i in range(num_tests):
            x = np.random.uniform(0.5, self.field_dims.field_length / 2 - 0.5)
            y = np.random.uniform(
                -self.field_dims.field_width / 2 + 0.5,
                self.field_dims.field_width / 2 - 0.5,
            )
            test_positions.append((x, y))

        # Coletar dados de performance
        solve_times = []
        risk_scores = []
        defense_probabilities = []
        distances_to_goal = []

        for ball_pos in test_positions:
            result = self.optimizer.optimize_position(ball_pos)

            solve_times.append(result.solve_time_ms)
            risk_scores.append(result.risk_score)
            defense_probabilities.append(result.defense_probability)

            # Calcular distância da bola ao gol
            goal_center = self.risk_model.goal_center
            distance = np.sqrt(
                (ball_pos[0] - goal_center[0]) ** 2
                + (ball_pos[1] - goal_center[1]) ** 2
            )
            distances_to_goal.append(distance)

        # Plot 1: Tempo de execução vs. posição
        scatter1 = ax1.scatter(
            distances_to_goal,
            solve_times,
            c=risk_scores,
            cmap="coolwarm",
            alpha=0.7,
            s=50,
        )
        ax1.set_xlabel("Distância da Bola ao Gol (m)", fontsize=12)
        ax1.set_ylabel("Tempo de Execução (ms)", fontsize=12)
        ax1.set_title("Performance de Execução", fontsize=14, fontweight="bold")
        ax1.grid(True, alpha=0.3)
        plt.colorbar(scatter1, ax=ax1, label="Score de Risco")

        # Plot 2: Distribuição dos scores de risco
        ax2.hist(risk_scores, bins=20, alpha=0.7, color="skyblue", edgecolor="black")
        ax2.axvline(
            np.mean(risk_scores),
            color="red",
            linestyle="--",
            label=f"Média: {np.mean(risk_scores):.3f}",
        )
        ax2.set_xlabel("Score de Risco", fontsize=12)
        ax2.set_ylabel("Frequência", fontsize=12)
        ax2.set_title(
            "Distribuição dos Scores de Risco", fontsize=14, fontweight="bold"
        )
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Plot 3: Relação risco vs. distância
        ax3.scatter(distances_to_goal, risk_scores, alpha=0.6, color="green")

        # Ajuste de curva
        z = np.polyfit(distances_to_goal, risk_scores, 2)
        p = np.poly1d(z)
        x_smooth = np.linspace(min(distances_to_goal), max(distances_to_goal), 100)
        ax3.plot(
            x_smooth, p(x_smooth), "r--", alpha=0.8, linewidth=2, label="Tendência"
        )

        ax3.set_xlabel("Distância da Bola ao Gol (m)", fontsize=12)
        ax3.set_ylabel("Score de Risco", fontsize=12)
        ax3.set_title("Risco vs. Distância da Bola", fontsize=14, fontweight="bold")
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Plot 4: Eficiência do sistema
        efficiency_scores = [
            prob / (time_ms / 1000)
            for prob, time_ms in zip(defense_probabilities, solve_times)
        ]

        ax4.scatter(
            defense_probabilities,
            efficiency_scores,
            c=distances_to_goal,
            cmap="viridis",
            alpha=0.7,
            s=50,
        )
        ax4.set_xlabel("Probabilidade de Defesa", fontsize=12)
        ax4.set_ylabel("Eficiência (Prob/Tempo)", fontsize=12)
        ax4.set_title(
            "Eficiência do Sistema de Otimização", fontsize=14, fontweight="bold"
        )
        ax4.grid(True, alpha=0.3)

        cbar4 = plt.colorbar(ax4.collections[0], ax=ax4)
        cbar4.set_label("Distância ao Gol (m)")

        # Estatísticas gerais
        stats_text = (
            f"Estatísticas Gerais (n={num_tests}):\n"
            f"Tempo médio: {np.mean(solve_times):.1f}ms\n"
            f"Risco médio: {np.mean(risk_scores):.3f}\n"
            f"Prob. defesa média: {np.mean(defense_probabilities):.3f}\n"
            f"Eficiência média: {np.mean(efficiency_scores):.1f}"
        )

        fig.text(
            0.02,
            0.02,
            stats_text,
            fontsize=10,
            bbox=dict(boxstyle="round", facecolor="lightgray", alpha=0.8),
        )

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=self.dpi, bbox_inches="tight")
            logger.info(f"Performance analysis saved to {save_path}")

        return fig

    def generate_demo_report(self, output_dir: str = "goalkeeper_demo_output"):
        """
        Gera relatório completo de demonstração do sistema.

        Args:
            output_dir: Diretório para salvar os arquivos
        """
        import os

        # Sempre salva na pasta GoalkeeperOptimization do projeto
        base_dir = "GoalkeeperOptimization"
        output_dir = os.path.join(base_dir, output_dir)

        os.makedirs(output_dir, exist_ok=True)
        logger.info(f"Generating demo report in {output_dir}")
        
        # Criar diretório se não existir
        os.makedirs(output_dir, exist_ok=True)

        logger.info(f"Generating demo report in {output_dir}")

        # Posições de teste para demonstração
        demo_positions = [
            (1.0, 0.0),  # Centro do campo
            (1.5, 0.3),  # Diagonal direita
            (0.8, -0.2),  # Próximo, esquerda
            (2.0, 0.4),  # Longe, direita
            (0.5, 0.0),  # Muito próximo, centro
        ]

        # 1. Mapas de calor para cada posição
        for i, ball_pos in enumerate(demo_positions):
            fig = self.plot_risk_heatmap(ball_pos)
            save_path = os.path.join(output_dir, f"heatmap_ball_{i+1}.png")
            fig.savefig(save_path, dpi=self.dpi, bbox_inches="tight")
            plt.close(fig)
            logger.info(f"Heatmap {i+1} saved")

        # 2. Comparação de métodos
        fig_comparison = self.plot_optimization_comparison(demo_positions)
        comparison_path = os.path.join(output_dir, "methods_comparison.png")
        fig_comparison.savefig(comparison_path, dpi=self.dpi, bbox_inches="tight")
        plt.close(fig_comparison)

        # 3. Análise de performance
        fig_performance = self.plot_performance_analysis()
        performance_path = os.path.join(output_dir, "performance_analysis.png")
        fig_performance.savefig(performance_path, dpi=self.dpi, bbox_inches="tight")
        plt.close(fig_performance)

        # 4. Demonstração interativa (sequência de imagens)
        animation_positions = [
            (2.0, 0.0),
            (1.8, 0.2),
            (1.5, 0.3),
            (1.2, 0.1),
            (1.0, -0.1),
            (0.8, 0.0),
            (0.6, 0.2),
            (0.5, 0.0),
        ]

        fig_demo = self.create_interactive_demo(animation_positions)
        plt.close(fig_demo)

        # 5. Relatório textual
        self._generate_text_report(demo_positions, output_dir)

        logger.info(f"Demo report completed. Files saved in {output_dir}")

    def _generate_text_report(
        self, test_positions: List[Tuple[float, float]], output_dir: str
    ):
        """Gera relatório textual com análise detalhada."""
        report_path = os.path.join(output_dir, "optimization_report.txt")

        with open(report_path, "w", encoding="utf-8") as f:
            f.write("=" * 60 + "\n")
            f.write("RELATÓRIO DE OTIMIZAÇÃO DO GOLEIRO - UnBall\n")
            f.write("Universidade de Brasília\n")
            f.write("=" * 60 + "\n\n")

            f.write("CONFIGURAÇÃO DO SISTEMA:\n")
            f.write(
                f"- Dimensões do campo: {self.field_dims.field_length}m x {self.field_dims.field_width}m\n"
            )
            f.write(f"- Largura do gol: {self.field_dims.goal_width}m\n")
            f.write(
                f"- Métodos disponíveis: {', '.join(self.optimizer.get_optimization_methods())}\n\n"
            )

            f.write("ANÁLISE DAS POSIÇÕES DE TESTE:\n")
            f.write("-" * 40 + "\n")

            for i, ball_pos in enumerate(test_positions, 1):
                result = self.optimizer.optimize_position(ball_pos)

                f.write(
                    f"\nTeste {i}: Bola em ({ball_pos[0]:.2f}, {ball_pos[1]:.2f})\n"
                )
                f.write(
                    f"  Posição ótima do goleiro: ({result.optimal_position[0]:.3f}, {result.optimal_position[1]:.3f})\n"
                )
                f.write(f"  Score de risco: {result.risk_score:.4f}\n")
                f.write(
                    f"  Probabilidade de defesa: {result.defense_probability:.4f}\n"
                )
                f.write(f"  Tempo de cálculo: {result.solve_time_ms:.2f}ms\n")
                f.write(f"  Método utilizado: {result.solver_status}\n")

            # Benchmark se múltiplos métodos disponíveis
            if len(self.optimizer.get_optimization_methods()) > 1:
                f.write("\n" + "=" * 40 + "\n")
                f.write("BENCHMARK DOS MÉTODOS:\n")
                f.write("=" * 40 + "\n")

                benchmark = self.optimizer.benchmark_methods(test_positions)

                for method, stats in benchmark.items():
                    f.write(f"\n{method.upper()}:\n")
                    f.write(f"  Tempo médio: {stats['avg_time']:.2f}ms\n")
                    f.write(f"  Tempo mínimo: {stats['min_time']:.2f}ms\n")
                    f.write(f"  Tempo máximo: {stats['max_time']:.2f}ms\n")
                    f.write(f"  Tempo total: {stats['total_time']:.2f}ms\n")

            f.write("\n" + "=" * 60 + "\n")
            f.write("CONCLUSÕES:\n")
            f.write("- O sistema demonstra capacidade de otimização em tempo real\n")
            f.write("- Posicionamento baseado em minimização de risco geométrico\n")
            f.write("- Adequado para integração em sistema de controle de robôs SSL\n")
            f.write("=" * 60 + "\n")

        logger.info(f"Text report saved to {report_path}")


def create_demo_visualization(
    optimizer: GoalkeeperOptimizer,
) -> GoalkeeperVisualization:
    """
    Cria instância de visualização para demonstração.

    Args:
        optimizer: Otimizador configurado

    Returns:
        Instância de visualização
    """
    return GoalkeeperVisualization(optimizer)


# Exemplo de uso
if __name__ == "__main__":
    # Isso seria executado quando o arquivo é rodado diretamente
    from GoalkeeperOptimization.goalkeeper_optimizer import create_default_optimizer

    # Criar otimizador e visualização
    optimizer = create_default_optimizer(use_precomputed=True)
    viz = create_demo_visualization(optimizer)

    # Gerar demonstração completa
    print("Gerando demonstração do sistema de otimização do goleiro...")
    viz.generate_demo_report("demo_output")

    # Exemplo de visualização individual
    ball_position = (1.2, 0.3)
    fig = viz.plot_risk_heatmap(ball_position)
    plt.show()

    print(
        "Demonstração concluída! Verifique a pasta 'demo_output' para os arquivos gerados."
    )
