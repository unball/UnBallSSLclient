"""
GoalkeeperOptimization/goalkeeper_optimizer.py

Este módulo implementa a otimização matemática do posicionamento do goleiro
para minimizar o risco de tomar gol.

Autor: UnBall Team - Universidade de Brasília
"""

import threading
import time
from typing import Tuple, Dict, List, Optional, Any
from dataclasses import dataclass

# OR-Tools imports
try:
    from ortools.linear_solver import pywraplp

    ORTOOLS_AVAILABLE = True
except ImportError:
    ORTOOLS_AVAILABLE = False
    print("WARNING: OR-Tools not available. Install with: pip install ortools")

from GoalkeeperOptimization.goalkeeper_risk_model import (
    GoalkeeperRiskModel,
    FieldDimensions,
    GoalkeeperConstraints,
)

from utils.logger import get_logger

logger = get_logger("goalkeeper_optimizer")


@dataclass
class OptimizationResult:
    """Resultado da otimização do posicionamento do goleiro."""

    optimal_position: Tuple[float, float]
    risk_score: float
    defense_probability: float
    solve_time_ms: float
    solver_status: str
    objective_value: float


class GoalkeeperOptimizer:
    """
    Otimizador principal para posicionamento do goleiro usando OR-Tools.

    Este otimizador resolve o problema de minimização de risco em tempo real,
    considerando as restrições físicas do goleiro e do campo.
    """

    def __init__(
        self, risk_model: GoalkeeperRiskModel, use_precomputed_matrix: bool = True
    ):
        self.risk_model = risk_model
        self.use_precomputed_matrix = use_precomputed_matrix
        self.logger = logger

        # Cache para otimização
        self._optimization_cache = {}
        self._cache_lock = threading.Lock()

        # Matriz pré-computada (opcional)
        self._risk_matrix = None
        self._position_grid = None

        if not ORTOOLS_AVAILABLE:
            self.logger.warning("OR-Tools not available. Using fallback optimization.")

        if self.use_precomputed_matrix:
            self._precompute_risk_matrix()

        self.logger.info(
            f"GoalkeeperOptimizer initialized. OR-Tools: {ORTOOLS_AVAILABLE}, "
            f"Precomputed matrix: {use_precomputed_matrix}"
        )

    def _precompute_risk_matrix(self):
        """
        Pré-computa matriz de risco para diferentes posições da bola e goleiro.

        Esta abordagem é mais eficiente para demonstrações e análises offline,
        mas pode consumir mais memória.
        """
        self.logger.info("Pre-computing risk matrix...")
        start_time = time.time()

        # Grid de posições da bola (apenas posições relevantes)
        field_dims = self.risk_model.field_dims
        ball_x_positions = []
        ball_y_positions = []

        # Bola apenas no campo ofensivo (x > 0 para simplificar)
        x_step = 0.2  # 20cm de resolução
        y_step = 0.2  # 20cm de resolução

        for x in [i * x_step for i in range(1, int(field_dims.field_length / x_step))]:
            ball_x_positions.append(x)

        for y in [
            i * y_step
            for i in range(
                int(-field_dims.field_width / (2 * y_step)),
                int(field_dims.field_width / (2 * y_step)) + 1,
            )
        ]:
            ball_y_positions.append(y)

        # Grid de posições do goleiro (linha Y)
        goal_half_width = field_dims.goal_width / 2
        margin = self.risk_model.constraints.robot_radius
        gk_y_positions = []

        gk_y_step = 0.05  # 5cm de resolução para o goleiro
        for y in [
            i * gk_y_step
            for i in range(
                int((-goal_half_width + margin) / gk_y_step),
                int((goal_half_width - margin) / gk_y_step) + 1,
            )
        ]:
            gk_y_positions.append(y)

        # Pré-computar matriz
        self._risk_matrix = {}
        self._position_grid = {
            "ball_x": ball_x_positions,
            "ball_y": ball_y_positions,
            "gk_y": gk_y_positions,
            "gk_x": self.risk_model.goalkeeper_line_x,
        }

        computed = 0

        for ball_x in ball_x_positions:
            for ball_y in ball_y_positions:
                ball_pos = (ball_x, ball_y)
                self._risk_matrix[ball_pos] = {}

                for gk_y in gk_y_positions:
                    gk_pos = (self._position_grid["gk_x"], gk_y)
                    risk_score = self.risk_model.calculate_risk_score(ball_pos, gk_pos)
                    self._risk_matrix[ball_pos][gk_y] = risk_score
                    computed += 1

        computation_time = time.time() - start_time
        self.logger.info(
            f"Risk matrix pre-computed: {computed} combinations in {computation_time:.2f}s"
        )

    def optimize_position_ortools(
        self, ball_pos: Tuple[float, float]
    ) -> OptimizationResult:
        """
        Otimiza posição do goleiro usando OR-Tools.

        Args:
            ball_pos: Posição da bola (x, y)

        Returns:
            Resultado da otimização
        """
        if not ORTOOLS_AVAILABLE:
            return self._fallback_optimization(ball_pos)

        start_time = time.time()

        # Criar solver
        solver = pywraplp.Solver.CreateSolver("SCIP")
        if not solver:
            self.logger.error("SCIP solver not available")
            return self._fallback_optimization(ball_pos)

        # Variáveis de decisão
        # Posição Y do goleiro (X é fixo na linha de defesa)
        field_dims = self.risk_model.field_dims
        goal_half_width = field_dims.goal_width / 2
        margin = self.risk_model.constraints.robot_radius

        min_y = -goal_half_width + margin
        max_y = goal_half_width - margin

        # Variável contínua para posição Y do goleiro
        gk_y = solver.NumVar(min_y, max_y, "goalkeeper_y")

        # Para OR-Tools, vamos discretizar o problema ou usar aproximação linear
        # Aqui usamos uma abordagem simplificada: aproximação quadrática do risco

        # Calcular posição ótima analítica como referência
        optimal_pos_analytical = self.risk_model.calculate_optimal_position(ball_pos)
        target_y = optimal_pos_analytical[1]

        # Função objetivo: minimizar diferença quadrática da posição ótima
        # (aproximação da minimização de risco)
        objective = solver.Objective()
        objective.SetCoefficient(gk_y, 2 * target_y)  # Termo linear
        objective.SetOffset(target_y**2)  # Termo constante
        objective.SetMinimization()

        # Resolver
        status = solver.Solve()
        solve_time_ms = (time.time() - start_time) * 1000

        # Processar resultado
        if status == pywraplp.Solver.OPTIMAL:
            optimal_gk_y = gk_y.solution_value()
            optimal_gk_pos = (self.risk_model.goalkeeper_line_x, optimal_gk_y)

            risk_score = self.risk_model.calculate_risk_score(ball_pos, optimal_gk_pos)
            defense_prob = self.risk_model.calculate_defense_probability(
                ball_pos, optimal_gk_pos
            )

            result = OptimizationResult(
                optimal_position=optimal_gk_pos,
                risk_score=risk_score,
                defense_probability=defense_prob,
                solve_time_ms=solve_time_ms,
                solver_status="OPTIMAL",
                objective_value=solver.Objective().Value(),
            )

            self.logger.debug(
                f"OR-Tools optimization successful: {result.optimal_position}"
            )
            return result

        else:
            self.logger.warning(f"OR-Tools solver failed with status: {status}")
            return self._fallback_optimization(ball_pos)

    def optimize_position_matrix(
        self, ball_pos: Tuple[float, float]
    ) -> OptimizationResult:
        """
        Otimiza posição usando matriz pré-computada.

        Args:
            ball_pos: Posição da bola

        Returns:
            Resultado da otimização
        """
        if not self._risk_matrix:
            self.logger.warning("Risk matrix not available. Using analytical method.")
            return self._fallback_optimization(ball_pos)

        start_time = time.time()

        # Encontrar posição da bola mais próxima no grid
        closest_ball_pos = self._find_closest_ball_position(ball_pos)

        if closest_ball_pos not in self._risk_matrix:
            self.logger.warning(f"Ball position {ball_pos} not found in matrix")
            return self._fallback_optimization(ball_pos)

        # Encontrar posição Y do goleiro com menor risco
        risk_scores = self._risk_matrix[closest_ball_pos]
        optimal_gk_y = min(risk_scores.keys(), key=lambda y: risk_scores[y])
        min_risk = risk_scores[optimal_gk_y]

        optimal_gk_pos = (self._position_grid["gk_x"], optimal_gk_y)
        defense_prob = 1.0 - min_risk

        solve_time_ms = (time.time() - start_time) * 1000

        result = OptimizationResult(
            optimal_position=optimal_gk_pos,
            risk_score=min_risk,
            defense_probability=defense_prob,
            solve_time_ms=solve_time_ms,
            solver_status="OPTIMAL_MATRIX",
            objective_value=min_risk,
        )

        self.logger.debug(
            f"Matrix optimization: {result.optimal_position}, risk: {min_risk:.3f}"
        )
        return result

    def _find_closest_ball_position(
        self, ball_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """Encontra posição da bola mais próxima no grid pré-computado."""
        ball_x, ball_y = ball_pos

        # Encontrar X mais próximo
        closest_x = min(self._position_grid["ball_x"], key=lambda x: abs(x - ball_x))

        # Encontrar Y mais próximo
        closest_y = min(self._position_grid["ball_y"], key=lambda y: abs(y - ball_y))

        return (closest_x, closest_y)

    def _fallback_optimization(
        self, ball_pos: Tuple[float, float]
    ) -> OptimizationResult:
        """
        Método de fallback usando cálculo analítico direto.

        Args:
            ball_pos: Posição da bola

        Returns:
            Resultado da otimização analítica
        """
        start_time = time.time()

        optimal_pos = self.risk_model.calculate_optimal_position(ball_pos)
        risk_score = self.risk_model.calculate_risk_score(ball_pos, optimal_pos)
        defense_prob = self.risk_model.calculate_defense_probability(
            ball_pos, optimal_pos
        )

        solve_time_ms = (time.time() - start_time) * 1000

        result = OptimizationResult(
            optimal_position=optimal_pos,
            risk_score=risk_score,
            defense_probability=defense_prob,
            solve_time_ms=solve_time_ms,
            solver_status="ANALYTICAL",
            objective_value=risk_score,
        )

        self.logger.debug(f"Analytical optimization: {result.optimal_position}")
        return result

    def optimize_position(
        self, ball_pos: Tuple[float, float], method: str = "auto"
    ) -> OptimizationResult:
        """
        Método principal de otimização com seleção automática do método.

        Args:
            ball_pos: Posição da bola (x, y)
            method: Método de otimização ("auto", "ortools", "matrix", "analytical")

        Returns:
            Resultado da otimização
        """
        # Cache para evitar recálculos desnecessários
        cache_key = (round(ball_pos[0], 2), round(ball_pos[1], 2), method)

        with self._cache_lock:
            if cache_key in self._optimization_cache:
                cached_result = self._optimization_cache[cache_key]
                self.logger.debug(f"Using cached result for {ball_pos}")
                return cached_result

        # Selecionar método automaticamente
        if method == "auto":
            if self.use_precomputed_matrix and self._risk_matrix:
                method = "matrix"
            elif ORTOOLS_AVAILABLE:
                method = "ortools"
            else:
                method = "analytical"

        # Executar otimização
        if method == "ortools":
            result = self.optimize_position_ortools(ball_pos)
        elif method == "matrix":
            result = self.optimize_position_matrix(ball_pos)
        else:  # analytical
            result = self._fallback_optimization(ball_pos)

        # Armazenar em cache (limitado a 100 entradas)
        with self._cache_lock:
            if len(self._optimization_cache) > 100:
                # Remove entrada mais antiga
                oldest_key = next(iter(self._optimization_cache))
                del self._optimization_cache[oldest_key]

            self._optimization_cache[cache_key] = result

        self.logger.debug(
            f"Optimization completed: method={method}, time={result.solve_time_ms:.1f}ms"
        )
        return result

    def get_optimization_methods(self) -> List[str]:
        """Retorna lista de métodos de otimização disponíveis."""
        methods = ["analytical"]

        if self._risk_matrix:
            methods.append("matrix")

        if ORTOOLS_AVAILABLE:
            methods.append("ortools")

        return methods

    def benchmark_methods(
        self, test_positions: List[Tuple[float, float]]
    ) -> Dict[str, Dict]:
        """
        Compara performance dos diferentes métodos de otimização.

        Args:
            test_positions: Lista de posições da bola para teste

        Returns:
            Estatísticas de benchmark para cada método
        """
        methods = self.get_optimization_methods()
        results = {}

        for method in methods:
            method_results = {
                "solve_times": [],
                "risk_scores": [],
                "total_time": 0,
                "avg_time": 0,
                "min_time": float("inf"),
                "max_time": 0,
            }

            self.logger.info(f"Benchmarking method: {method}")
            start_total = time.time()

            for ball_pos in test_positions:
                result = self.optimize_position(ball_pos, method=method)

                method_results["solve_times"].append(result.solve_time_ms)
                method_results["risk_scores"].append(result.risk_score)
                method_results["min_time"] = min(
                    method_results["min_time"], result.solve_time_ms
                )
                method_results["max_time"] = max(
                    method_results["max_time"], result.solve_time_ms
                )

            method_results["total_time"] = (time.time() - start_total) * 1000
            method_results["avg_time"] = sum(method_results["solve_times"]) / len(
                method_results["solve_times"]
            )

            results[method] = method_results

            self.logger.info(
                f"Method {method}: avg={method_results['avg_time']:.1f}ms, "
                f"min={method_results['min_time']:.1f}ms, max={method_results['max_time']:.1f}ms"
            )

        return results

    def clear_cache(self):
        """Limpa cache de otimização."""
        with self._cache_lock:
            self._optimization_cache.clear()
        self.logger.info("Optimization cache cleared")

    def get_cache_stats(self) -> Dict:
        """Retorna estatísticas do cache."""
        with self._cache_lock:
            return {
                "cache_size": len(self._optimization_cache),
                "cache_keys": list(self._optimization_cache.keys())[
                    :5
                ],  # Primeiras 5 chaves
            }


class ThreadedGoalkeeperOptimizer:
    """
    Versão threaded do otimizador para uso em tempo real.

    Esta classe roda a otimização em uma thread separada para não bloquear
    o loop principal do jogo.
    """

    def __init__(self, optimizer: GoalkeeperOptimizer):
        self.optimizer = optimizer
        self.logger = get_logger("threaded_goalkeeper_optimizer")

        # Threading
        self._optimization_thread = None
        self._stop_event = threading.Event()
        self._result_lock = threading.Lock()

        # Estado
        self._current_ball_pos = None
        self._latest_result = None
        self._optimization_active = False

        self.logger.info("ThreadedGoalkeeperOptimizer initialized")

    def start_optimization_thread(self):
        """Inicia thread de otimização contínua."""
        if self._optimization_thread and self._optimization_thread.is_alive():
            self.logger.warning("Optimization thread already running")
            return

        self._stop_event.clear()
        self._optimization_thread = threading.Thread(
            target=self._optimization_loop, daemon=True
        )
        self._optimization_thread.start()

        self.logger.info("Optimization thread started")

    def stop_optimization_thread(self):
        """Para thread de otimização."""
        if self._optimization_thread and self._optimization_thread.is_alive():
            self._stop_event.set()
            self._optimization_thread.join(timeout=1.0)

            if self._optimization_thread.is_alive():
                self.logger.warning("Optimization thread did not stop gracefully")
            else:
                self.logger.info("Optimization thread stopped")

    def update_ball_position(self, ball_pos: Tuple[float, float]):
        """
        Atualiza posição da bola para otimização.

        Args:
            ball_pos: Nova posição da bola
        """
        self._current_ball_pos = ball_pos
        self.logger.debug(f"Ball position updated: {ball_pos}")

    def get_latest_result(self) -> Optional[OptimizationResult]:
        """
        Retorna resultado mais recente da otimização.

        Returns:
            Último resultado de otimização ou None se não disponível
        """
        with self._result_lock:
            return self._latest_result

    def _optimization_loop(self):
        """Loop principal da thread de otimização."""
        self.logger.info("Optimization loop started")

        while not self._stop_event.is_set():
            try:
                if self._current_ball_pos is not None:
                    # Executar otimização
                    result = self.optimizer.optimize_position(self._current_ball_pos)

                    # Atualizar resultado
                    with self._result_lock:
                        self._latest_result = result

                    self.logger.debug(
                        f"Optimization completed: {result.optimal_position}"
                    )

                # Aguardar antes da próxima iteração (não otimizar muito frequentemente)
                time.sleep(0.05)  # 20Hz

            except Exception as e:
                self.logger.error(f"Error in optimization loop: {e}")
                time.sleep(0.1)  # Aguarda mais em caso de erro

        self.logger.info("Optimization loop stopped")


def create_default_optimizer(use_precomputed: bool = True) -> GoalkeeperOptimizer:
    """
    Cria otimizador com configurações padrão.

    Args:
        use_precomputed: Se deve usar matriz pré-computada

    Returns:
        Instância do otimizador configurado
    """
    from GoalkeeperOptimization.goalkeeper_risk_model import (
        create_default_risk_model,
    )

    risk_model = create_default_risk_model()
    optimizer = GoalkeeperOptimizer(risk_model, use_precomputed_matrix=use_precomputed)

    return optimizer


# Exemplo de uso e teste
if __name__ == "__main__":
    # Criar otimizador
    optimizer = create_default_optimizer(use_precomputed=True)

    # Testar otimização
    test_ball_positions = [
        (1.0, 0.0),  # Centro do campo
        (1.5, 0.3),  # Lateral direita
        (0.5, -0.2),  # Próximo ao gol
        (2.0, 0.4),  # Longe do gol
    ]

    print("Testing goalkeeper optimization:")
    print("=" * 50)

    for ball_pos in test_ball_positions:
        result = optimizer.optimize_position(ball_pos)

        print(f"\nBall position: {ball_pos}")
        print(f"Optimal goalkeeper position: {result.optimal_position}")
        print(f"Risk score: {result.risk_score:.3f}")
        print(f"Defense probability: {result.defense_probability:.3f}")
        print(f"Solve time: {result.solve_time_ms:.1f}ms")
        print(f"Method: {result.solver_status}")

    # Benchmark dos métodos
    if len(optimizer.get_optimization_methods()) > 1:
        print("\n" + "=" * 50)
        print("Benchmarking optimization methods:")

        benchmark_results = optimizer.benchmark_methods(test_ball_positions)

        for method, stats in benchmark_results.items():
            print(f"\n{method.upper()}:")
            print(f"  Average time: {stats['avg_time']:.1f}ms")
            print(f"  Min time: {stats['min_time']:.1f}ms")
            print(f"  Max time: {stats['max_time']:.1f}ms")
            print(f"  Total time: {stats['total_time']:.1f}ms")
