# debug.py
"""
Wrapper para o sistema de logging existente em utils/logger.py
=============================================================

Este arquivo serve como uma interface simplificada para o sistema de logging
existente, mantendo compatibilidade com imports antigos e novos.

Para usar logging no projeto:
    from debug import get_logger
    logger = get_logger("component_name")
    logger.info("Sua mensagem aqui")
"""

# Importar todas as funções do sistema de logging existente
from utils.logger import (
    get_logger,
    setup_logger,
    configure_from_config,
    LOG_LEVELS,
    _loggers,
)

# Exportar as funções principais para facilitar o uso
__all__ = ["get_logger", "setup_logger", "configure_from_config", "LOG_LEVELS"]


# Funções de conveniência adicionais
def set_debug_level(component: str, level: str) -> None:
    """
    Define o nível de debug para um componente específico.

    Args:
        component (str): Nome do componente
        level (str): Novo nível de log (DEBUG, INFO, WARNING, ERROR, CRITICAL)
    """
    if component in _loggers:
        _loggers[component].setLevel(LOG_LEVELS.get(level.upper(), LOG_LEVELS["INFO"]))
    else:
        setup_logger(component, level=level)


def get_all_loggers():
    """Retorna todos os loggers ativos."""
    return _loggers.copy()


def disable_debug_globally():
    """Desabilita debug globalmente (muda para INFO)."""
    import logging

    for logger in _loggers.values():
        if logger.level == logging.DEBUG:
            logger.setLevel(logging.INFO)


def enable_debug_globally():
    """Habilita debug globalmente (muda para DEBUG)."""
    import logging

    for logger in _loggers.values():
        logger.setLevel(logging.DEBUG)
