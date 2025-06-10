import logging
import os
import sys
from typing import Dict, Optional

# Dicionário para armazenar loggers por componente
_loggers: Dict[str, logging.Logger] = {}

# Níveis de log para configuração
LOG_LEVELS = {
    "DEBUG": logging.DEBUG,
    "INFO": logging.INFO,
    "WARNING": logging.WARNING,
    "ERROR": logging.ERROR,
    "CRITICAL": logging.CRITICAL,
}


def setup_logger(
    name: str, level: str = "INFO", log_to_file: bool = False, log_dir: str = "logs"
) -> logging.Logger:
    """
    Configura um logger com o nome e nível especificados.

    Args:
        name: Nome do componente/módulo para o logger
        level: Nível de log (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_to_file: Se True, salva logs em arquivo
        log_dir: Diretório onde salvar os logs

    Returns:
        Logger configurado
    """
    if name in _loggers:
        return _loggers[name]

    # Criar logger
    logger = logging.getLogger(name)
    logger.setLevel(LOG_LEVELS.get(level.upper(), logging.INFO))

    # Formato do log
    formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )

    # Handler para console
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    # Handler para arquivo se solicitado
    if log_to_file:
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        file_handler = logging.FileHandler(f"{log_dir}/{name}.log")
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    _loggers[name] = logger
    return logger


def get_logger(name: str) -> logging.Logger:
    """
    Obtém um logger existente ou cria um novo com configurações padrão.

    Args:
        name: Nome do componente/módulo

    Returns:
        Logger para o componente
    """
    if name not in _loggers:
        return setup_logger(name)
    return _loggers[name]


def configure_from_config(config: dict) -> None:
    """
    Configura loggers com base em um dicionário de configuração.

    Args:
        config: Dicionário com configurações de debug/log
    """
    debug_flags = config.get("debug_flags", {})

    # Configurar logger global
    global_level = "DEBUG" if debug_flags.get("all", False) else "INFO"
    setup_logger("main", level=global_level)

    # Configurar loggers específicos
    components = {
        "path_planning": debug_flags.get("path_planning", False),
        "robot_behavior": debug_flags.get("robot_behavior", False),
        "threads": debug_flags.get("threads", False),
        "timing": debug_flags.get("timing", False),
    }

    for component, is_debug in components.items():
        level = "DEBUG" if is_debug else "INFO"
        setup_logger(component, level=level)
