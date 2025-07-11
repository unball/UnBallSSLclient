import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import os


def gerar_graficos_consolidados():
    """
    Este script carrega múltiplos gráficos PNG e os combina em uma única figura
    para ser utilizada no relatório.

    Certifique-se de que os arquivos PNG estejam na mesma pasta que este script,
    ou forneça o caminho correto para eles.
    """
    print("Iniciando a geração da figura consolidada...")

    # --- Configuração dos Arquivos e Títulos ---
    # Mapeia a posição no grid para o arquivo de imagem e seu título
    imagens_para_plotar = {
        (0, 0): {
            "arquivo": "methods_comparison.png",
            "titulo": "A) Benchmark de Performance dos Métodos",
        },
        (0, 1): {
            "arquivo": "performance_analysis.png",
            "titulo": "B) Análise de Performance da Solução Final",
        },
        (1, 0): {
            "arquivo": "heatmap_ball_2.png",
            "titulo": "C) Mapa de Risco (Ataque Lateral)",
        },
        (1, 1): {
            "arquivo": "heatmap_ball_5.png",
            "titulo": "D) Mapa de Risco (Ataque Frontal Crítico)",
        },
    }

    # Verifica se todos os arquivos existem antes de começar
    for config in imagens_para_plotar.values():
        if not os.path.exists(config["arquivo"]):
            # LINHA CORRIGIDA AQUI
            print(f"Erro: Arquivo não encontrado -> {config['arquivo']}")
            print(
                "Por favor, verifique se todos os arquivos PNG estão na pasta correta."
            )
            return

    # --- Criação da Figura ---
    # Cria uma figura e um conjunto de subplots em um grid 2x2
    fig, axs = plt.subplots(2, 2, figsize=(18, 14), dpi=100)

    print("Montando o grid de gráficos...")

    # Itera sobre o dicionário para preencher o grid
    for posicao, config in imagens_para_plotar.items():
        linha, coluna = posicao
        ax = axs[linha, coluna]

        # Carrega a imagem PNG
        img = mpimg.imread(config["arquivo"])

        # Mostra a imagem no subplot correspondente
        ax.imshow(img)

        # Remove os eixos (números e marcações)
        ax.axis("off")

        # Adiciona um título para cada subplot
        ax.set_title(config["titulo"], fontsize=14, fontweight="bold", pad=10)

    # Ajusta o layout para evitar sobreposição de títulos
    plt.tight_layout(pad=3.0)

    # Adiciona um título geral para a figura
    fig.suptitle(
        "Análise Visual e de Performance do Sistema de Otimização",
        fontsize=20,
        fontweight="bold",
        y=1.02,
    )

    # --- Salvando a Figura Final ---
    arquivo_saida = "graficos_consolidados.png"
    try:
        plt.savefig(arquivo_saida, dpi=150, bbox_inches="tight")
        print(f"\nSucesso! Figura consolidada salva como: '{arquivo_saida}'")
    except Exception as e:
        print(f"\nErro ao salvar a figura: {e}")


if __name__ == "__main__":
    try:
        import matplotlib
    except ImportError:
        print("Erro: A biblioteca 'matplotlib' não está instalada.")
        print("Por favor, instale-a com o comando: pip install matplotlib")
    else:
        gerar_graficos_consolidados()
