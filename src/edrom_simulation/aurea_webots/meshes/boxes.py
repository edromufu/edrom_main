#!/usr/bin/env python3
import os
import numpy as np
from stl import mesh

def calcular_bounding_box(arquivo_stl):
    """
    Carrega um arquivo STL e calcula as dimensões de sua caixa delimitadora.
    """
    try:
        malha = mesh.Mesh.from_file(arquivo_stl)

        # Encontra os pontos mínimos e máximos em cada eixo
        min_coords = malha.min_
        max_coords = malha.max_

        # Calcula o tamanho (dimensões) da caixa
        tamanho = max_coords - min_coords

        # Arredonda para 4 casas decimais para clareza
        return np.round(tamanho, 4)

    except Exception as e:
        print(f"Erro ao processar o arquivo {arquivo_stl}: {e}")
        return None

if __name__ == "__main__":
    # Pega o diretório atual (onde o script está)
    diretorio_atual = os.getcwd()
    print(f"--- Medindo arquivos .STL em: {diretorio_atual} ---\n")

    # Encontra todos os arquivos .STL na pasta
    arquivos_stl = [f for f in os.listdir(diretorio_atual) if f.lower().endswith('.stl')]

    if not arquivos_stl:
        print("Nenhum arquivo .STL encontrado neste diretório.")
    else:
        print("--- Resultados (para copiar no seu arquivo .proto) ---")
        for nome_arquivo in sorted(arquivos_stl):
            dimensoes = calcular_bounding_box(nome_arquivo)
            if dimensoes is not None:
                # Imprime no formato exato para o Webots: size X Y Z
                print(f"Link: {nome_arquivo.replace('.STL', '')} -> size {dimensoes[0]} {dimensoes[1]} {dimensoes[2]}")