# Cálculo de volume aproximado de árvores

## Requisitos

Cyclone 3dr
Nuvem de pontos

## Video

Esta rotina é mostrada no vídeo: [Cálculo de volume de troncos](https://youtu.be/R-Wn5VaWwY8).

## Explicação

Rotina Javascript que permite, de forma aproximada, calcular volumes de árvores a partir de nuvens de pontos.
A rotina converte pedaços de troncos das árovres e no final calcula o volume total. Para execução desse código é necessária a preparação da nuvem de pontos. 

## Passos antes de executar a rotina

### Importação de uma ou múltiplas nuvens de pontos
Diversos formatos de nuvens de pontos podem ser importados.

### Classificação da nuvem de pontos
A classificação tenta separar a nuvem de pontos em vegetação e em outras classes. Então, nesse caso, tirando a visualização da parte que não nos interessa e deixando apenas com a parte dos troncos, teremos nossa nuvem de pontos inicial.

### Filtrar em pedaços pequenos
Para filtrar mais os pequenos pedaços de tronco que não nos interessa, selecionamos a nuvem de pontos, depois uma distância para segmentação no cyclone 3dr.

### Segmentação por distância
Esses parâmetros são explicados melhor em [Tree Meshing](https://github.com/Cyclone3DR/Scripts/tree/master/TreeMeshing). 
Em geral, quanto menor esses parâmetros, mais vai filtrando a nuvem de pontos e vai colocando mais pedaços do tronco. Quanto maiores são esses parâmetros, menos troncos terá. 

### Executar a rotina
Em resumo, este código divide a nuvem de pontos em vários pedaços e faz uma malha de cada pedaço. No final, o volume total aproximado é calculado.

Espero ter ajudado e muito obrigado pelo seu tempo!
