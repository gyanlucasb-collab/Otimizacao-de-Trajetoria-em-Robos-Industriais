## Otimização de Trajetória em Robôs Industriais utilizando PID e Algoritmo Genético
## Descrição

Este projeto apresenta a implementação de um controlador PID otimizado por Algoritmo Genético (AG) para um sistema representativo de trajetória em robôs industriais.

O desenvolvimento foi realizado em Python e teve como objetivo comparar o desempenho do controlador PID convencional com o controlador otimizado por AG, avaliando:

- Tempo de acomodação;
- Overshoot;
- Erro em regime permanente;
- Integral do Erro Absoluto (IAE).

## Tecnologias utilizadas
- Python 3
- NumPy
- Matplotlib
- Control
- DEAP

## Resultados obtidos
## PID convencional
Kp = 50;
Ki = 30;
Kd = 5;
Tempo de acomodação = 5,52 segundos;
Erro em regime = 0,00236;
IAE = 0,66174;

## PID otimizado por AG
Kp = 201,47;
Ki = 364,05;
Kd = 20,56;
Tempo de acomodação = 0,22 segundos;
Erro em regime = 0;
IAE = 0,05511;

## Gráficos
Comparação entre os controladores

Erro ao longo do tempo

Convergência do Algoritmo Genético

## Execução

pip install -r requirements.txt











