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
| Parâmetro               | PID Convencional | PID + AG |
| ----------------------- | ---------------: | -------: |
| Kp                      |               50 |   201,47 |
| Ki                      |               30 |   364,05 |
| Kd                      |                5 |    20,56 |
| Tempo de acomodação (s) |             5,52 |     0,22 |
| Erro em regime          |          0,00236 |        0 |
| IAE                     |          0,66174 |  0,05511 |

## Gráficos
- Comparação entre os controladores
- Erro ao longo do tempo
- Convergência do Algoritmo Genético

## Execução

pip install -r requirements.txt













