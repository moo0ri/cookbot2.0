# Cookbot 2.0

# Controle de Sistemas Robóticos – Cookbot

**Disciplina:** Controle de Sistemas Robóticos (SEM0591)  
**Universidade:** Universidade de São Paulo  
**Professores:** Adriano Siqueira, Marcelo Becker  

## Integrantes

- Fael Juraszek Pareja – 13730023  
- João Lucas de Felício Pereira da Silva – 13678904  
- Pedro Augusto Codognhoto dos Santos – 13679120  
- Victor Martinez Lechuga Dutra – 13678950  
- Vinícius Mori Sartor – 13678550  

---

## Sobre o projeto

Este repositório contém todo o desenvolvimento teórico, computacional e experimental do projeto de controle do manipulador robótico Cookbot, incluindo:

- Modelagem dinâmica e parâmetros do robô  
- Geração de trajetórias e análise de singularidades  
- Projeto, implementação e simulação do controlador PID  
- Análise comparativa teoria x simulação  
- Scripts para geração de gráficos de validação e análise  
- Relatório final e vídeo de apresentação  



---

## Instalação e uso

1. **Pré-requisitos:**  
   Python 3.9.21  
   roboticstoolbox-python==1.1.1  
   numpy==1.26.4  
   sympy==1.13.3  
   matplotlib==3.5.1  
   scipy

2. **Instale as dependências:**  

pip install -r requirements.txt


3. **Para rodar as simulações e gerar gráficos:**  
- Execute os scripts principais dentro da pasta `src/`:
  - `Trajetória com controle.py` — simula o controle PID e salva a trajetória real.
  - `plot_traj_real.py` — plota a trajetória real simulada.
  - `plot_traj_des.py` — plota a trajetória desejada (teoria).
  - `plot_torques.py` — plota os torques aplicados em cada junta.
  - `Simulacao_palito.py` — interface de ensino e visualização do robô.
- Os arquivos de dados `.npy` estão em `data/` e são gerados/atualizados pela simulação.
- Os gráficos gerados podem ser salvos em `figuras/`.

---

## Sobre o relatório final

O relatório final (em `relatorio/RelatorioFinal.pdf`) está estruturado conforme solicitado:

1. **Definição da trajetória e análise de singularidades:**  
- Escolha da trajetória no espaço operacional  
- Análise matemática das singularidades  
- Justificativa da trajetória

2. **Projeto do controlador linear (PID):**  
- Equação do controlador e dedução teórica  
- Cálculo e justificativa dos ganhos  
- Gráficos obrigatórios para cada junta:  
  - Seguimento da trajetória: posição desejada x obtida  
  - Evolução do erro de posição ao longo do tempo  
  - Sinais de controle: torques aplicados em cada junta  
  - Análise comparativa teoria vs simulação  

3. **Resultados, análise e conclusão:**  
- Discussão dos erros observados e possíveis melhorias

4. **Apresentação em vídeo:**  
- Vídeo explicativo disponível em `videos/apresentacao.mp4`

---

## Observações

- O código está modularizado e comentado para facilitar análise, reprodução e avaliação.
- Todos os gráficos obrigatórios podem ser gerados com os scripts fornecidos.
- Para dúvidas ou reprodutibilidade, consulte os notebooks e scripts de simulação na pasta `src/`.

---

**Todo o código-fonte, dados, gráficos, relatório final e vídeo estão disponíveis neste repositório público, conforme solicitado na avaliação.**

