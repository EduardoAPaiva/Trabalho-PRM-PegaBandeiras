# Trabalho de Programação de Robôs Móveis - Robô que encontra e pega bandeiras

## Autores

- Eduardo Alves Paiva - 15448481
- Henrique Ribeiro de Figueiredo - 15645007

## Arquitetura

O robô foi programado utilizando uma arquitetura baseada em um mapa de estados. A seguir estão os cinco estados possíveis e uma breve explicação para cada um:

- EXPLORANDDO - Nesse estado, o robô ainda não enxergou a bandeira. Portanto, ele anda sempre para frente desviando de qualquer obstáculo que ele enxergar até encontrar a bandeira. Indepentemente do estado atual do robô, caso ele perca a bandeira de vista ele retorna a esse estado.

- BANDEIRA_ENCONTRADA - Nesse estado, o robô enxergou a bandeira em algum ponto da imagem gerada pela câmera. Dessa forma, começa a girar em torno do próprio eixo a fim de alinhar a bandeira ao centro da imagem. Quando alinha corretamente, avança para o estado INDO_PARA_BANDEIRA.

- INDO_PARA_BANDEIRA - Nesse estado, o robô já está com a bandeira centralizada e avança para frente até chegar na bandeira. Caso a bandeira saia do centro da imagem, ele retorna para o estado BANDEIRA_ENCONTRADA, para recentralizar. Caso ande para frente mas encontre um obstáculo no meio do caminho, troca o estado para DESVIANDO.

- DESVIANDO - Nesse estado, o robô enxergou um obstáculo. Portanto, ele vira para o lado oposto do qual enxergou o objeto até o objeto sair da frente. Avança até passar pelo objeto e começa a girar de volta para o lado da bandeira até reencontrá-la. Caso perca a bandeira de vista, volta para o estado EXPLORANDO.

- POSICIONANDOO_NA_BANDEIRA - Avalia se a imagem já tem uma porcentagem suficiente da bandeira. Dessa forma, começa a alinhar o robô corretamente com o mastro da bandeira, ficando completamente na direção correta e numa distância suficiente.

## Slides para apresentação

Os slides para apresentação na feira de extensão se encontram no link a seguir

https://docs.google.com/presentation/d/1-3t1QLUyHwFFuJGPD4rX3NHQ02xn1N1pefTn20Z8CGg/edit?usp=drivesdk

## Pré-requisitos

Certifique-se de que o workspace já foi compilado e que todas as dependências do ROS 2 estão instaladas.

Caso contrário, vá até o diretório raiz do workspace e execute o seguinte comando:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## Download

Navegue até a pasta src do workspace:

```bash
cd ~/seu_workspace/src
```

Execute o comando para clone:

```bash
git clone https://github.com/EduardoAPaiva/Trabalho-PRM-PegaBandeiras pega_bandeiras
```

## Execução

Abra **três terminais** e siga os passos abaixo.

### Terminal 1 – Inicialização do Ambiente

Navegue até a pasta do workspace:

```bash
cd ~/seu_workspace
```

Compile o pacote usando o comando:

```bash
colcon build
```

Carregue o ambiente ROS 2:

```bash
source install/setup.bash
```

Inicie o primeiro nó:

```bash
ros2 launch pega_bandeiras inicia_simulacao.launch.py
```

---

### Terminal 2 – Carregamento do Robô

Navegue até a pasta do workspace:

```bash
cd ~/seu_workspace
```

Carregue o ambiente ROS 2:

```bash
source install/setup.bash
```

Execute o lançamento do robô:

```bash
ros2 launch pega_bandeiras carrega_robo.launch.py
```

---

### Terminal 3 – Controle do Robô

Navegue até a pasta do workspace:

```bash
cd ~/seu_workspace
```

Carregue o ambiente ROS 2:

```bash
source install/setup.bash
```

Execute o nó de controle:

```bash
ros2 run pega_bandeiras controle_robo
```

---

## Resumo Rápido

### Terminal 1

```bash
cd ~/seu_workspace
colcon build
source install/setup.bash
ros2 launch pega_bandeiras inicia_simulacao.launch.py
```

### Terminal 2

```bash
cd ~/seu_workspace
source install/setup.bash
ros2 launch pega_bandeiras carrega_robo.launch.py
```

### Terminal 3

```bash
cd ~/seu_workspace
source install/setup.bash
ros2 run pega_bandeiras controle_robo
```

---

✅ Após iniciar os três terminais, o sistema estará pronto para execução e controle do robô.
