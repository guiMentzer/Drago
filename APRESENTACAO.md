# Documentação do Código Arduino - Projeto Drago

Este repositório contém o firmware para o Arduino responsável pelo controle dos servos motores do braço robótico **Drago**. O código atua como o estágio final de uma pipeline que converte comandos de software (ROS2/MoveIt) em movimentos físicos.

## 1. Função `angleToPulse`
Esta função é responsável por mapear os ângulos recebidos para a contagem de pulso PWM específica de cada junta. Ela garante que o servo não tente se mover além dos limites físicos configurados através de verificações condicionais `if`.

(guilherme): 
Seguinte, o motor recebe largura de pulso, que vai de 78 a 585: 
```cpp
#define MIN_PWM  78 // This is the 'minimum' pulse length count (out of 4096)
#define MAX_PWM  585 // This is the 'maximum' pulse length count (out of 4096)
```
Mas o computador envia graus, de acordo com o limite de cada junta, algumas variam de -90 a +90 e outras de -10 a +170: moral da história, os limites variam. Por isso, preciso declarar o limite inferior e superior de cada junta: 
```cpp
// Máx. e mín. ângulo da junta 1
#define MIN_1 -1.57*(180/3.1415)
#define MAX_1 1.57*(180/3.1415)

// Máx. e mín. ângulo da junta 2
#define MIN_2 -1.31*(180/3.1415)
#define MAX_2 2.50*(180/3.1415)

// Máx. e mín. ângulo da junta 3
#define MIN_3 -4.00*(180/3.1415)
#define MAX_3 0.15*(180/3.1415)

// Máx. e mín. ângulo da junta 4
#define MIN_4 -1.57*(180/3.1415)
#define MAX_4 1.57*(180/3.1415)

// Máx. e mín. ângulo da junta 5
#define MIN_5 -1.57*(180/3.1415)
#define MAX_5 1.396*(180/3.1415)

// Máx. e mín. ângulo da junta 6
#define MIN_6 -1.57*(180/3.1415)
#define MAX_6 1.57*(180/3.1415)

``` 
Note que está usando a conversão de radianos para graus, porque no software lá está dizendo os limites em radianos, então é melhor deixar esses valores à vista. 

Então precisamos de uma função que seja chamada a todo momento que queremos mover um motor, para converter o valor de graus em largura de pulso:  

```cpp
int angleToPulse(float angle, int joint) { 
  switch(joint) { 
    case 0: 
      if (angle > MAX_1) { return MAX_PWM; } 
      if (angle < MIN_1) { return MIN_PWM; } 
      // Mapeia o ângulo entre o PWM mínimo e máximo com fator de calibração
      return map(angle, MIN_1, MAX_1, MIN_PWM, MAX_PWM) + 48;
  } 
}
```
Para que a função atribua os valores corretos a determinada junta (motor), também passamos qual a junta queremos: 
```cpp
int angleToPulse(float angle, int joint)
``` 
Para cada junta, teremos valores diferentes, por isso usar o switch. Cada caso, de 0 a 5, representa uma junta (motor)

Dentro de cada junta, se os valores extrapolam os limites, retornamos apenas o limite, para não mandar ao robô um valor que ele não reconhece: 

```cpp
 if (angle > MAX_1) { return MAX_PWM; } 
 if (angle < MIN_1) { return MIN_PWM; } 
```

Se o valor está dentro dos limites, então retornamos um mapeamento desse valor dentro de outros limites (mesma proporção, mas com tamnho diferente)(estude a função map)

```cpp
 return map(angle, MIN_1, MAX_1, MIN_PWM, MAX_PWM) + 48;
```

## 3. Loop Principal e Comunicação Serial
O `void loop()` monitora constantemente a porta serial. Se receber o comando `"HOME"`, o robô retorna à posição inicial. Se receber comandos de juntas iniciados por `"J "`, ele processa a string para atualizar a posição de cada motor.

```cpp
void setup() { 
  Serial.begin(115200); 
  Driver.begin(); 
  Driver.setPWMFreq(SERVO_FREQ);
}

void loop() { 
  if (!Serial.available()) return; // Aguarda dados seriais

  String input = Serial.readStringUntil('\n'); 
  input.trim();

  if (input.length() == 0) return; 

  // Comando para retornar à posição de repouso
  if (input == "HOME") { 
    for(byte j=0; j<=5; j++) { 
      angles[j] = 0.0;
      Driver.setPWM(j, 0, angleToPulse(angles[j], j)); 
      delay(10); 
    } 
  } 

  // Recebimento de novos ângulos das juntas
  if (input.startsWith("J ")) { 
    String data = input.substring(3); 
    data.trim();
    // O processamento extrai os valores de radianos para o array angles[]
    // e executa Driver.setPWM() para cada junta.
  }
}
```

## Conclusão
O uso da função `map()` e da estrutura `switch` permite que o Drago tenha movimentos calibrados individualmente para cada motor, traduzindo o planejamento virtual em ação física precisa.