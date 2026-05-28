# Documentação do Código Arduino - Projeto Drago

Este repositório contém o firmware para o Arduino responsável pelo controle dos servos motores do braço robótico **Drago**. O código atua como o estágio final de uma pipeline que converte comandos de software (ROS2/MoveIt) em movimentos físicos.

## 1. Função `angleToPulse`
Esta função é responsável por mapear os ângulos recebidos para a contagem de pulso PWM específica de cada junta. Ela garante que o servo não tente se mover além dos limites físicos configurados através de verificações condicionais `if`.

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