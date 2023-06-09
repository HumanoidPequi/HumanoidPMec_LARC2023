# Esquemáticos e informações sobre os IMUS
 As bibliotecas Podem ser encontradas na própria Arduino IDE
- Para o MPU6050: FastIMU
- Para o BNO080: SparkFun BNO080 Cortex Based IMU

## Lembrando que o as pinos padrão da ESP para I2C são:
### D21 - SDA
### D22 - SCL


![image](https://github.com/HumanoidPequi/HumanoidPMec_LARC2023/assets/60831651/ec226b15-0de1-47e2-bb14-a14f83d5f3a4)

## Esquemáticos (BNO080 e MPU6050)
![image](https://github.com/HumanoidPequi/HumanoidPMec_LARC2023/assets/60831651/3da8655d-b3f6-46ab-bf41-bc29d33af83b)

# ...[Atenção]...
caso seja encontrado problemas com a utilização do MPU6050, adicione resistores de 10k nos sinais SDA e SCL em configuração pull-up (resistores ligados diretamente no positivo [3.3V] de alimentação)
