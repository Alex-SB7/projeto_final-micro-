# 🎶 Projeto Final: Sistema Sonoro Sincronizado no RP2040 Utilizando a BitDogLab

Este repositório contém o código-fonte e documentação do projeto final de da disciplina de Microcontroladores e Microprocessadores, em que duas placas **BitDogLab (RP2040/Pico W)** reproduzem melodias em sincronia, com controles dinâmicos de velocidade, volume e visualização de níveis sonoros.

---

## 🚀 Funcionalidades

1. **Reprodução Sincronizada via UART**  
   - Master/Slave comunicando sequências de notas em tempo real.  
2. **Controle Analógico por Joystick (ADC)**  
   - Eixo X → Volume (duty‑cycle PWM)  
   - Eixo Y → Velocidade (intervalo entre notas)  
3. **Geração de Notas com PWM**  
   - Buzzer passivo recebe ondas quadradas na frequência de cada nota.  
4. **Visualização em OLED via I²C**  
   - Gráfico dinâmico de intensidade sonora (dB) no display embutido da BitDogLab.  
5. **Comunicação Wi‑Fi (opcional)**  
   - Expansão futura para toca­dorias distribuídas sem cabeamento.

---

🎥 Demonstração
🔗 Vídeo de Funcionamento: Indisponível no momento.


📚 Referências
RP2040 Datasheet – Raspberry Pi Foundation

Pico C/C++ SDK – https://www.raspberrypi.com/documentation/pico-sdk/

BitDogLab Documentation 

I²C, UART e PWM Tutorials – Pico SDK examples

🤝 Contribuidores

-Alex dos Santos Bomfim 
-Alexandre Fernandes das Neves Junior 
-Erick da Silva Sousa

Este projeto demonstra a integração de múltiplos periféricos em sistemas embarcados e abre caminho para aplicações de som ambiente distribuído e controle remoto.

