/**************
 *  RobotDyn
 *  Dimmer Library
 *  **************
 * 
 * The following sketch is meant to define the dimming value through the serial port of the controller, 
 * using USE_SERIAL.begin 
 *    void printSpace() function is used for adding of space after functional data
 *    void loop()  serial port evaluator, used to register and define values in dimmer.setPower(outVal);
 *    
 * 
 *  ---------------------- OUTPUT & INPUT Pin table ---------------------
 *  +---------------+-------------------------+-------------------------+
 *  |   Board       | INPUT Pin               | OUTPUT Pin              |
 *  |               | Zero-Cross              |                         |
 *  +---------------+-------------------------+-------------------------+
 *  | Lenardo       | D7 (NOT CHANGABLE)      | D0-D6, D8-D13           |
 *  +---------------+-------------------------+-------------------------+
 *  | Mega          | D2 (NOT CHANGABLE)      | D0-D1, D3-D70           |
 *  +---------------+-------------------------+-------------------------+
 *  | Uno           | D2 (NOT CHANGABLE)      | D0-D1, D3-D20           |
 *  +---------------+-------------------------+-------------------------+
 *  | ESP8266       | D1(IO5),    D2(IO4),    | D0(IO16),   D1(IO5),    |
 *  |               | D5(IO14),   D6(IO12),   | D2(IO4),    D5(IO14),   |
 *  |               | D7(IO13),   D8(IO15),   | D6(IO12),   D7(IO13),   |
 *  |               |                         | D8(IO15)                |
 *  +---------------+-------------------------+-------------------------+
 *  | ESP32         | 4(GPI36),   6(GPI34),   | 8(GPO32),   9(GP033),   |
 *  |               | 5(GPI39),   7(GPI35),   | 10(GPIO25), 11(GPIO26), |
 *  |               | 8(GPO32),   9(GP033),   | 12(GPIO27), 13(GPIO14), |
 *  |               | 10(GPI025), 11(GPIO26), | 14(GPIO12), 16(GPIO13), |
 *  |               | 12(GPIO27), 13(GPIO14), | 23(GPIO15), 24(GPIO2),  |
 *  |               | 14(GPIO12), 16(GPIO13), | 25(GPIO0),  26(GPIO4),  |
 *  |               | 21(GPIO7),  23(GPIO15), | 27(GPIO16), 28(GPIO17), |
 *  |               | 24(GPIO2),  25(GPIO0),  | 29(GPIO5),  30(GPIO18), |
 *  |               | 26(GPIO4),  27(GPIO16), | 31(GPIO19), 33(GPIO21), |
 *  |               | 28(GPIO17), 29(GPIO5),  | 34(GPIO3),  35(GPIO1),  |
 *  |               | 30(GPIO18), 31(GPIO19), | 36(GPIO22), 37(GPIO23), |
 *  |               | 33(GPIO21), 35(GPIO1),  |                         |
 *  |               | 36(GPIO22), 37(GPIO23), |                         |
 *  +---------------+-------------------------+-------------------------+
 *  | Arduino M0    | D7 (NOT CHANGABLE)      | D0-D6, D8-D13           |
 *  | Arduino Zero  |                         |                         |
 *  +---------------+-------------------------+-------------------------+
 *  | Arduino Due   | D0-D53                  | D0-D53                  |
 *  +---------------+-------------------------+-------------------------+
 *  | STM32         | PA0-PA15,PB0-PB15       | PA0-PA15,PB0-PB15       |
 *  | Black Pill    | PC13-PC15               | PC13-PC15               |
 *  | BluePill      |                         |                         |
 *  | Etc...        |                         |                         |
 *  +---------------+-------------------------+-------------------------+
 */

#include <RBDdimmer.h>
#include <PID_v1.h>
#include <Ultrasonic.h>


#define USE_SERIAL  Serial
#define outputPin  4 
#define zerocross  2

// Defina as conexões do sensor ultrassônico
#define TRIGGER_PIN 6
#define ECHO_PIN 7

//Configurações do controlador
double Setpoint, Input, Output; // Define as variáveis de entrada, saída e setpoint
double Kp = 0, Ki = 0, Kd = 0; // Define os parâmetros do controlador PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // Cria uma instância do controlador PID

// Placa dimmer
dimmerLamp dimmer(outputPin);
float outVal = 0.0;

// Crie um objeto Ultrasonic
Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);

void setup() {
  USE_SERIAL.begin(9600); 
  dimmer.begin(NORMAL_MODE, ON);
  USE_SERIAL.println("Dimmer Program is starting...");
  USE_SERIAL.println("Set value");
  
  myPID.SetMode(AUTOMATIC); // Inicializa o controlador PID
}

void printSpace(int val)
{
  if ((val / 100) == 0) USE_SERIAL.print(" ");
  if ((val / 10) == 0) USE_SERIAL.print(" ");
}

void loop() {
  
  
  // Lê o valor do sensor de entrada
  Input = leituraDistancia_Objeto();

  // Calcula a saída do controlador PID
  myPID.Compute();

  // Envia a saída do controlador para o atuador
  defineSaida_Motor(Output);
  delay(50);
}

float leituraDistancia_Objeto()
{
  float distancia = ultrasonic.read(); // Realize a leitura da distância em centímetros
  return distancia;
}
void defineSaida_Motor(float saida)
{
    float preVal = outVal;

  // REMOVER APÓS A IMPLEMENTAÇÃO DO PID
  if (USE_SERIAL.available())
  {
    float buf = USE_SERIAL.parseFloat();  // ALTERAR para a variável SAIDA
    if (buf != 0) outVal = buf;
    delay(200);
  }
  // REMOVER APÓS A IMPLEMENTAÇÃO DO PID 
  
  dimmer.setPower(outVal);

  if (preVal != outVal)
  {
    USE_SERIAL.print("lampValue -> ");
    printSpace(dimmer.getPower());
    USE_SERIAL.print(dimmer.getPower());
    USE_SERIAL.println("%");
  }
  
}
//com alguns testes, a melhor faixa de operação é entre 20% e 90%. Fora isso dá uns erros
