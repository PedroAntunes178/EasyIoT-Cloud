#include <ESP8266WiFi.h>
#include <MQTT.h>
#include <EEPROM.h>


// Definir o DEBUG como 1 faz com que os prints apareçam
#define DEBUG 0


// O SSID é o nome da rede a que o vosso computador se vai conectar
// A password é a da rede de internet a qual te estas a conectar
#define AP_SSID     "xxx"
#define AP_PASSWORD "xxx"


// Antes de se definir a palavra pass e o username é preciso criar conta em:
// https://easyiot-cloud.com
#define EIOTCLOUD_USERNAME "MDemonKing"
#define EIOTCLOUD_PASSWORD "<(AB$[t~!_zt52@y"


// MQTT é um protocolo de transmissão de mensagens para IoT (Internet of Things) da OASIS (Organization for the Advancement of Structured Information Standards)
// O endereço que definimos aqui é onde ele vai buscar a informação
#define EIOT_CLOUD_ADDRESS "cloud.iot-playground.com"



#define PIN_PUMP         BUILTIN_LED //D0  // nodemcu built in LED
#define PIN_BUTTON       D3  // nodemcu flash button
#define PIN_HUM_ANALOG   A0  // Pin analógico da humidade

// Valores máximos e minimos que se vão ler no Pin analógico da humidade
#define MAX_ANALOG_VAL         956
#define MIN_ANALOG_VAL         250

// Define o tempo que a bomba está a funcionar o tempo em que está parada
#define IRRIGATION_TIME        10 // Tempo de irrigação em segundos
#define IRRIGATION_PAUSE_TIME  300 // Tempo de pausa da irrigação em segundos - só é relevante no modo de automatico


// Estado de funcionamento da bomba de aguá
typedef enum {
  s_idle             = 0,  // irrigation idle
  s_irrigation_start = 1,  // start irrigation
  s_irrigation       = 2,  // irrigate
  s_irrigation_stop  = 3,  // irrigation stop
} e_state;


// Definir a partir de onde é que vamos guardar na EEPROM (memoria não volatile)
#define CONFIG_START 0
#define CONFIG_VERSION "NEECv01"

// Estrutura dos dados que estão guardados na memoria
struct StoreStruct {
  // Este array de chars serve apenas para verificar se a versão está correcta
  char version[8];
  // Daqui para baixo definimos as variaveis que
  uint moduleId;  // module id
} storage = {
  CONFIG_VERSION, // Guarda NEECv01
  0, // Valor default do module 0
};


// Define os nomes dos parametros que se vão poder modificar a partir da cloud
#define PARAM_HUMIDITY_TRESHOLD   "Sensor.Parameter1"
#define PARAM_MANUAL_AUTO_MODE    "Sensor.Parameter2"
#define PARAM_PUMP_ON             "Sensor.Parameter3"
#define PARAM_HUMIDITY            "Sensor.Parameter4"

// Definir o tempo que esperamos pelas mensagens da cloud
#define MS_IN_SEC  1000 // 1S  

// Criar o objecto de MQTT
MQTT myMqtt("", EIOT_CLOUD_ADDRESS, 1883);

// Variaveis globais
int state; // Diz em que estado é que estamos
int soilHumidityThreshold; // Limiar da humidade do solo
bool autoMode; // Define se o modo automatico está ligado ou não
String valueStr(""); // Guarda o valor que é para enviar para a cloud
String topic(""); // Guarda uma string relativamente ao topico a que nos queremos referir quando enviamos uma mensagem para a cloud
boolean result; // Recebe o resultado da comunicação com a cloud
int lastAnalogReading; // Ultimo valor analogico lido a partir do pin analogico
bool autoModeOld; // Estado antigo do autoMode
int soilHumidityThresholdOld; // Limiar da humidade do solo antigo
unsigned long startTime; // Guarda um instante de tempo inicial
int soilHum; // Humidade do solo
int irrigatorCounter; // Conta quantos segundos o nodeMCU está num determinado estado


// Esta função vai correr só uma vez quando se liga o nodeMCU à corrente
void setup() {
  state = s_idle; // começa no estado idle
  // Defenir o modo de funcionamento dos pins do nodeMCU
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);

  autoMode = false; // inicializa com o modo automatico desligado
  soilHumidityThresholdOld = -1; // inicializa com o valor de limiar da humidade do solo a -1
  startTime = millis(); // Vai buscar o valor inicial do tempo, a millis() retorna o tempo decorrido desde que o nodeMCU foi ligado
  soilHum = -1; // inicializa com o valor da humidade do solo a -1

  Serial.begin(115200); //Inicia a comunicação com uma largura de banda de 115200 bps (baud rate)

  // Conecta a rede Wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(AP_SSID, AP_PASSWORD);

  // Os print que se seguem servem para verificar se o nodeMCU está conectado à rede
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(AP_SSID);

  while (WiFi.status() != WL_CONNECTED) { // Espera até estar conectado
    delay(500);
    Serial.print(".");
  };

  Serial.println("WiFi connected");
  Serial.println("Connecting to MQTT server");
  // Acaba aqui

  // Inicia a comunicação com a EEPROM (memoria não volátil)
  EEPROM.begin(512);
  // Carrega as configurações do utilizador
  loadConfig();


  // Define o id do client
  // E gerar o nome do cliente baseando-se no endereço MAC e nos ultimos 8 bits do contador de microsegundos
  String clientName;
  uint8_t mac[6];
  WiFi.macAddress(mac); // Vai buscar o endereço MAC
  //clientName += "esp8266-";
  clientName += macToStr(mac);
  clientName += "-";
  clientName += String(micros() & 0xff, 16);
  myMqtt.setClientId((char*) clientName.c_str()); // .c_str() retorna um ponteiro para a string terminada com \0 como no C

  // Imprime o nome dado ao cliente
  Serial.print("MQTT client id:");
  Serial.println(clientName);

  // Definir as funções que vão ocorrer em resposta a certos eventos
  myMqtt.onConnected(myConnectedCb);
  myMqtt.onDisconnected(myDisconnectedCb);
  myMqtt.onPublished(myPublishedCb);
  myMqtt.onData(myDataCb);

  // Conectar à cloud por MQTT
  myMqtt.setUserPwd(EIOTCLOUD_USERNAME, EIOTCLOUD_PASSWORD);
  myMqtt.connect();

  delay(500); // para por 0.5 sec

  // Imprime o id do modulo em utilização
  Serial.print("ModuleId: ");
  Serial.println(storage.moduleId);


  // Se necessário cria um modulo
  if (storage.moduleId == 0)
  {
    //create module
    Serial.println("create module: /NewModule");
    storage.moduleId = myMqtt.NewModule();

    if (storage.moduleId == 0)
    {
      Serial.println("Module NOT created. Check module limit");
      while (1)
        delay(100);
    }

    // Definir tipo de modulo
    Serial.println("Set module type");
    myMqtt.SetModuleType(storage.moduleId, "ZMT_IRRIGATOR");

    // create Sensor.Parameter1 - humidity treshold value
    Serial.println("new parameter: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY_TRESHOLD);
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_HUMIDITY_TRESHOLD);
    // set IsCommand
    Serial.println("set isCommand: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY_TRESHOLD);
    myMqtt.SetParameterIsCommand(storage.moduleId, PARAM_HUMIDITY_TRESHOLD, true);


    // create Sensor.Parameter2
    // Sensor.Parameter2 - manual/auto mode 0 - manual, 1 - auto mode
    Serial.println("new parameter: /" + String(storage.moduleId) + "/" + PARAM_MANUAL_AUTO_MODE);
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_MANUAL_AUTO_MODE);
    // set IsCommand
    Serial.println("set isCommand: /" + String(storage.moduleId) + "/" + PARAM_MANUAL_AUTO_MODE);
    myMqtt.SetParameterIsCommand(storage.moduleId, PARAM_MANUAL_AUTO_MODE, true);


    // create Sensor.Parameter3
    // Sensor.Parameter3 - pump on/ pump off
    Serial.println("new parameter: /" + String(storage.moduleId) + "/" + PARAM_PUMP_ON);
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_PUMP_ON);
    // set IsCommand
    Serial.println("set isCommand: /" + String(storage.moduleId) + "/" + PARAM_PUMP_ON);
    myMqtt.SetParameterIsCommand(storage.moduleId, PARAM_PUMP_ON, true);


    // create Sensor.Parameter4
    // Sensor.Parameter4 - current soil humidity
    Serial.println("new parameter: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY);
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_HUMIDITY);
    // set Description
    Serial.println("set description: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY);
    myMqtt.SetParameterDescription(storage.moduleId, PARAM_HUMIDITY, "Soil moist.");
    // set Unit
    Serial.println("set Unit: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY);
    myMqtt.SetParameterUnit(storage.moduleId, PARAM_HUMIDITY, "%");
    // set dbLogging
    Serial.println("set Unit: /" + String(storage.moduleId) + "/" + PARAM_HUMIDITY);
    myMqtt.SetParameterDBLogging(storage.moduleId, PARAM_HUMIDITY, true);

    // Guarda as novas configurações na EEPROM
    saveConfig();
  }

  subscribe();

  // Lê o primeiro valor a partir do pin analogico
  lastAnalogReading = analogRead(PIN_HUM_ANALOG);

  // Define o modo automatico antigo como o contrário do atual definido em cima
  autoModeOld = !autoMode;
}

void loop() {
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
#ifdef DEBUG
    Serial.print(".");
#endif
  }

  int in = digitalRead(PIN_BUTTON);

  //Serial.println(in);
  if (in == 0)
  {
    while (digitalRead(PIN_BUTTON) == 0)
      delay(100);

    if (state == s_idle || state == s_irrigation_start)
      state = s_irrigation_start;
    else
      state = s_irrigation_stop;
  }



  // post auto mode changes
  if (autoModeOld != autoMode)
  {
    autoModeOld = autoMode;

    if (autoMode)
      valueStr = String("1");
    else
      valueStr = String("0");

    topic  = "/" + String(storage.moduleId) + "/" + PARAM_MANUAL_AUTO_MODE;
    result = myMqtt.publish(topic, valueStr, 0, 1);

    Serial.print("Publish topic: ");
    Serial.print(topic);
    Serial.print(" value: ");
    Serial.println(valueStr);
  }

  // post treshold changes
  if (soilHumidityThreshold != soilHumidityThresholdOld)
  {
    soilHumidityThresholdOld = soilHumidityThreshold;
    valueStr = String(soilHumidityThreshold);

    topic  = "/" + String(storage.moduleId) + "/" + PARAM_HUMIDITY_TRESHOLD;
    result = myMqtt.publish(topic, valueStr, 0, 1);

    Serial.print("Publish topic: ");
    Serial.print(topic);
    Serial.print(" value: ");
    Serial.println(valueStr);
  }

  if (IsTimeout())
  {
    startTime = millis();
    // process every second
    int aireading = analogRead(PIN_HUM_ANALOG);

    Serial.print("Analog value: ");
    Serial.print(aireading);
    Serial.print(" ");
    // filter s
    lastAnalogReading += (aireading - lastAnalogReading) / 10;
    Serial.print(lastAnalogReading);

    // calculate soil humidity in %
    int newSoilHum = map(lastAnalogReading, MIN_ANALOG_VAL, MAX_ANALOG_VAL, 100, 0);
    Serial.print(", Soil hum %:");
    Serial.println(newSoilHum);

    // limit to 0-100%
    if (newSoilHum < 0)
      newSoilHum = 0;

    if (newSoilHum > 100)
      newSoilHum = 100;

    // report soil humidity if changed
    if (soilHum != newSoilHum)
    {
      soilHum = newSoilHum;
      //esp.send(msgHum.set(soilHum));

      valueStr = String(soilHum);
      topic  = "/" + String(storage.moduleId) + "/" + PARAM_HUMIDITY;
      result = myMqtt.publish(topic, valueStr, 0, 1);

      Serial.print("Publish topic: ");
      Serial.print(topic);
      Serial.print(" value: ");
      Serial.println(valueStr);
    }


    // irrigator state machine
    switch (state)
    {
      case s_idle:
        if (irrigatorCounter <= IRRIGATION_PAUSE_TIME)
          irrigatorCounter++;

        if (irrigatorCounter >= IRRIGATION_PAUSE_TIME && autoMode)
        {
          if (soilHum <= soilHumidityThreshold)
            state = s_irrigation_start;
        }
        break;
      case s_irrigation_start:
        irrigatorCounter = 0;
        digitalWrite(PIN_PUMP, HIGH);
        //esp.send(msgMotorPump.set((uint8_t)1));
        valueStr = String(1);
        topic  = "/" + String(storage.moduleId) + "/" + PARAM_PUMP_ON;
        result = myMqtt.publish(topic, valueStr, 0, 1);

        Serial.print("Publish topic: ");
        Serial.print(topic);
        Serial.print(" value: ");
        Serial.println(valueStr);

        state = s_irrigation;
        break;
      case s_irrigation:
        if (irrigatorCounter++ > IRRIGATION_TIME)
          state = s_irrigation_stop;
        break;
      case s_irrigation_stop:
        irrigatorCounter = 0;
        state = s_idle;
        //esp.send(msgMotorPump.set((uint8_t)0));
        valueStr = String(0);
        topic  = "/" + String(storage.moduleId) + "/" + PARAM_PUMP_ON;
        result = myMqtt.publish(topic, valueStr, 0, 1);

        digitalWrite(PIN_PUMP, LOW);
        break;
    }
  }

}

/*
   A função loadConfig vai carregar as configurações lá guardadas
*/
void loadConfig() {
  // Precisamos de verificar se a versão na memoria corresponde à nossa. É uma maneira fácil de verificar se esta foi corrompida ou não
  // Se as versões não corresponderem vão ser usados os valores de default
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
    for (unsigned int t = 0; t < sizeof(storage); t++)
      *((char*)&storage + t) = EEPROM.read(CONFIG_START + t);
}

/*
   A função saveConfig vai guardar as configurações na EEPROM
*/
void saveConfig() {
  for (unsigned int t = 0; t < sizeof(storage); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&storage + t));

  EEPROM.commit();
}

/*
   A função macToStr vai transformar o endereço mac numa string
*/
String macToStr(const uint8_t* mac)
{
  String result; //explicar que as funções dão prioridade de nome as variaveis locais
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}


/*
   A função IsTimeout vai verificar se já passou mais de 1 sec
*/
boolean IsTimeout()
{
  unsigned long now = millis();
  if (startTime <= now)
  {
    if ( (unsigned long)(now - startTime )  < MS_IN_SEC )
      return false;
  }
  else
  {
    if ( (unsigned long)(startTime - now) < MS_IN_SEC )
      return false;
  }

  return true;
}


/*
   A função subscribe vai ...
*/
void subscribe()
{
  if (storage.moduleId != 0)
  {
    // Sensor.Parameter1 - humidity treshold value
    myMqtt.subscribe("/" + String(storage.moduleId) + "/" + PARAM_HUMIDITY_TRESHOLD);

    // Sensor.Parameter2 - manual/auto mode 0 - manual, 1 - auto mode
    myMqtt.subscribe("/" + String(storage.moduleId) + "/" + PARAM_MANUAL_AUTO_MODE);

    // Sensor.Parameter3 - pump on/ pump off
    myMqtt.subscribe("/" + String(storage.moduleId) + "/" + PARAM_PUMP_ON);
  }
}

/*
   A função myConnectedCb vai executar quando o nodeMCU establecer a conexão por MQTT à cloud
*/
void myConnectedCb() {
#ifdef DEBUG
  Serial.println("Connected to MQTT server");
#endif
  subscribe();
}

/*
   A função myDisconnectedCb vai executar quando o nodeMCU se deconectar da ligação por MQTT à cloud
*/
void myDisconnectedCb() {
#ifdef DEBUG
  Serial.println("disconnected. try to reconnect...");
#endif
  delay(500);
  myMqtt.connect();
}

/*
   A função myPublishedCb vai executar quando o nodeMCU enviar algo para a cloud
*/
void myPublishedCb() {
#ifdef DEBUG
  Serial.println("published.");
#endif
}

/*
   A função myDataCb vai executar quando o nodeMCU receber dados pela conexão MQTT à cloud
*/
void myDataCb(String& topic, String& data) {
#ifdef DEBUG
  Serial.print("Receive topic: ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(data);
#endif
  if (topic == String("/" + String(storage.moduleId) + "/" + PARAM_HUMIDITY_TRESHOLD)) // Executa se a mensagem que receber for relativa a uma variação da humidade limiar do solo
  {
    soilHumidityThreshold = data.toInt();
    Serial.println("soilHumidityThreshold");
    Serial.println(data);
  }

  else if (topic == String("/" + String(storage.moduleId) + "/" + PARAM_MANUAL_AUTO_MODE)) // Executa se a mensagem que receber for relativa a ligar ou desligar o modo automatico
  {
    autoMode = (data == String("1"));
    Serial.println("Auto mode");
    Serial.println(data);
  }
  else if (topic == String("/" + String(storage.moduleId) + "/" + PARAM_PUMP_ON)) // Executa se a mensagem que receber for relativa a ligar ou desligar a bomba de aguá
  {
    if (data == String("1"))
      state = s_irrigation_start;
    else
      state = s_irrigation_stop;
    Serial.println("Pump");
    Serial.println(data);
  }
}
