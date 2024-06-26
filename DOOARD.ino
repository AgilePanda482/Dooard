//Libreria para la comuniciacion del ESP32 y Telegram
#include <UniversalTelegramBot.h>
//Informacion sencible de conexiones a internet y Telegram
#include </home/david/Documentos/ProgramasDeArduino/DOARD/librerias/token.h>

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <ArduinoJson.h>

/*Plantilla que permite imprimir informacion como en C++ sin recurrir al ciclo FOR
Ejemplo: Serial << "Hola mundo!"*/
template<class T> inline Print &operator <<(Print &obj, T arg) {
    obj.print(arg);
    return obj;
}

//Modelo de la camara
#define CAMERA_MODEL_AI_THINKER

//Pines para controlar la camara
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22
#define FLASH_LED_PIN 4

WiFiClientSecure clientTCP;
UniversalTelegramBot DOOARD(token, clientTCP);

//Revisa por un nuevo mensaje cada 1 segundo
const int botRequestDelay = 1000;
const byte botonPin = 12;
const byte buzzPin = 13;
const byte dt = 100;
byte leerBoton;
bool mandarFoto = false;
bool EstadoFlash = LOW;
unsigned long lastTimeBotRan;

void configuracionInicialCamara();
void gestionarNuevosMensajes(int numNuevosMensajes);
String enviarFotoTelegram();


void setup(){
  Serial.begin(115200);
  Serial << "Iniciando...";

  pinMode(FLASH_LED_PIN, OUTPUT);
  
  pinMode(botonPin, INPUT_PULLUP);
  pinMode(buzzPin, OUTPUT);
  pinMode(33, OUTPUT);

  configuracionInicialCamara();

  //Declaro que el ESP32 se conectara a una red
  WiFi.mode(WIFI_STA);
  Serial << "\n" << "Conectando a " << ssid;
  WiFi.begin(ssid, password);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Agregando certificacion root para api.telegram.org

  //Conexion hacia internet...
  while(WiFi.status() != WL_CONNECTED) {
    Serial << ".";
    delay(500);
  }
  Serial << "\n" << "ESP32-CAM dirección IP: " << WiFi.localIP();

  
}

void loop(){
  //Prender led rojo del ESP32-CAM
  digitalWrite(33, LOW);
    
  //Serial.println(leerBoton);
  //delay(dt);

    //Si el boton se aprieta, se activara el Buzzer y mandara un mensaje a telegram.

  if (mandarFoto) {
    DOOARD.sendMessage(IDchat, "Preparando foto..."); 
    enviarFotoTelegram();
    mandarFoto = false;
  }
  //Retraso para buscar nuevos mensajes en telegram
  if (millis() > lastTimeBotRan + botRequestDelay) {
    int numNuevosMensajes = DOOARD.getUpdates(DOOARD.last_message_received + 1);
    while (numNuevosMensajes) {
      Serial.println("got response");
      gestionarNuevosMensajes(numNuevosMensajes);
      numNuevosMensajes = DOOARD.getUpdates(DOOARD.last_message_received + 1);
    }

    lastTimeBotRan = millis();
  }
  leerBoton = digitalRead(botonPin);

  if (leerBoton == 0) {
    digitalWrite(buzzPin, HIGH);
    DOOARD.sendMessage(IDchat, "Actividad detectada");
    delay(dt);
    mandarFoto = true;
    digitalWrite(buzzPin, LOW);
    }
    else{
     digitalWrite(buzzPin, LOW);
    }

}

/*-------------------------------------------------------------------*/

void configuracionInicialCamara(){

  //configuración de la cámara en un objeto camera_config_t
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  //Configuraciónes especiales si la placa tiene PSRAM:
  if(psramFound()){
    config.frame_size = FRAMESIZE_XGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
    //En caso contrario:
  }else{
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  //Iniciar camara
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Error al iniciar la camara con codigo de error 0x%x", err);
    //DOOARD.sendMessage(IDchat, "¡Error!", "");
    return;
  }

  //Disminuye el tamaño de los fotogramas para aumentar la velocidad de fotogramas inicial
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
}

/*---------------------------------------------------------*/

void gestionarNuevosMensajes(int numNuevosMensajes) {
  Serial << "Gestionando Nuevos mensajes: " << numNuevosMensajes;

  //un ciclo for que se repetira dependiendo de los mensajes en telegram
  for (int i = 0; i < numNuevosMensajes; i++) {
    //Guardamos el chat id y lo comparamos
    String chat_id = String(DOOARD.messages[i].chat_id);
    //En caso de que el usuario no sea el correcto, volvemos a iterar
    if (chat_id != IDchat){
      DOOARD.sendMessage(chat_id, "Usuario NO autorizado", "");
      continue;
    }
    
    // Imprimimos el mensaje recibido
    String text = DOOARD.messages[i].text;
    Serial << text;
    
    //from_name es el nombre de usuario en telegram
    String from_name = DOOARD.messages[i].from_name;
    if (text == "/start") {
      DOOARD.sendMessage(IDchat, "Bienvenido " + from_name, "");
      delay(500);
      DOOARD.sendMessage(IDchat, "Usa los siguientes comandos para interactuar con el ESP32-CAM: ", "");
      delay(500);
      DOOARD.sendMessage(IDchat, "/foto : toma una nueva foto");
      delay(250);
      DOOARD.sendMessage(IDchat, "/flash : Apaga o enciende el flash");
      delay(500);
    }
    if (text == "/flash") {
      if(!EstadoFlash == true){
        DOOARD.sendMessage(IDchat, "Activando Flash");
      }else{
        DOOARD.sendMessage(IDchat, "Apagando Flash");
      }
      EstadoFlash = !EstadoFlash;
      digitalWrite(FLASH_LED_PIN, EstadoFlash);
    }
    if (text == "/foto") {
      mandarFoto = true;
    }
  }
}

/*---------------------------------------------------------------*/

String enviarFotoTelegram() {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";
  //Puntero fb de freambuffer declarado como null
  camera_fb_t *fb = NULL;
  const size_t tamanioBuffer = 2048;
  
  Serial.println("Connect to " + String(myDomain));


  if (clientTCP.connect(myDomain, 443)) {
    fb = esp_camera_fb_get();

    if(!fb) {
      delay(1000);
      DOOARD.sendMessage(IDchat, "¡Error al tomar la foto!");
      DOOARD.sendMessage(IDchat, "Vuelve a intentarlo.");
      ESP.restart();
      return "Captura de Camara Fallida";
    }  
    
    //???
    String head = "--Electro\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + IDchat + "\r\n--Electro\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--Electro--\r\n";

    //longitud de los datos de la imagen capturada por la cámara.
    uint16_t imageLen = fb->len;
    //ongitud de los datos adicionales que se enviarán con la imagen.
    uint16_t extraLen = head.length() + tail.length();
    //longitud total de los datos que se enviarán.
    uint16_t totalLen = imageLen + extraLen;

    //Se construye el encabezado HTTP que se usara para enviar imagenes
    clientTCP.println("POST /bot"+token+"/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=Electro");
    clientTCP.println();
    clientTCP.print(head);
  

    //puntero a la memoria del búfer de imagen
    uint8_t *fbBuf = fb->buf;
    //tamaño del búfer de imagen en bytes
    size_t fbLen = fb->len;

    for (size_t n = 0; n < fbLen; n += tamanioBuffer) {
      if (n + tamanioBuffer < fbLen) {
        clientTCP.write(fbBuf, tamanioBuffer);
        fbBuf += tamanioBuffer;

      } else if (fbLen % tamanioBuffer > 0) {

        size_t remainder = fbLen % tamanioBuffer;
        clientTCP.write(fbBuf, remainder);
      }
    }

    clientTCP.print(tail);
      
    //liberamos memoria
    esp_camera_fb_return(fb);
      
    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;
    
    // Espera y procesamiento de los datos del cliente TCP
    while ((startTimer + waitTime) > millis()){
      Serial.print(".");
      delay(100);      
    
      while (clientTCP.available()) {
        char c = clientTCP.read();
        
        if (state==true) {
          getBody += String(c);        
        }
        if (c == '\n') {
          if (getAll.length() == 0) {
            state = true; 
          } 
          getAll = "";
        } 
        else if (c != '\r') {
          getAll += String(c);
        }
        startTimer = millis();
      }
      if (getBody.length() > 0) {
        break;
      }
    }
    clientTCP.stop();
    Serial.println(getBody);
  } else {
    getBody="Connected to api.telegram.org failed.";
    Serial.println("Connected to api.telegram.org failed.");
  }

  esp_camera_fb_return(fb);
  return getBody;

}