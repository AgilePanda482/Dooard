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
//Pines para MicroSD
#define CLK 14
#define CMD 15
#define DATA0 2

WiFiClientSecure clientTCP;
UniversalTelegramBot DOARD(token, clientTCP);

const byte botonPin = 12;
const byte buzzPin = 13;
const byte dt = 50;
byte leerBoton;
bool mandarFoto = false;

//Revisa por un nuevo mensaje cada 1 segundo
const int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

void setup(){
    Serial.begin(115200);
    Serial.println("Iniciando...");

    pinMode(botonPin, INPUT_PULLUP);
    pinMode(buzzPin, OUTPUT);
    pinMode(33, OUTPUT);

    configuracionInicialCamara();


    WiFi.mode(WIFI_STA);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println();
    Serial.print("ESP32-CAM IP Address: ");
    Serial.println(WiFi.localIP()); 
    
    //Detecta si todo a salido bien. En caso de 15 intentos fallidos de conexion, informara del error
    /*if(DOARD.testConnection() == true){
        Serial.println("\n Conectado Correctamente");
        DOARD.sendMessage(IDchat, "En Linea correctamente");
    }
    else{
        Serial.println("\n No se puede conectar a la red WiFi");
        return;
    }*/
}

void loop(){
    digitalWrite(33, LOW);
    //Variable necesaria para enviar mensajes
    
    leerBoton = digitalRead(botonPin);
    Serial.println(leerBoton);
    delay(dt);

    //Si el boton se aprieta, se activara el Buzzer y mandara un mensaje a telegram.
    /*if (leerBoton == 0) {
        digitalWrite(buzzPin, HIGH);
        DOARD.sendMessage(IDchat, "Actividad detectada");
        delay(dt);
    }
    else{
        digitalWrite(buzzPin, LOW);
    }*/

    if (mandarFoto) {
      Serial.println("Preparing photo");
      enviarFotoTelegram(); 
      mandarFoto = false;
    }
    if (millis() > lastTimeBotRan + botRequestDelay) {
      int numNuevosMensajes = DOARD.getUpdates(DOARD.last_message_received + 1);
      while (numNuevosMensajes) {
        Serial.println("got response");
        gestionarNuevosMensajes(numNuevosMensajes);
        numNuevosMensajes = DOARD.getUpdates(DOARD.last_message_received + 1);
      }
      lastTimeBotRan = millis();
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
        config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
        config.jpeg_quality = 10;
        config.fb_count = 2;

    //En caso contrario:
    }else {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

    //Iniciar camara
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Error al iniciar la camara con codigo de error 0x%x", err);
        return;
    }

    //Disminuye el tamaño de los fotogramas para aumentar la velocidad de fotogramas inicial
    sensor_t * s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_CIF);  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
}

/*---------------------------------------------------------*/

void gestionarNuevosMensajes(int numNuevosMensajes) {
  Serial.print("Gestionando Nuevos mensajes: ");
  Serial.println(numNuevosMensajes);

  for (int i = 0; i < numNuevosMensajes; i++) {
    String chat_id = String(DOARD.messages[i].chat_id);
    if (chat_id != IDchat){
      DOARD.sendMessage(chat_id, "Usuario NO autorizado", "");
      continue;
    }
    
    // Print the received message
    String text = DOARD.messages[i].text;
    Serial.println(text);
    
    String from_name = DOARD.messages[i].from_name;
    if (text == "/start") {
      String welcome = "Welcome , " + from_name + "\n";
      welcome += "Use the following commands to interact with the ESP32-CAM \n";
      welcome += "/photo : takes a new photo\n";
      welcome += "/flash : toggles flash LED \n";
      DOARD.sendMessage(IDchat, welcome, "");
    }
    if (text == "/photo") {
      mandarFoto = true;
      Serial.println("New photo request");
    }
  }
}

/*---------------------------------------------------------------*/

String enviarFotoTelegram() {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Captura de Camara Fallida");
    delay(1000);
    ESP.restart();
    return "Captura de Camara Fallida";
  }  
  
  Serial.println("Conectando hacia " + String(myDomain));


  if (clientTCP.connect(myDomain, 443)) {
    Serial.println("Conexión Exitosa");
    
    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"IDchat\"; \r\n\r\n" + IDchat + "\r\n--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;
  
    clientTCP.println("POST /bot"+token+"/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    clientTCP.println();
    clientTCP.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0;n<fbLen;n=n+1024) {
      if (n+1024<fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        clientTCP.write(fbBuf, remainder);
      }
    }  
    
    clientTCP.print(tail);
    
    esp_camera_fb_return(fb);
    
    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + waitTime) > millis()){
      Serial.print(".");
      delay(100);      
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state==true) getBody += String(c);        
        if (c == '\n') {
          if (getAll.length()==0) state=true; 
          getAll = "";
        } 
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length()>0) break;
    }
    clientTCP.stop();
    Serial.println(getBody);
  }
  else {
    getBody="Connected to api.telegram.org failed.";
    Serial.println("Connected to api.telegram.org failed.");
  }
  return getBody;
}