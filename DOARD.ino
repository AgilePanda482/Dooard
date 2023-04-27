//Libreria para la comuniciacion del ESP32 y Telegram
#include <CTBot.h>
//Informacion sencible de conexiones a internet y Telegram
#include </home/david/Documentos/ProgramasDeArduino/DOARD/librerias/token.h>

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

CTBot DOARD;
const byte botonPin = 14;
const byte buzzPin = 13;
byte leerBoton;
const byte dt = 50;

void setup(){
    Serial.begin(115200);
    Serial.println("Iniciando...");

    pinMode(botonPin, INPUT);
    pinMode(buzzPin, OUTPUT);
    
    //Conexion a internet y a telegram
    DOARD.wifiConnect(ssid, password);
    DOARD.setTelegramToken(token);
    
    //Detecta si todo a salido bien. En caso de 15 intentos fallidos de conexion, informara del error
    if(DOARD.testConnection() == true){
        Serial.println("\n Conectado Correctamente");
        DOARD.sendMessage(IDchat, "En Linea correctamente");
    }
    else{
        Serial.println("\n No se puede conectar a la red WiFi");
        return;
    }
}

void loop(){
    //Variable necesaria para enviar mensajes
    TBMessage msg;
    
    leerBoton = digitalRead(botonPin);
    Serial.println(leerBoton);
    delay(dt);

    //Si el boton se aprieta, se activara el Buzzer y mandara un mensaje a telegram.
    if (leerBoton == 0) {
        digitalWrite(buzzPin, HIGH);
        DOARD.sendMessage(IDchat, "Actividad detectada");
        delay(dt);
    }
    else{
        digitalWrite(buzzPin, LOW);
    }
}
