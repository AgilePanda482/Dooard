//Libreria para la comuniciacion del ESP32 y Telegram
#include <CTBot.h>
//Informacion sencible de conexiones a internet y Telegram
#include <//home/david/Documentos/ProgramasDeArduino/DOARD/info.h>

/*Plantilla que permite imprimir informacion como en C++ sin recurrir al ciclo FOR
Ejemplo: Serial << "Hola mundo!"*/
template<class T> inline Print &operator <<(Print &obj, T arg) {
    obj.print(arg);
    return obj;
}

CTBot DOARD;
const byte botonPin = 35;
const byte buzzPin = 33;
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
