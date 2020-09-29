#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>

extern "C" {
  #include "espconn.h"
}

// Nombre max de clients 
#define MAX_SRV_CLIENT 5
// Taille du buffer pour les commandes
#define BUFSIZE 32
// Angle d'un pas
#define STEPANGLE 0.225 // 0.9 // 0.225 EasyDriver microstepping default 1.8

// Assignation des GPIO
#define AZSTEP D2
#define AZDIR  D1
#define ELSTEP D4
#define ELDIR  D3

/*
#define AZSTEP D4
#define AZDIR  D3
#define ELSTEP D2
#define ELDIR  D1
*/

// configuration en EEPROM
struct EEconf {
  char ssid[32];
  char password[64];
  char myhostname[32];
};
EEconf readconf;

// mot de passe pour l'OTA
const char* otapass = "123456";
// gestion du temps pour calcul de la durée de la MaJ
unsigned long otamillis;

// Utilisation classique de millis()
unsigned long previousMillis = 0;

// Tableau pour représenter les clients
WiFiClient clients[MAX_SRV_CLIENT];
// Notre serveur
WiFiServer server(4533);
// Nombre de client (vérification)
int max_srv_client = MAX_SRV_CLIENT;

// Positions courantes
int pos_az = 0;
int pos_el = 0;
// Positions cibles
int tpos_az = 0;
int tpos_el = 0;

void confOTA() {
  // Port 8266 (défaut)
  ArduinoOTA.setPort(8266);

  // Hostname défaut : esp8266-[ChipID]
  ArduinoOTA.setHostname(readconf.myhostname);

  // mot de passe pour OTA
  ArduinoOTA.setPassword(otapass);

  ArduinoOTA.onStart([]() {
    Serial.println("/!\\ Maj OTA");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\n/!\\ MaJ terminee");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progression: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

void setup() {
  // GPIO
  pinMode(AZDIR, OUTPUT);
  pinMode(AZSTEP, OUTPUT);
  pinMode(ELDIR, OUTPUT);
  pinMode(ELSTEP, OUTPUT);
  
  // Moniteur série
  Serial.begin(115200);
  Serial.println();

  // Lecture configuration Wifi
  EEPROM.begin(sizeof(readconf));
  EEPROM.get(0, readconf);

  // Connexion au Wifi
  Serial.print("Wifi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(readconf.ssid, readconf.password);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print(" IP:");
  Serial.println(WiFi.localIP());

  // configuration OTA
  confOTA();

  if(MAX_SRV_CLIENT > espconn_tcp_get_max_con()) {
    max_srv_client = espconn_tcp_get_max_con();
  }

  server.begin();
  server.setNoDelay(true);
  Serial.printf("Attente de clients (max=%d) sur 4355\r\n",
    max_srv_client);
}

void loop() {
  // buffer pour les commandes
  char clientline[BUFSIZE];
  // caractère pour "K", "p", et "_"
  char ch;
  // valeur azimut et élévation
  float in_az = 0;
  float in_el = 0;
  // valeur reset
  int rst = 0;
  // itération
  int i;

  /* Gestion des nouvelles connexions */
  // Un nouveau client ?
  if (server.hasClient()) {
    // trouver un emplacement libre
    for (i=0; i < max_srv_client; i++) {
      if (!clients[i].connected()) {
        clients[i] = server.available();
        Serial.printf("Nouveau client (%d) depuis %s\r\n",
          i, clients[i].remoteIP().toString().c_str());
        break;
      }
    }
    // Pas de place, on refuse
    if (i == max_srv_client) {
      server.available().println("busy");
      // server.available() est un objet WiFiClient temporaire.
      // Dès qu'on quitte la portée où il existe, les méthodes 
      // flush() et stop() sont utilisées automatiquement
      Serial.printf("server is busy with %d active connections\r\n", i);
    }
  }

  /* Gestion des messages reçus */
  // On fait le tour des clients ?
  for (i=0; i < max_srv_client; i++) {
    if (clients[i] && clients[i].connected()) {
      if (clients[i].available()) {
        // on a des données à lire
        // stockage dans un tableau de char C
        String line = clients[i].readStringUntil('\n');
        line.trim();
        line.toCharArray(clientline, sizeof(clientline));

        if(sscanf(clientline, "P %f %f", &in_az, &in_el) == 2) {
          // Analyse pour la comment P avec deux float
          Serial.printf("in val: %f   %f", in_az, in_el);
          if(in_az > 180 || in_az < -180 || in_el > 90 || in_el < 0) {
            // hors limites !
            clients[i].printf("RPRT -1\n");
            Serial.printf(" -> tpos: INVALID!\r\n", tpos_az, tpos_el);
          } else {
            // mise à jour position cible
            tpos_az = in_az/STEPANGLE;
            tpos_el = in_el/STEPANGLE;
            Serial.printf(" -> tpos: %d   %d\r\n", tpos_az, tpos_el);
            clients[i].printf("RPRT 0\n");
          }
        } else if(sscanf(clientline, "R %d", &rst) == 1) {
          // Commande R pout reset
          if(rst == 1) {
            tpos_az = tpos_el = pos_az = pos_el = 0;
          }
          clients[i].printf("RPRT 0\n");
        } else if(sscanf(clientline, " %c", &ch) >= 1) {
          // Analyse des commandes d'un caractère
          if(ch == 'q' || ch == 'Q' ) {
            // quitter
            clients[i].flush();
            clients[i].stop();
            Serial.printf("Client %d quit\r\n", i);
          } else if(ch == 'p') {
            // lecture position
            Serial.printf("out val: %d   %d -> %f   %f\r\n", pos_az, pos_el, pos_az*STEPANGLE, pos_el*STEPANGLE);
            clients[i].printf("%.6f\n%.6f\n", pos_az*STEPANGLE, pos_el*STEPANGLE);
          } else if(ch == 'K') {
            // parcage antenne/pointeur
            tpos_az = 0;
            tpos_el = 0;
          } else if(ch == '_') {
            // info
            clients[i].printf("ESP8266 rotor\n");
            Serial.printf("Client info\r\n");
          } else {
            // Commande inconnue
            clients[i].printf("RPRT -1\n");
          }
        } else {
          // Commande inconnue
          clients[i].printf("RPRT -1\n");
          Serial.printf("syntax error\r\n");
        }
      }
    }
  }

  /* Gestion des moteurs et des positions */
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1) {
    previousMillis = currentMillis;
    if(pos_az < tpos_az) {
      digitalWrite(AZDIR, HIGH);   // inversé !
      digitalWrite(AZSTEP, HIGH);
      delayMicroseconds(10);
      digitalWrite(AZSTEP, LOW);
      pos_az++;
    }
    if(pos_az > tpos_az) {
      digitalWrite(AZDIR, LOW);
      digitalWrite(AZSTEP, HIGH);
      delayMicroseconds(10);
      digitalWrite(AZSTEP, LOW);
      pos_az--;
    }
    if(pos_el < tpos_el) {
      digitalWrite(ELDIR, LOW);
      digitalWrite(ELSTEP, HIGH);
      delayMicroseconds(10);
      digitalWrite(ELSTEP, LOW);
      pos_el++;
    }
    if(pos_el > tpos_el) {
      digitalWrite(ELDIR, HIGH);
      digitalWrite(ELSTEP, HIGH);
      delayMicroseconds(10);
      digitalWrite(ELSTEP, LOW);
      pos_el--;
    }
  }

  // gestion OTA
  ArduinoOTA.handle();
}
