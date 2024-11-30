// secrets.h
#ifndef SECRETS_H
#define SECRETS_H

#define WIFI_SSID "tu_ssid"
#define WIFI_PASSWORD "tu_contraseña_wifi"

// AWS IoT Configuración
#define AWS_IOT_ENDPOINT "tu_endpoint.iot.us-east-1.amazonaws.com"
#define MQTT_USER "tu_usuario"
#define MQTT_PASSWORD "tu_contraseña"

// Certificados para conectar al endpoint de AWS IoT (estos valores son solo ejemplos)
#define AWS_CERT_CA "ruta_al_certificado_CA.pem"
#define AWS_CERT_CRT "ruta_al_certificado_cliente.crt"
#define AWS_CERT_PRIVATE "ruta_al_certificado_cliente_private.key"

#endif
