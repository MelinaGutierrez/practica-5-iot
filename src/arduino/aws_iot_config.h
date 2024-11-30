#ifndef AWS_IOT_CONFIG_H
#define AWS_IOT_CONFIG_H

// Configuración del endpoint de AWS IoT
#define AWS_IOT_ENDPOINT "tu-endpoint.iot.us-east-1.amazonaws.com"

// Puerto MQTT para la conexión segura (SSL/TLS)
#define AWS_IOT_MQTT_PORT 8883

// Nombre de cosa (Thing) en AWS IoT
#define AWS_IOT_THING_NAME "Env_Control_0001"

// Identificadores de los certificados para la conexión con AWS IoT
#define AWS_IOT_CERTIFICATE "ruta_al_certificado_cliente.crt"
#define AWS_IOT_PRIVATE_KEY "ruta_al_certificado_cliente_private.key"
#define AWS_IOT_CA_CERTIFICATE "ruta_al_certificado_CA.pem"

// Definir los topics de MQTT que se utilizarán para la comunicación con AWS IoT
#define AWS_IOT_UPDATE_TOPIC "$aws/things/Env_Control_0001/shadow/update"
#define AWS_IOT_DELTA_TOPIC "$aws/things/Env_Control_0001/shadow/update/delta"

// Configuración de credenciales para la autenticación en AWS IoT (opcional si usas claves)
#define AWS_IOT_MQTT_USERNAME "tu_usuario_mqtt"
#define AWS_IOT_MQTT_PASSWORD "tu_contraseña_mqtt"

#endif // AWS_IOT_CONFIG_H
