// alexa-skill.js
const Alexa = require('ask-sdk-core');
const AWS = require('aws-sdk');

// Configuración del cliente AWS IoT y DynamoDB
const IotData = new AWS.IotData({ endpoint: 'aea4fbhz5gq4c-ats.iot.us-east-2.amazonaws.com' });
const dynamoDB = new AWS.DynamoDB.DocumentClient();

// Función para obtener el nombre del dispositivo de DynamoDB
async function getThingNameFromDynamoDB() {
    const params = {
        TableName: 'env_control_data'
    };

    try {
        const data = await dynamoDB.scan(params).promise();
        if (data.Items && data.Items.length > 0) {
            return data.Items[0].thing_name;
        }
        throw new Error('No se encontró un nombre de dispositivo en DynamoDB');
    } catch (error) {
        console.error('Error al obtener el nombre del dispositivo:', error);
        throw error;
    }
}

// Handler para obtener la temperatura y otros valores del dispositivo
const CheckTemperatureIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'CheckTemperatureIntent';
    },
    async handle(handlerInput) {
        let temperature = 'desconocida';
        try {
            const result = await getShadowPromise(ShadowParams);
            console.log('Shadow recibido para temperatura:', JSON.stringify(result)); 
            
            if (result.state && result.state.reported) {
                console.log('Estado reportado:', JSON.stringify(result.state.reported)); 
                if (typeof result.state.reported.temperature !== 'undefined') {
                    temperature = result.state.reported.sensors.temperature;
                }
            }
        } catch (error) {
            console.error('Error al consultar la temperatura:', error);
            return handlerInput.responseBuilder
                .speak('No pude obtener la temperatura en este momento. Por favor, intenta más tarde.')
                .getResponse();
        }

        const speakOutput = `La temperatura actual es de ${temperature} grados.`;
        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt('¿Hay algo más con lo que te pueda ayudar?')
            .getResponse();
    }
};

// Parámetros dinámicos para controlar el ventilador, motor y la alarma juntos
async function createFunctionParms(actuadores) {
    return {
        payload: {
            actuadores,
            functionName: 'actuadoresEnFuncion',
        }
    };
}
