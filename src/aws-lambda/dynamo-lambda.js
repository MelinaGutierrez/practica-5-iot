// dynamo-lambda.js
import { DynamoDBClient } from "@aws-sdk/client-dynamodb";
import { PutCommand, DynamoDBDocumentClient } from "@aws-sdk/lib-dynamodb";

const client = new DynamoDBClient({});
const docClient = DynamoDBDocumentClient.from(client, {
    marshallOptions: {
        removeUndefinedValues: true, 
    },
});

export const handler = async (event) => {
    // Extraer el estado deseado de los actuadores
    const desiredState = event.state?.desired?.actuators || {};
    
    const command = new PutCommand({
        TableName: "env_control_data",
        Item: {
            timeStamp: event.timestamp,
            thing_name: event.thing_name,
            sn: event.sn,
            temperature: event.temperature,
            humidity: event.humidity,
            actuators: desiredState, 
        },
    });
    

    try {
        const response = await docClient.send(command);
        console.log("Datos guardados en DynamoDB:", response);
    } catch (error) {
        console.error("Error al guardar datos en DynamoDB:", error);
    }
};
