import boto3
import uuid
from decimal import Decimal
import datetime
import json
from boto3.dynamodb.conditions import Key


iot_client = boto3.client('iot-data')

def publish_to_mqtt_topic(topic, payload):
    iot_client.publish(
        topic=topic,
        qos=1,
        payload=json.dumps(payload)
    )

def lambda_handler(event, context):
    s3 = boto3.client('s3')
    rekognition = boto3.client('rekognition')
    dynamodb = boto3.resource('dynamodb')
    table = dynamodb.Table('details_lambda_images')

    bucket_fotos_recientes = 'images3fromiot'
    bucket_fotos_trabajadores = 'images3users'

    fotos_recientes = s3.list_objects_v2(Bucket=bucket_fotos_recientes)

    matches = []
    match_status = 0

    for foto in fotos_recientes.get('Contents', []):
        trabajadores = s3.list_objects_v2(Bucket=bucket_fotos_trabajadores)

        for trabajador in trabajadores.get('Contents', []):
            print(f"Comparando {foto['Key']} con {trabajador['Key']}")
            try:
                response = rekognition.compare_faces(
                    SourceImage={
                        'S3Object': {
                            'Bucket': bucket_fotos_recientes,
                            'Name': foto['Key']
                        }
                    },
                    TargetImage={
                        'S3Object': {
                            'Bucket': bucket_fotos_trabajadores,
                            'Name': trabajador['Key']
                        }
                    }
                )
            except Exception as e:
                print(f"Error al comparar {foto['Key']} con {trabajador['Key']}: {str(e)}")
                continue  # Continuar con la siguiente comparación
    
            if response['FaceMatches']:
                current_time = datetime.datetime.now()
                current_date_str = current_time.strftime("%Y-%m-%d")
                current_time_iso = current_time.isoformat()
    
                # Consultar DynamoDB para verificar registros previos del mismo trabajador en el día actual
                # (Asegúrate de que tu tabla DynamoDB esté configurada correctamente para soportar esta consulta)
                response_dynamodb = table.query(
                    IndexName='tuIndiceSecundario',
                    KeyConditionExpression=Key('FotoTrabajador').eq(trabajador['Key']) &
                                            Key('Date').eq(current_date_str)
                )
    
                match_status = 2 if response_dynamodb['Items'] else 1
    
                match = {
                    'FotoTrabajador': trabajador['Key'],
                    'Confianza': Decimal(str(response['FaceMatches'][0]['Similarity'])),
                    'Timestamp': current_time_iso,
                    'Status': match_status
                }
                matches.append(match)
                break
    
        if match_status != 0:
            break
    
        
    # Almacenar resultados en DynamoDB
    if matches:
        comparison_id = str(uuid.uuid4())
        table.put_item(
            Item={
                'id': comparison_id,
                'Matches': matches
            }
        )

   
    # Retornar el resultado
    if matches:
        return {
            'statusCode': 200,
            'body': {
                'id': comparison_id,
                'message': 'Comparison results stored successfully!'
            }
        }
    else:
        return {
            'statusCode': 200,
            'body': {
                'message': 'No matches found!'
            }
        }