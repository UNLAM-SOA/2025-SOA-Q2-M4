package com.example.avifeeder;

import android.app.Service;
import android.content.Intent;
import android.os.IBinder;
import android.util.Log;

import androidx.annotation.Nullable;

import org.eclipse.paho.android.service.MqttAndroidClient;
import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttMessageListener;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttCallbackExtended;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

public class MqttService extends Service {
    private static final String TAG = "MqttService";
    private MqttAndroidClient mqttClient;
    private static final String BROKER_URL = "tcp://broker.hivemq.com:1883";
    private static final String CLIENT_ID = MqttClient.generateClientId();
    private static final String TOPIC = "AviFeeder/Datos";

    @Override
    public void onCreate() {
        super.onCreate();
        Log.d(TAG, "Servicio MQTT iniciado");

        mqttClient = new MqttAndroidClient(getApplicationContext(), BROKER_URL, CLIENT_ID);

        MqttConnectOptions options = new MqttConnectOptions();
        options.setAutomaticReconnect(true);
        options.setCleanSession(true);

        mqttClient.setCallback(new MqttCallbackExtended() {
            @Override
            public void connectComplete(boolean reconnect, String serverURI) {
                Log.d(TAG, "Conectado al broker MqTT...");
                subscribeToTopic();
            }

            @Override
            public void connectionLost(Throwable cause) {
                //Log.e(TAG, "Conexión MQTT perdida", cause);
                Log.e(TAG, "Conexión MQTT perdida...");
            }

            @Override
            public void messageArrived(String topic, MqttMessage message) {
                String payload = new String(message.getPayload());
                Log.d(TAG, "Mensaje recibido: " + payload);

                // Notificar a las Activities mediante broadcast
                Intent intent = new Intent("MQTT_MESSAGE");
                intent.putExtra("topic", topic);
                intent.putExtra("message", payload);
                sendBroadcast(intent);
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken token) {
                // No necesario si solo recibimos datos
            }
        });

        connect(options);
    }

    private void connect(MqttConnectOptions options) {
        try {
            mqttClient.connect(options, null, new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    Log.d(TAG, "Conectado");
                    subscribeToTopic();
                }

                @Override
                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                    Log.e(TAG, "Error al conectar", exception);
                }
            });
        } catch (MqttException e) {
            e.printStackTrace();
        }
    }

    private void subscribeToTopic() {
        try {
            mqttClient.subscribe(TOPIC, 0, new IMqttMessageListener() {
                @Override
                public void messageArrived(String topic, MqttMessage message) {
                    String payload = new String(message.getPayload());
                    Log.d(TAG, "Mensaje recibido: " + payload);

                    // 🔔 Notificar también desde aquí (seguro)
                    Intent intent = new Intent("MQTT_MESSAGE");
                    intent.putExtra("topic", topic);
                    intent.putExtra("message", payload);
                    sendBroadcast(intent);
                }
            });
        } catch (MqttException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        try {
            if (mqttClient != null && mqttClient.isConnected()) {
                mqttClient.disconnect();
            }
        } catch (MqttException e) {
            e.printStackTrace();
        }
        Log.d(TAG, "Servicio MQTT destruido");
    }

    @Nullable
    @Override
    public IBinder onBind(Intent intent) {
        return null; // Servicio no ligado (se maneja por startService)
    }
}