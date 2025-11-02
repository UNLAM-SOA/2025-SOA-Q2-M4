package com.example.avifeeder;

import android.app.Service;
import android.content.Intent;
import android.os.Binder;
import android.os.Handler;
import android.os.IBinder;
import android.util.Log;

import androidx.annotation.Nullable;

import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

public class MqttService extends Service {

    private static final String TAG = "MqttService";
    public static final String MQTT_MESSAGE_BROADCAST = "com.example.avifeeder.MQTT_MESSAGE";
    public static final String MQTT_MESSAGE_KEY = "message";
    private static final String BROKER_URL = "tcp://broker.hivemq.com:1883";
    private static final String TOPIC = "AviFeeder/Datos";
    private MqttClient mqttClient;
    private final Handler handler = new Handler();
    private boolean isReconnecting = false;
    private static final int RECONNECT_DELAY_MS = 5000; // 5 segundos
    private final IBinder binder = new LocalBinder();

    @Override
    public void onCreate() {
        super.onCreate();
        connectToBroker();
    }

    private void connectToBroker() {
        try {
            String clientId = MqttClient.generateClientId();
            mqttClient = new MqttClient(BROKER_URL, clientId, null);

            mqttClient.setCallback(new MqttCallback() {
                @Override
                public void connectionLost(Throwable cause) {
                    Log.w(TAG, "Conexión MQTT perdida", cause);
                    scheduleReconnect();
                }

                @Override
                public void messageArrived(String topic, MqttMessage message) {
                    String msg = message.toString();
                    Log.d(TAG, "Mensaje recibido: " + msg);

                    // Enviar broadcast a MainActivity
                    try {
                        Intent intent = new Intent(MQTT_MESSAGE_BROADCAST);
                        intent.putExtra(MQTT_MESSAGE_KEY, msg);
                        intent.setPackage(getPackageName()); // Garantiza que solo la app reciba
                        sendBroadcast(intent);
                    } catch (Exception e) {
                        Log.e(TAG, "Error enviando broadcast MQTT", e);
                    }
                }

                @Override
                public void deliveryComplete(IMqttDeliveryToken token) {
                    Log.d(TAG, "Mensaje MQTT entregado");
                }
            });

            MqttConnectOptions options = new MqttConnectOptions();
            options.setCleanSession(true);

            mqttClient.connect(options);
            mqttClient.subscribe(TOPIC, 0);

            Log.i(TAG, "Conectado y suscrito al tópico: " + TOPIC);
            isReconnecting = false;

        } catch (MqttException e) {
            Log.e(TAG, "Error conectando al broker MQTT", e);
            scheduleReconnect();
        } catch (Exception e) {
            Log.e(TAG, "Error inesperado en MQTT", e);
            scheduleReconnect();
        }
    }

    private void scheduleReconnect() {
        if (isReconnecting) return; // Evitar múltiples reconexiones simultáneas
        isReconnecting = true;

        Log.w(TAG, "Intentando reconectar en " + RECONNECT_DELAY_MS / 1000 + " segundos...");

        handler.postDelayed(() -> {
            Log.i(TAG, "Reintentando conexión MQTT...");
            connectToBroker();
        }, RECONNECT_DELAY_MS);
    }

    /**
     * Publica un mensaje en el tópico configurado
     */
    public void publish(String message) {
        try {
            if (mqttClient != null && mqttClient.isConnected()) {
                MqttMessage mqttMessage = new MqttMessage(message.getBytes());
                mqttClient.publish(TOPIC, mqttMessage);
                Log.d(TAG, "Mensaje publicado: " + message);
            } else {
                Log.w(TAG, "No se puede publicar, MQTT no conectado");
            }
        } catch (MqttException e) {
            Log.e(TAG, "Error publicando mensaje MQTT", e);
        } catch (Exception e) {
            Log.e(TAG, "Error inesperado publicando mensaje MQTT", e);
        }
    }

    @Override
    public void onDestroy() {
        handler.removeCallbacksAndMessages(null);
        try {
            if (mqttClient != null && mqttClient.isConnected()) {
                mqttClient.disconnect();
                Log.d(TAG, "Desconectado del broker MQTT");
            }
        } catch (MqttException e) {
            Log.e(TAG, "Error al desconectar MQTT", e);
        } catch (Exception e) {
            Log.e(TAG, "Error inesperado al desconectar MQTT", e);
        }
        super.onDestroy();
    }

    public class LocalBinder extends Binder {
        public MqttService getService() {
            return MqttService.this;
        }
    }

    @Nullable
    @Override
    public IBinder onBind(Intent intent) {
        return binder;
    }
}
