package com.example.avifeeder;

import android.annotation.SuppressLint;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.widget.ImageButton;
import android.widget.TextView;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;
import androidx.core.graphics.Insets;

import org.json.JSONException;
import org.json.JSONObject;

public class MainActivity extends AppCompatActivity {

    private static final String TAG = "MainActivity";
    private BroadcastReceiver mqttReceiver;
    private TextView textViewNivelAguaValor;
    private TextView textViewNivelComidaValor;
    private TextView textViewActualizacionValor;

    @SuppressLint("UnspecifiedRegisterReceiverFlag")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Referencias UI
        textViewNivelAguaValor = findViewById(R.id.textViewNivelAguaValor);
        textViewNivelComidaValor = findViewById(R.id.textViewNivelComidaValor);
        textViewActualizacionValor = findViewById(R.id.textViewActualizacionValor);

        // Botón para pasar a ActuadoresActivity
        ImageButton btnMas = findViewById(R.id.btnMas);
        btnMas.setOnClickListener(v -> {
            Intent intent = new Intent(MainActivity.this, ActuadoresActivity.class);
            startActivity(intent);
        });

        // Ajuste EdgeToEdge (opcional)
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main), (v, insets) -> {
            Insets systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars());
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom);
            return insets;
        });

        // Inicializar el BroadcastReceiver antes de iniciar el servicio
        mqttReceiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                String message = intent.getStringExtra(MqttService.MQTT_MESSAGE_KEY);
                Log.d(TAG, "Mensaje MQTT recibido. Actualizando UI");
                if (message != null) {
                    updateUIWithMqttData(message);
                }
            }
        };

        // Registrar el receiver **en onCreate** para no perder mensajes
        String mqttBroadcastAction = getString(R.string.mqtt_message_broadcast);
        IntentFilter filter = new IntentFilter(mqttBroadcastAction);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            registerReceiver(mqttReceiver, filter, Context.RECEIVER_NOT_EXPORTED);
        }
        else
        {
            registerReceiver(mqttReceiver, filter);
        }

        // Iniciar servicio MQTT
        Intent mqttServiceIntent = new Intent(this, MqttService.class);
        startService(mqttServiceIntent);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        // Desregistrar receiver
        unregisterReceiver(mqttReceiver);
    }

    private void updateUIWithMqttData(String message) {
        try {
            JSONObject json = new JSONObject(message);

            int potValue = json.optInt("PotValue", 0);
            int distance = json.optInt("Distance (cm)", 0);
            int timeLog  = json.optInt("TimeLog (s)", 0);

            // Evaluación para 'PotValue'
            String potValueStatus = (potValue >= 2048) ? "SUFICIENTE" : "INSUFICIENTE";

            // Evaluación para 'Distance'
            String distanceStatus = (distance < 20) ? "SUFICIENTE" : "INSUFICIENTE";

            // Formateo de 'TimeLog' en HH:MM:SS
            int hours = timeLog / 3600;
            int minutes = (timeLog % 3600) / 60;
            int seconds = timeLog % 60;
            @SuppressLint("DefaultLocale") String timeFormatted = String.format("%02d:%02d:%02d", hours, minutes, seconds); // Formato HH:MM:SS

            // Actualizar la UI en el hilo principal
            runOnUiThread(() -> {
                textViewNivelAguaValor.setText(potValueStatus);
                textViewNivelComidaValor.setText(distanceStatus);
                textViewActualizacionValor.setText(timeFormatted);
            });

        } catch (JSONException e) {
            Log.e(TAG, "Error parseando JSON: " + e.getMessage());
        }
    }
}
