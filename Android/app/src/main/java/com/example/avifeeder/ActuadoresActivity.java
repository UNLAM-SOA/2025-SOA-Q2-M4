package com.example.avifeeder;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.net.ConnectivityManager;
import android.net.Network;
import android.net.NetworkRequest;
import android.os.Build;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;
import android.widget.ImageButton;
import android.widget.Toast;
import android.widget.Button;

import androidx.activity.EdgeToEdge;
import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

import com.google.android.material.snackbar.Snackbar;

public class ActuadoresActivity extends AppCompatActivity implements NetworkChangeReceiver.NetworkChangeListener {

    private static final String TAG = "ActuadoresActivity";

    private NetworkChangeReceiver networkReceiver;
    private Snackbar noInternetSnackbar;
    private MqttService mqttService;
    private boolean isBound = false;
    private ConnectivityManager.NetworkCallback networkCallback; // Para Android 9+

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_actuadores);

        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main), (v, insets) -> {
            Insets systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars());
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom);
            return insets;
        });

        // Botón "volver"
        ImageButton btnVolver = findViewById(R.id.btnVolver);
        btnVolver.setOnClickListener(v -> finish());

        // Snackbar de red
        noInternetSnackbar = Snackbar.make(findViewById(android.R.id.content),
                "Sin conexión a Internet",
                Snackbar.LENGTH_INDEFINITE);

        // Vincular botones del layout con acciones MQTT
        Button btnActivarLedComida = findViewById(R.id.btnActivarLedComida);
        Button btnDesactivarLedComida = findViewById(R.id.btnDesactivarLedComida);
        Button btnActivarLedAgua = findViewById(R.id.btnActivarLedAgua);
        Button btnDesactivarLedAgua = findViewById(R.id.btnDesactivarLedAgua);

        // Asignar listeners
        btnActivarLedComida.setOnClickListener(v -> enviarComando("LED_COMIDA_ON"));
        btnDesactivarLedComida.setOnClickListener(v -> enviarComando("LED_COMIDA_OFF"));
        btnActivarLedAgua.setOnClickListener(v -> enviarComando("LED_AGUA_ON"));
        btnDesactivarLedAgua.setOnClickListener(v -> enviarComando("LED_AGUA_OFF"));
    }

    @Override
    protected void onStart() {
        super.onStart();
        // Iniciar y vincular al servicio MQTT
        Intent intent = new Intent(this, MqttService.class);
        startService(intent);
        bindService(intent, connection, Context.BIND_AUTO_CREATE);
    }

    @Override
    protected void onStop() {
        super.onStop();
        if (isBound) {
            unbindService(connection);
            isBound = false;
        }
    }

    @Override
    protected void onResume() {
        super.onResume();

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            // Modo moderno: NetworkCallback
            ConnectivityManager cm = (ConnectivityManager) getSystemService(Context.CONNECTIVITY_SERVICE);
            NetworkRequest request = new NetworkRequest.Builder().build();

            networkCallback = new ConnectivityManager.NetworkCallback() {
                @Override
                public void onAvailable(@NonNull Network network) {
                    runOnUiThread(() -> onNetworkChange(true));
                }

                @Override
                public void onLost(@NonNull Network network) {
                    runOnUiThread(() -> onNetworkChange(false));
                }
            };

            cm.registerNetworkCallback(request, networkCallback);
        } else {
            // Compatibilidad con Android < 9 (Pie)
            registerLegacyReceiver();
        }

        boolean connected = NetworkUtils.isInternetAvailable(this);
        onNetworkChange(connected);
    }

    /**
     *  Registro del BroadcastReceiver para Android < 9 (Pie)
     *  Suprime la advertencia de método deprecado.
     */
    @SuppressWarnings("deprecation")
    private void registerLegacyReceiver() {
        networkReceiver = new NetworkChangeReceiver(this);
        IntentFilter filter = new IntentFilter(ConnectivityManager.CONNECTIVITY_ACTION);
        registerReceiver(networkReceiver, filter);
    }

    @Override
    protected void onPause() {
        super.onPause();
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU && networkCallback != null) {
                ConnectivityManager cm = (ConnectivityManager) getSystemService(Context.CONNECTIVITY_SERVICE);
                cm.unregisterNetworkCallback(networkCallback);
                networkCallback = null;
            } else if (networkReceiver != null) {
                unregisterReceiver(networkReceiver);
                networkReceiver = null;
            }
        } catch (Exception e) {
            Log.e(TAG, "Error al desregistrar receptor de red", e);
        }
    }

    @Override
    public void onNetworkChange(boolean isConnected) {
        if (!isConnected) {
            if (noInternetSnackbar != null && !noInternetSnackbar.isShown()) {
                noInternetSnackbar.show();
            }
        } else if (noInternetSnackbar != null && noInternetSnackbar.isShown()) {
            noInternetSnackbar.dismiss();
        }
    }

    // Conexión al servicio MQTT
    private final ServiceConnection connection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            MqttService.LocalBinder binder = (MqttService.LocalBinder) service;
            mqttService = binder.getService();
            isBound = true;
            Log.d(TAG, "Servicio MQTT conectado");
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            isBound = false;
            Log.w(TAG, "Servicio MQTT desconectado");
        }
    };

    // Método de ejemplo para publicar comandos MQTT
    private void enviarComando(String comando) {
        if (isBound && mqttService != null) {
            mqttService.publish(comando);
            Toast.makeText(this, "Comando enviado: " + comando, Toast.LENGTH_SHORT).show();
        } else {
            Toast.makeText(this, "MQTT no conectado", Toast.LENGTH_SHORT).show();
        }
    }
}
