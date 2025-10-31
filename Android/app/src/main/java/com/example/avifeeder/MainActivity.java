package com.example.avifeeder;

import android.content.Intent;
import android.content.IntentFilter;
import android.net.ConnectivityManager;
import android.os.Bundle;
import android.view.View;
import android.widget.ImageButton;

import androidx.activity.EdgeToEdge;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

import com.google.android.material.snackbar.Snackbar;

public class MainActivity extends AppCompatActivity implements NetworkChangeReceiver.NetworkChangeListener {

    private NetworkChangeReceiver networkReceiver;
    private Snackbar noInternetSnackbar;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_main);

        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main), (v, insets) -> {
            Insets systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars());
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom);
            return insets;
        });

        //Botón "más" para pasar a ActuadoresActivity
        ImageButton btnMas = findViewById(R.id.btnMas);
        btnMas.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(MainActivity.this, ActuadoresActivity.class);
                startActivity(intent);
            }
        });

        // Preparamos Snackbar (no se muestra hasta que haya desconexión)
        noInternetSnackbar = Snackbar.make(findViewById(android.R.id.content),
                "⚠️ Sin conexión a Internet",
                Snackbar.LENGTH_INDEFINITE);
    }

    @Override
    protected void onResume() {
        super.onResume();
        // Registrar el BroadcastReceiver dinámicamente
        networkReceiver = new NetworkChangeReceiver(this);
        IntentFilter filter = new IntentFilter(ConnectivityManager.CONNECTIVITY_ACTION);
        registerReceiver(networkReceiver, filter);

        // Chequeo inmediato al reanudar (opcional: para mostrar banner si ya está desconectado)
        boolean connected = NetworkUtils.isInternetAvailable(this);
        onNetworkChange(connected);

        // Mqtt
    }

    @Override
    protected void onPause() {
        super.onPause();
        // Anular registro para evitar fugas
        try {
            unregisterReceiver(networkReceiver);
        } catch (IllegalArgumentException e) {
            // por si no está registrado
            e.printStackTrace();
        }
        networkReceiver = null;
    }

    @Override
    public void onNetworkChange(boolean isConnected) {
        // Este metodo se ejecuta en el hilo principal (onReceive del BroadcastReceiver)
        if (!isConnected) {
            // Mostrar snackbar indefinido hasta que vuelva la conexión
            if (noInternetSnackbar != null && !noInternetSnackbar.isShown()) {
                noInternetSnackbar.show();
            }
        } else {
            // Si había un snackbar de sin internet, lo ocultamos
            if (noInternetSnackbar != null && noInternetSnackbar.isShown()) {
                noInternetSnackbar.dismiss();
            }
            // Mensaje corto indicando reconexión (opcional)
            //Snackbar.make(findViewById(android.R.id.content),
            //        "✅ Conectado a Internet",
            //        Snackbar.LENGTH_SHORT).show();
        }
    }
}