package com.example.avifeeder;

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

public class ActuadoresActivity extends AppCompatActivity implements NetworkChangeReceiver.NetworkChangeListener {

    private NetworkChangeReceiver networkReceiver;
    private Snackbar noInternetSnackbar;

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

        //Botón "volver" para regresar a Activity Main
        ImageButton btnVolver = findViewById(R.id.btnVolver);
        btnVolver.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                finish(); // Vuelve al activity anterior
            }
        });

        // Preparar Snackbar para mostrar desconexión
        noInternetSnackbar = Snackbar.make(findViewById(android.R.id.content),
                "⚠️ Sin conexión a Internet",
                Snackbar.LENGTH_INDEFINITE);
    }

    @Override
    protected void onResume() {
        super.onResume();
        // Registrar BroadcastReceiver dinámicamente
        networkReceiver = new NetworkChangeReceiver(this);
        IntentFilter filter = new IntentFilter(ConnectivityManager.CONNECTIVITY_ACTION);
        registerReceiver(networkReceiver, filter);

        // Chequeo inmediato al reanudar
        boolean connected = NetworkUtils.isInternetAvailable(this);
        onNetworkChange(connected);
    }

    @Override
    protected void onPause() {
        super.onPause();
        // Anular registro para evitar fugas
        try {
            unregisterReceiver(networkReceiver);
        } catch (IllegalArgumentException e) {
            e.printStackTrace();
        }
        networkReceiver = null;
    }

    @Override
    public void onNetworkChange(boolean isConnected) {
        // Este metodo se ejecuta en el hilo principal (onReceive del BroadcastReceiver)
        if (!isConnected) {
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