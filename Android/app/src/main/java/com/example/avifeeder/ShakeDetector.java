package com.example.avifeeder;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;

public class ShakeDetector implements SensorEventListener {

    private static final String TAG = "ShakeDetector";
    private static final float SHAKE_THRESHOLD_GRAVITY = 2.0F; // fuerza mínima
    private static final int SHAKE_MIN_DURATION_MS = 1000; // duración mínima de la sacudida
    private static final int SHAKE_SLOP_TIME_MS = 500; // tiempo mínimo entre eventos
    private static final int SHAKE_TOLERANCE_MS = 150; // tolerancia a pausas breves
    private long shakeStartTime = 0;
    private long lastShakeTimestamp = 0;
    private long lastAboveThresholdTime = 0;
    private boolean shaking = false;
    private final OnShakeListener listener;

    public interface OnShakeListener {
        void onShake();
    }

    // Constructor
    public ShakeDetector(OnShakeListener listener) {
        this.listener = listener;
    }

    // Si detecta cambios en el sensor, evalua para detectar Shake
    @Override
    public void onSensorChanged(SensorEvent event) {
        float x = event.values[0];
        float y = event.values[1];
        float z = event.values[2];

        float gX = x / SensorManager.GRAVITY_EARTH;
        float gY = y / SensorManager.GRAVITY_EARTH;
        float gZ = z / SensorManager.GRAVITY_EARTH;

        float gForce = (float) Math.sqrt(gX * gX + gY * gY + gZ * gZ);
        long now = System.currentTimeMillis();

        // Si detecta fuerza G, empieza a evaluar
        if (gForce > SHAKE_THRESHOLD_GRAVITY) {
            if (!shaking) {
                shaking = true; // Setea estado de shake
                shakeStartTime = now;
                Log.d(TAG, "Sacudida iniciada");
            }
            lastAboveThresholdTime = now;

            // Si continua el shake luego del tiempo definido, lanza el evento
            if (shaking && (now - shakeStartTime >= SHAKE_MIN_DURATION_MS)) {
                if (lastShakeTimestamp + SHAKE_SLOP_TIME_MS < now) {
                    lastShakeTimestamp = now;
                    Log.d(TAG, "¡Sacudida detectada!");
                    if (listener != null) listener.onShake();
                }
                shaking = false; // Reiniciar estado para detectar nuevos shake
            }
        } else {
            // Si no continua el shake, se corta la evaluacion (evitar falsos eventos de shake)
            if (shaking && (now - lastAboveThresholdTime > SHAKE_TOLERANCE_MS)) {
                shaking = false; // Reiniciar estado para detectar nuevos shake
                Log.d(TAG, "Sacudida interrumpida antes de cumplir la duración mínima");
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // No usado, pero necesario por interfaz SensorEventListener
    }
}
