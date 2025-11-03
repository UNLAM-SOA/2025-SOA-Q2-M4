package com.example.avifeeder;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;

public class ShakeDetector implements SensorEventListener {

    public interface OnShakeListener {
        void onShake();
    }

    private static final String TAG = "ShakeDetector";

    // Configuración
    private static final float SHAKE_THRESHOLD_GRAVITY = 1.2F; // fuerza mínima
    private static final int SHAKE_MIN_DURATION_MS = 250;      // duración mínima de la sacudida
    private static final int SHAKE_SLOP_TIME_MS = 500;         // tiempo mínimo entre eventos
    private static final int SHAKE_TOLERANCE_MS = 150;         // tolerancia a pausas breves
    private long shakeStartTime = 0;
    private long lastShakeTimestamp = 0;
    private long lastAboveThresholdTime = 0;
    private boolean shaking = false;

    private final OnShakeListener listener;

    public ShakeDetector(OnShakeListener listener) {
        this.listener = listener;
    }

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

        if (gForce > SHAKE_THRESHOLD_GRAVITY) {
            if (!shaking) {
                shaking = true;
                shakeStartTime = now;
                Log.d(TAG, "Sacudida iniciada");
            }
            lastAboveThresholdTime = now;

            if (shaking && (now - shakeStartTime >= SHAKE_MIN_DURATION_MS)) {
                if (lastShakeTimestamp + SHAKE_SLOP_TIME_MS < now) {
                    lastShakeTimestamp = now;
                    Log.d(TAG, "¡Sacudida detectada!");
                    if (listener != null) listener.onShake();
                }
                shaking = false; // reiniciar estado para detectar nuevas sacudidas
            }
        } else {
            // Permitir pequeñas pausas
            if (shaking && (now - lastAboveThresholdTime > SHAKE_TOLERANCE_MS)) {
                shaking = false;
                Log.d(TAG, "Sacudida interrumpida antes de cumplir la duración mínima");
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // No usado
    }
}
