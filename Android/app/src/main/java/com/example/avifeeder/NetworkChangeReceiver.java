package com.example.avifeeder;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.net.ConnectivityManager;

public class NetworkChangeReceiver extends BroadcastReceiver {

    public interface NetworkChangeListener {
        void onNetworkChange(boolean isConnected);
    }

    private final NetworkChangeListener listener;

    public NetworkChangeReceiver(NetworkChangeListener listener) {
        this.listener = listener;
    }

    @Override
    public void onReceive(Context context, Intent intent) {
        boolean isConnected = NetworkUtils.isInternetAvailable(context);
        if (listener != null) {
            listener.onNetworkChange(isConnected);
        }
    }
}