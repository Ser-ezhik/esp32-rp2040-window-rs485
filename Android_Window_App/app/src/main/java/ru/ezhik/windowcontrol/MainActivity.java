package ru.ezhik.windowcontrol;

import android.app.Activity;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.os.Bundle;
import android.view.Gravity;
import android.view.ViewGroup;
import android.webkit.WebChromeClient;
import android.webkit.HttpAuthHandler;
import android.webkit.WebResourceError;
import android.webkit.WebResourceRequest;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TextView;

public class MainActivity extends Activity {
    private static final String DEFAULT_HOST = "http://192.168.100.5";
    private static final String DEFAULT_USER = "admin";
    private WebView webView;
    private SharedPreferences prefs;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        prefs = getSharedPreferences("window_control", MODE_PRIVATE);
        if (prefs.getBoolean("configured", false)) {
            showWeb();
        } else {
            showSettings("ESP32 connection");
        }
    }

    private String normalizeUrl(String input) {
        String value = input == null ? "" : input.trim();
        if (value.length() == 0) value = DEFAULT_HOST;
        if (!value.startsWith("http://") && !value.startsWith("https://")) {
            value = "http://" + value;
        }
        while (value.endsWith("/")) value = value.substring(0, value.length() - 1);
        if (!value.endsWith("/mobile")) value += "/mobile";
        return value;
    }

    private void showWeb() {
        webView = new WebView(this);
        WebSettings settings = webView.getSettings();
        settings.setJavaScriptEnabled(true);
        settings.setDomStorageEnabled(true);
        settings.setDatabaseEnabled(true);
        settings.setCacheMode(WebSettings.LOAD_DEFAULT);

        webView.setWebChromeClient(new WebChromeClient());
        webView.setWebViewClient(new WebViewClient() {
            @Override
            public void onReceivedHttpAuthRequest(WebView view, HttpAuthHandler handler, String host, String realm) {
                String user = prefs.getString("user", DEFAULT_USER);
                String pass = prefs.getString("pass", "");
                if (user.length() > 0 || pass.length() > 0) {
                    handler.proceed(user, pass);
                } else {
                    handler.cancel();
                    showSettings("ESP32 login required");
                }
            }

            @Override
            public void onReceivedError(WebView view, WebResourceRequest request, WebResourceError error) {
                if (request == null || request.isForMainFrame()) {
                    showSettings("Cannot open ESP32 page");
                }
            }

            @SuppressWarnings("deprecation")
            @Override
            public void onReceivedError(WebView view, int errorCode, String description, String failingUrl) {
                showSettings("Cannot open ESP32 page");
            }
        });

        setContentView(webView);
        webView.loadUrl(prefs.getString("url", normalizeUrl(DEFAULT_HOST)));
    }

    private void showSettings(String message) {
        webView = null;

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setPadding(36, 36, 36, 36);
        root.setGravity(Gravity.CENTER_HORIZONTAL);
        root.setBackgroundColor(Color.rgb(3, 9, 24));

        TextView title = new TextView(this);
        title.setText(message);
        title.setTextColor(Color.WHITE);
        title.setTextSize(22);
        title.setGravity(Gravity.CENTER);
        root.addView(title, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));

        TextView hint = new TextView(this);
        hint.setText("Enter ESP32 address and web login.");
        hint.setTextColor(Color.rgb(170, 190, 220));
        hint.setTextSize(15);
        hint.setPadding(0, 24, 0, 10);
        root.addView(hint);

        EditText url = new EditText(this);
        url.setSingleLine(true);
        url.setTextColor(Color.WHITE);
        url.setHintTextColor(Color.rgb(130, 150, 180));
        url.setText(prefs.getString("url", normalizeUrl(DEFAULT_HOST)).replace("/mobile", ""));
        url.setHint(DEFAULT_HOST);
        root.addView(url, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));

        EditText user = new EditText(this);
        user.setSingleLine(true);
        user.setTextColor(Color.WHITE);
        user.setHintTextColor(Color.rgb(130, 150, 180));
        user.setText(prefs.getString("user", DEFAULT_USER));
        user.setHint("Login");
        root.addView(user, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));

        EditText pass = new EditText(this);
        pass.setSingleLine(true);
        pass.setTextColor(Color.WHITE);
        pass.setHintTextColor(Color.rgb(130, 150, 180));
        pass.setText(prefs.getString("pass", ""));
        pass.setHint("Password");
        root.addView(pass, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));

        Button save = new Button(this);
        save.setText("Save and open");
        root.addView(save, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));

        save.setOnClickListener(v -> {
            prefs.edit()
                    .putString("url", normalizeUrl(url.getText().toString()))
                    .putString("user", user.getText().toString().trim())
                    .putString("pass", pass.getText().toString())
                    .putBoolean("configured", true)
                    .apply();
            showWeb();
        });

        setContentView(root);
    }

    @Override
    public void onBackPressed() {
        if (webView != null && webView.canGoBack()) {
            webView.goBack();
        } else if (webView != null) {
            showSettings("ESP32 connection");
        } else {
            super.onBackPressed();
        }
    }
}
