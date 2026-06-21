package ru.ezhik.windowcontrol;

import android.app.Activity;
import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.net.wifi.WifiManager;
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
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class MainActivity extends Activity {
    private static final String DEFAULT_HOST = "http://192.168.100.5";
    private static final String DEFAULT_USER = "admin";
    private static final int DISCOVERY_PORT = 4210;
    private static final String DISCOVERY_QUERY = "WINDOW_DISCOVER";
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

        Button scan = new Button(this);
        scan.setText("Scan ESP32");
        root.addView(scan, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT));

        LinearLayout results = new LinearLayout(this);
        results.setOrientation(LinearLayout.VERTICAL);
        root.addView(results, new LinearLayout.LayoutParams(
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

        scan.setOnClickListener(v -> scanEsp32(url, results));

        setContentView(root);
        scanEsp32(url, results);
    }

    private String jsonValue(String json, String key) {
        Pattern pattern = Pattern.compile("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
        Matcher matcher = pattern.matcher(json);
        return matcher.find() ? matcher.group(1) : "";
    }

    private void addResultButton(EditText url, LinearLayout results, String label, String foundUrl) {
        runOnUiThread(() -> {
            Button button = new Button(this);
            button.setText(label);
            button.setOnClickListener(v -> url.setText(foundUrl.replace("/mobile", "")));
            results.addView(button, new LinearLayout.LayoutParams(
                    ViewGroup.LayoutParams.MATCH_PARENT,
                    ViewGroup.LayoutParams.WRAP_CONTENT));
        });
    }

    private void scanEsp32(EditText url, LinearLayout results) {
        results.removeAllViews();
        TextView searching = new TextView(this);
        searching.setText("Searching...");
        searching.setTextColor(Color.rgb(170, 190, 220));
        searching.setPadding(0, 8, 0, 8);
        results.addView(searching);

        new Thread(() -> {
            WifiManager.MulticastLock lock = null;
            try {
                WifiManager wifi = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
                if (wifi != null) {
                    lock = wifi.createMulticastLock("window-discovery");
                    lock.setReferenceCounted(false);
                    lock.acquire();
                }

                DatagramSocket socket = new DatagramSocket(DISCOVERY_PORT + 1);
                socket.setBroadcast(true);
                socket.setSoTimeout(650);
                byte[] query = DISCOVERY_QUERY.getBytes("UTF-8");
                DatagramPacket packet = new DatagramPacket(query, query.length, InetAddress.getByName("255.255.255.255"), DISCOVERY_PORT);
                socket.send(packet);

                long until = System.currentTimeMillis() + 1800;
                byte[] buffer = new byte[512];
                boolean found = false;
                while (System.currentTimeMillis() < until) {
                    try {
                        DatagramPacket response = new DatagramPacket(buffer, buffer.length);
                        socket.receive(response);
                        String text = new String(response.getData(), 0, response.getLength(), "UTF-8");
                        if (!text.contains("window_esp32")) continue;
                        String foundUrl = jsonValue(text, "url");
                        if (foundUrl.length() == 0) foundUrl = "http://" + response.getAddress().getHostAddress() + "/mobile";
                        String name = jsonValue(text, "name");
                        String ip = jsonValue(text, "ip");
                        if (name.length() == 0) name = "ESP32 Windows";
                        if (ip.length() == 0) ip = response.getAddress().getHostAddress();
                        addResultButton(url, results, name + "  " + ip, foundUrl);
                        found = true;
                    } catch (Exception ignored) {
                    }
                }
                socket.close();
                boolean finalFound = found;
                runOnUiThread(() -> {
                    if (results.getChildCount() > 0) results.removeViewAt(0);
                    if (!finalFound) {
                        TextView none = new TextView(this);
                        none.setText("ESP32 not found. Enter address manually.");
                        none.setTextColor(Color.rgb(220, 170, 170));
                        results.addView(none);
                    }
                });
            } catch (Exception e) {
                runOnUiThread(() -> {
                    results.removeAllViews();
                    TextView error = new TextView(this);
                    error.setText("Search error. Enter address manually.");
                    error.setTextColor(Color.rgb(220, 170, 170));
                    results.addView(error);
                });
            } finally {
                if (lock != null && lock.isHeld()) lock.release();
            }
        }).start();
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
