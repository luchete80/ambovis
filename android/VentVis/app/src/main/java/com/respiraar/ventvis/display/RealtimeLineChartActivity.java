
package com.respiraar.ventvis.display;

import android.Manifest;
import android.annotation.SuppressLint;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.util.Pair;
import android.view.Menu;
import android.view.MenuItem;
import android.view.WindowManager;
import android.widget.TextView;
import android.widget.Toast;

import androidx.core.content.ContextCompat;


import com.github.mikephil.charting.data.Entry;
import com.github.mikephil.charting.highlight.Highlight;
import com.github.mikephil.charting.listener.OnChartValueSelectedListener;
import com.google.android.gms.oss.licenses.OssLicensesMenuActivity;
import com.respiraar.ventvis.R;
import com.respiraar.ventvis.communication.SerialDevice;
import com.respiraar.ventvis.communication.UsbService;

import java.lang.ref.WeakReference;
import java.util.ArrayList;


public class RealtimeLineChartActivity extends ChartBase implements
        OnChartValueSelectedListener {

    private RealtimeLine chartPressure;
    private RealtimeLine chartFlow;
    private RealtimeLine chartVol;
    private SerialDevice serialDevice;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_realtime_linechart);

        setTitle("VentVis");

        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_SENSOR);

        serialDevice = new SerialDevice(new MyHandler(this), this);

        chartPressure = new RealtimeLine(findViewById(R.id.chart1), this, tfLight,
                "Pressure", 60.0, 0.0, 40.0);
        chartFlow = new RealtimeLine(findViewById(R.id.chart2),this, tfLight,
                "Flow", 60.0, 0.0, 1000.0);
        chartVol = new RealtimeLine(findViewById(R.id.chart3),this, tfLight,
                "Vol", 60.0, 0.0, 700.0);

        feedMultiple();
    }

    private void testAddEntry() {

        ArrayList<Pair> pressures = new ArrayList<>();
        ArrayList<Pair> fluxes = new ArrayList<>();
        ArrayList<Pair> vols = new ArrayList<>();

        Pair<String, Float> p_bmp = Pair.create("BMP280",
                (float) (Math.random() * 40 + 30f));
        pressures.add(p_bmp);
        Pair<String, Float> p_honey = Pair.create("HONEYWELL (A1)",
                (float) (Math.random() * 40 + 30f));
        pressures.add(p_honey);
        Pair<String, Float> p_dpt = Pair.create("A0",
                (float) (Math.random() * 40 + 30f));
        pressures.add(p_dpt);
        Pair<String, Float> flux = Pair.create("Flux [ml/s]",
                (float) (Math.random() * 100 + 30f));
        fluxes.add(flux);
        Pair<String, Float> vol = Pair.create("Volume [ml]",
                (float) (Math.random() * 700 + 30f));
        vols.add(vol);

        addEntry(pressures, fluxes, vols);
    }

    @SuppressLint("DefaultLocale")
    public void addEntry(ArrayList<Pair> pressures, ArrayList<Pair> flux, ArrayList<Pair> vol) {

        TextView valuePip = findViewById(R.id.valuePip);
        valuePip.setText(String.format("%.2f", 0.0));

        TextView valuePeep = findViewById(R.id.valuePeep);
        valuePeep.setText(String.format("%.2f", 0.0));

        TextView valueFr = findViewById(R.id.valueFr);
        valueFr.setText(String.format("%.2f", (float)flux.get(0).second));

        TextView valueVol = findViewById(R.id.valueVol);
        valueVol.setText(String.format("%.2f", (float)vol.get(0).second));

        chartPressure.addValue(pressures);
        chartFlow.addValue(flux);
        chartVol.addValue(vol);
    }

    private Thread thread;

    private void feedMultiple() {

        if (thread != null)
            thread.interrupt();

        final Runnable runnable = new Runnable() {
            @Override
            public void run() {
            }
        };

        thread = new Thread(new Runnable() {

            @Override
            public void run() {
                while (!Thread.currentThread().isInterrupted()) {
                    runOnUiThread(runnable);

                    try {
                        Thread.sleep(25);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        });

        thread.start();
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.realtime, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {

        switch (item.getItemId()) {
            case R.id.viewGithub: {
                Intent i = new Intent(Intent.ACTION_VIEW);
                i.setData(Uri.parse("https://github.com/PhilJay/MPAndroidChart/blob/master/MPChartExample/src/com/xxmassdeveloper/mpchartexample/RealtimeLineChartActivity.java"));
                startActivity(i);
                break;
            }
            case R.id.actionAdd: {
                testAddEntry();
                break;
            }
            case R.id.actionClear: {
                chartPressure.clearValues();
                chartFlow.clearValues();
                Toast.makeText(this, "Chart cleared!", Toast.LENGTH_SHORT).show();
                break;
            }
            case R.id.actionSave: {
                if (ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE) == PackageManager.PERMISSION_GRANTED) {
                    saveToGallery();
                } else {
                    requestStoragePermission(chartFlow.getChart());
                }
                break;
            }
            case R.id.actionLicenses: {
                startActivity(new Intent(this, OssLicensesMenuActivity.class));
                break;
            }
        }
        return true;
    }

    @Override
    public void onValueSelected(Entry e, Highlight h) {
        Log.i("Entry selected", e.toString());
    }

    @Override
    public void onNothingSelected() {
        Log.i("Nothing selected", "Nothing selected.");
    }

    @Override
    protected void saveToGallery() {
        saveToGallery(chartPressure.getChart(), "RealtimeLineChartActivity");
        saveToGallery(chartFlow.getChart(), "RealtimeLineChartActivity");
        saveToGallery(chartVol.getChart(), "RealtimeLineChartActivity");
    }

    @Override
    protected void onPause() {
        super.onPause();

        serialDevice.onPause();
        if (thread != null) {
            thread.interrupt();
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        serialDevice.onResume();
    }

    private static class MyHandler extends Handler {
        private final WeakReference<RealtimeLineChartActivity> mActivity;

        public MyHandler(RealtimeLineChartActivity activity) {
            mActivity = new WeakReference<>(activity);
        }

        public float GetFloat(String s) {
            // regular expression for an integer number
            try {
                return Float.parseFloat(s);
            }
            catch (NumberFormatException ex) {
                return -40000; // Use a number outside scale
            }
        }

        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case UsbService.MESSAGE_FROM_SERIAL_PORT:
                    String data = (String) msg.obj;
                    String[] dataArray = data.split("\\s");

                    ArrayList<Pair> pressures = new ArrayList<>();
                    ArrayList<Pair> fluxes = new ArrayList<>();
                    ArrayList<Pair> vols = new ArrayList<>();

                    Pair<String, Float> p_bmp = Pair.create("BMP280",
                            GetFloat(dataArray[0]));
                    pressures.add(p_bmp);
                    Pair<String, Float> p_honey = Pair.create("HONEYWELL (A1)",
                            GetFloat(dataArray[1]));
                    pressures.add(p_honey);
                    Pair<String, Float> p_dpt = Pair.create("A0",
                            GetFloat(dataArray[2]));
                    pressures.add(p_dpt);
                    Pair<String, Float> flux = Pair.create("Flux [ml/s]",
                            GetFloat(dataArray[3]));
                    fluxes.add(flux);
                    Pair<String, Float> vol = Pair.create("Volume [ml]",
                            GetFloat(dataArray[4]));
                    vols.add(vol);

                    mActivity.get().addEntry(pressures, fluxes, vols);
            }
        }
    }
}
