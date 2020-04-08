
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


public class RealtimeLineChartActivity extends ChartBase implements
        OnChartValueSelectedListener {

    private RealtimeLine chartPressure;
    private RealtimeLine chartFlow;
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

        chartPressure = new RealtimeLine(findViewById(R.id.chart1), this, tfLight, "Pressure");
        chartFlow = new RealtimeLine(findViewById(R.id.chart2), this, tfLight, "Flow");

        feedMultiple();
    }

    private void testAddEntry() {

        float valPressure = (float) (Math.random() * 40 + 30f);
        float valFlow = (float) (Math.random() * 40 + 30f);

        TextView valuePip = findViewById(R.id.valuePip);
        valuePip.setText(String.format("%.2f", valPressure));

        TextView valuePeep = findViewById(R.id.valuePeep);
        valuePeep.setText(String.format("%.2f", valPressure * 10));

        TextView valueFr = findViewById(R.id.valueFr);
        valueFr.setText(String.format("%.2f", valFlow));

        TextView valueVol = findViewById(R.id.valueVol);
        valueVol.setText(String.format("%.2f", valFlow *10));

        chartPressure.addValue(valPressure);
        chartFlow.addValue(valFlow);
    }

    @SuppressLint("DefaultLocale")
    public void addEntry(float valPressure, float valFlow) {

        TextView valuePip = findViewById(R.id.valuePip);
        valuePip.setText(String.format("%.2f", valPressure));

        TextView valuePeep = findViewById(R.id.valuePeep);
        valuePeep.setText(String.format("%.2f", valPressure * 10));

        TextView valueFr = findViewById(R.id.valueFr);
        valueFr.setText(String.format("%.2f", valFlow));

        TextView valueVol = findViewById(R.id.valueVol);
        valueVol.setText(String.format("%.2f", valFlow *10));

        chartPressure.addValue(valPressure);
        chartFlow.addValue(valFlow);
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

                    // Don't generate garbage runnables inside the loop.
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

        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case UsbService.MESSAGE_FROM_SERIAL_PORT:
                    String data = (String) msg.obj;
                    String[] dataArray = data.split("\\s");
                    //Parse data as described here:
                    // https://gitlab.com/reesistencia/reespirator-beagle-touch/-/blob/master/src/respyrator/ui.py
                    // Unused data is commented for now.

                    switch (dataArray[0]) {
                        case "CONFIG":
                            //config_pip = Float.parseFloat(dataArray[1]);
                            //config_peep = Float.parseFloat(dataArray[2];
                            //config_fr = Float.parseFloat(dataArray[3];
                            break;
                        case "DT":
                            float pres1 = (Float.parseFloat(dataArray[1])) / (float) 1000.0;
                            //pres2 = Float.parseFloat(dataArray[3]);
                            //# vol = Float.parseFloat(dataArray[3]);
                            float flow = (Float.parseFloat(dataArray[4])) / (float) 1000.0;
                            mActivity.get().addEntry(pres1, flow);
                            break;
                        case "VOL":
                            //vol = Float.parseFloat(dataArray[1];
                            break;
                        case "EOC":
                            //pip = Float.parseFloat(dataArray[1];
                            //peep = Float.parseFloat(dataArray[2];
                            //vol = Float.parseFloat(dataArray[3];
                            break;
                    }
                    break;
            }
        }
    }
}
