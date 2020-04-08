package com.respiraar.ventvis;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.os.Bundle;
import android.view.View;
import android.view.WindowManager;
import android.widget.AdapterView;
import android.widget.ListView;


import com.github.mikephil.charting.utils.Utils;
import com.respiraar.ventvis.communication.SerialActivity;
import com.respiraar.ventvis.display.ContentItem;
import com.respiraar.ventvis.display.MyAdapter;
import com.respiraar.ventvis.display.RealtimeLineChartActivity;

import java.util.ArrayList;

public class MainActivity extends AppCompatActivity implements AdapterView.OnItemClickListener {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_main);

        setTitle("VentVis App");

        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_SENSOR);

        // initialize the utilities
        Utils.init(this);

        ArrayList<ContentItem> objects = new ArrayList<>();

        objects.add(0, new ContentItem("Serial interface", "Direct text serial interface."));
        objects.add(1, new ContentItem("Plot pressure-flow", "Pressure and flow values over time."));

        MyAdapter adapter = new MyAdapter(this, objects);

        ListView lv = findViewById(R.id.listView1);
        lv.setAdapter(adapter);

        lv.setOnItemClickListener(this);
    }

    @Override
    public void onItemClick(AdapterView<?> av, View v, int pos, long arg3) {

        Intent i = null;

        switch (pos) {
            case 0:
                i = new Intent(this, SerialActivity.class);
                break;
            case 1:
                i = new Intent(this, RealtimeLineChartActivity.class);
                break;
        }
        if (i != null) startActivity(i);
    }

}
