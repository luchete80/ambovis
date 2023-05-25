package com.respiraar.ventvis.display;

import android.graphics.Color;
import android.graphics.Typeface;
import android.util.Pair;
import android.view.View;

import com.github.mikephil.charting.charts.LineChart;
import com.github.mikephil.charting.components.Legend;
import com.github.mikephil.charting.components.XAxis;
import com.github.mikephil.charting.components.YAxis;
import com.github.mikephil.charting.data.Entry;
import com.github.mikephil.charting.data.LineData;
import com.github.mikephil.charting.data.LineDataSet;
import com.github.mikephil.charting.listener.OnChartValueSelectedListener;
import com.github.mikephil.charting.utils.ColorTemplate;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;

public class RealtimeLine {

    private LineChart chart;
    private String lineTitle;
    private Date startTime;

    private float showTime;
    private float minValue;
    private float maxValue;

    private float textSize = 9;

    public RealtimeLine(View viewChart, OnChartValueSelectedListener chartListener,
                        Typeface typeFace, String title, double xWidth, double yMin, double yMax) {

        this.lineTitle = title;

        this.startTime = Calendar.getInstance().getTime();

        this.showTime = (float)xWidth;
        this.minValue = (float)yMin;
        this.maxValue = (float)yMax;

        chart = (LineChart) viewChart;
        chart.setOnChartValueSelectedListener(chartListener);

        // enable description text
        chart.getDescription().setEnabled(true);

        // enable touch gestures
        chart.setTouchEnabled(true);

        // enable scaling and dragging
        chart.setDragEnabled(true);
        chart.setScaleEnabled(true);
        chart.setDrawGridBackground(false);

        // if disabled, scaling can be done on x- and y-axis separately
        chart.setPinchZoom(true);

        // set an alternative background color
        chart.setBackgroundColor(Color.BLACK);

        // get the legend (only possible after setting data)
        Legend l = chart.getLegend();

        // modify the legend ...
        l.setForm(Legend.LegendForm.LINE);
        l.setTypeface(typeFace);
        l.setTextColor(Color.WHITE);
        l.setVerticalAlignment(Legend.LegendVerticalAlignment.TOP);
        l.setHorizontalAlignment(Legend.LegendHorizontalAlignment.RIGHT);
        l.setOrientation(Legend.LegendOrientation.HORIZONTAL);
        l.setDrawInside(true);

        XAxis xl = chart.getXAxis();
        xl.setTypeface(typeFace);
        xl.setTextColor(Color.WHITE);
        xl.setDrawGridLines(false);
        xl.setAvoidFirstLastClipping(true);
        xl.setEnabled(true);

        YAxis leftAxis = chart.getAxisLeft();
        leftAxis.setTypeface(typeFace);
        leftAxis.setTextColor(Color.WHITE);
        leftAxis.setAxisMaximum(maxValue);
        leftAxis.setAxisMinimum(minValue);
        leftAxis.setDrawGridLines(true);

        YAxis rightAxis = chart.getAxisRight();
        rightAxis.setEnabled(false);
    }

    public void addValue(ArrayList<Pair> values) {
        LineData data = chart.getData();

        Date currentTime = Calendar.getInstance().getTime();

        float xValue = (float)(((float)(currentTime.getTime() - startTime.getTime())) / 1000.0);

        if (data == null) {
            for (int i = 0; i < values.size(); i++) {
                Entry newEntry = new Entry(xValue, (float) values.get(i).second, values.get(i).first);

                ArrayList<Entry> newEntryList = new ArrayList<Entry>();
                newEntryList.add(newEntry);

                int color;

                if(i == 0) {
                    color = Color.GREEN;
                } else if ((i == 1)){
                    color = Color.BLUE;
                } else if ((i == 2)) {
                    color = Color.RED;
                } else {
                    color = Color.YELLOW;
                }

                LineDataSet newSet = createSet(newEntryList, color);
                if (data == null) {
                    data = new LineData(newSet);
                }else {
                    data.addDataSet(newSet);
                }
            }

            data.setValueTextColor(Color.WHITE);
            data.setValueTextSize(textSize);
        }else {
            for (int i = 0; i < values.size(); i++) {
                Entry newEntry = new Entry(xValue, (float) values.get(i).second, values.get(i).first);
                LineDataSet newSet = (LineDataSet) chart.getData().getDataSetByIndex(i);
                List<Entry> newEntryList = newSet.getValues();
                newEntryList.add(newEntry);
                newSet.setValues(newEntryList);
            }

        }
        chart.setData(data);
        chart.getData().notifyDataChanged();
        chart.notifyDataSetChanged();
        chart.invalidate();
        chart.setVisibleXRangeMaximum(showTime);

    }

    public void clearValues() {
        chart.clearValues();
    }

    public LineChart getChart() {
        return chart;
    }

    private LineDataSet createSet(ArrayList<Entry> entryList, int lineColor) {

        LineDataSet set = new LineDataSet(entryList,
                ((entryList.get(0).getData()!= null) ? entryList.get(0).getData().toString() : ""));
        set.setAxisDependency(YAxis.AxisDependency.LEFT);
        set.setColor(lineColor);
        set.setDrawCircles(false);
        set.setLineWidth(1f);
        set.setFillAlpha(65);
        set.setFillColor(ColorTemplate.getHoloBlue());
        set.setHighLightColor(Color.rgb(244, 117, 117));
        set.setValueTextColor(Color.WHITE);
        set.setValueTextSize(14f);
        set.setDrawValues(false);
        return set;
    }

}
