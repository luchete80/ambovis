package com.respiraar.ventvis.communication;

import android.content.pm.ActivityInfo;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import androidx.appcompat.app.AppCompatActivity;

import com.respiraar.ventvis.R;

import java.lang.ref.WeakReference;

public class SerialActivity extends AppCompatActivity {

    private TextView display;
    private EditText editText;
    private SerialDevice serialDevice;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_serial);

        setTitle("Serial interface");

        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_SENSOR);

        serialDevice = new SerialDevice(new MyHandler(this), this);

        display = findViewById(R.id.textView1);
        editText = findViewById(R.id.editText1);
        Button sendButton = findViewById(R.id.buttonSend);
        sendButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (!editText.getText().toString().equals("")) {
                    String data = editText.getText().toString();
                    if (serialDevice.connected()) {
                        display.append(data);
                        serialDevice.sendData(data);
                    }
                }
            }
        });
    }

    @Override
    public void onResume() {
        super.onResume();
        serialDevice.onResume();
    }

    @Override
    public void onPause() {
        super.onPause();
        serialDevice.onPause();
    }

    /*
     * This handler will be passed to UsbService. Data received from serial port is displayed through this handler
     */
    private static class MyHandler extends Handler {
        private final WeakReference<SerialActivity> mActivity;

        public MyHandler(SerialActivity activity) {
            mActivity = new WeakReference<>(activity);
        }

        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case UsbService.MESSAGE_FROM_SERIAL_PORT:
                    String data = (String) msg.obj;
                    mActivity.get().display.append(data);
                    break;
            }
        }
    }
}
