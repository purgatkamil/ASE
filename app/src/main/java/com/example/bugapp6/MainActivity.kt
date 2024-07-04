package com.example.bugapp6

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Bundle
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import kotlinx.coroutines.*
import java.io.IOException
import java.util.*

class MainActivity : AppCompatActivity() {
    private val REQUEST_ENABLE_BT = 1
    private val REQUEST_LOCATION_PERMISSION = 2

    private lateinit var bluetoothAdapter: BluetoothAdapter
    private lateinit var bluetoothDevice: BluetoothDevice
    private lateinit var bluetoothSocket: BluetoothSocket
    private lateinit var connectButton: Button
    private lateinit var northButton: Button
    private lateinit var southButton: Button
    private lateinit var eastButton: Button
    private lateinit var westButton: Button
    private lateinit var sendTimeButton: Button
    private lateinit var receivedMessagesTextView: TextView
    private var outputStream: CoroutineScope? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        connectButton = findViewById(R.id.connectButton)
        northButton = findViewById(R.id.northButton)
        southButton = findViewById(R.id.southButton)
        eastButton = findViewById(R.id.eastButton)
        westButton = findViewById(R.id.westButton)
        sendTimeButton = findViewById(R.id.sendTimeButton)
        receivedMessagesTextView = findViewById(R.id.receivedMessagesTextView)

        connectButton.setOnClickListener {
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                != PackageManager.PERMISSION_GRANTED
            ) {
                ActivityCompat.requestPermissions(
                    this,
                    arrayOf(Manifest.permission.ACCESS_FINE_LOCATION),
                    REQUEST_LOCATION_PERMISSION
                )
            } else {
                connectToDevice()
            }
        }

        northButton.setOnClickListener { sendDirection("N") }
        southButton.setOnClickListener { sendDirection("S") }
        eastButton.setOnClickListener { sendDirection("E") }
        westButton.setOnClickListener { sendDirection("W") }
        sendTimeButton.setOnClickListener { sendCurrentTime() }

        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter()

        if (bluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth is not supported on this device", Toast.LENGTH_SHORT).show()
            finish()
            return
        }

        if (!bluetoothAdapter.isEnabled) {
            val enableBtIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
            startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT)
        }
    }

    private fun connectToDevice() {
        val deviceAddress = "A0:A3:B3:2D:19:26"
        bluetoothDevice = bluetoothAdapter.getRemoteDevice(deviceAddress)
        val uuid = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB") // Standard UUID for SPP

        if (ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.ACCESS_FINE_LOCATION
            ) != PackageManager.PERMISSION_GRANTED
        ) {
            return
        }

        try {
            bluetoothSocket = bluetoothDevice.createRfcommSocketToServiceRecord(uuid)
            bluetoothSocket.connect()
            Toast.makeText(this, "Connected to ESP32", Toast.LENGTH_SHORT).show()
            startReceivingMessages()
            outputStream = CoroutineScope(Dispatchers.IO)
        } catch (e: IOException) {
            e.printStackTrace()
            Toast.makeText(this, "Connection failed", Toast.LENGTH_SHORT).show()
        } catch (e: SecurityException) {
            e.printStackTrace()
            Toast.makeText(
                this,
                "Location permission required to connect to Bluetooth devices",
                Toast.LENGTH_SHORT
            ).show()
        }
    }

    private fun sendDirection(direction: String) {
        outputStream?.launch {
            try {
                val outputStream = bluetoothSocket.outputStream
                outputStream.write(direction.toByteArray())
                outputStream.flush()
            } catch (e: IOException) {
                e.printStackTrace()
                withContext(Dispatchers.Main) {
                    Toast.makeText(this@MainActivity, "Failed to send direction", Toast.LENGTH_SHORT).show()
                }
            }
        }
    }

    private fun sendCurrentTime() {
        outputStream?.launch {
            try {
                val calendar = Calendar.getInstance()
                val hour = calendar.get(Calendar.HOUR_OF_DAY)
                val minute = calendar.get(Calendar.MINUTE)
                val second = calendar.get(Calendar.SECOND)
                val timeString = String.format("%02d:%02d:%02d", hour, minute, second)

                val outputStream = bluetoothSocket.outputStream
                outputStream.write(timeString.toByteArray())
                outputStream.flush()
            } catch (e: IOException) {
                e.printStackTrace()
                withContext(Dispatchers.Main) {
                    Toast.makeText(this@MainActivity, "Failed to send current time", Toast.LENGTH_SHORT).show()
                }
            }
        }
    }

    private fun startReceivingMessages() {
        val buffer = ByteArray(1024)
        CoroutineScope(Dispatchers.IO).launch {
            try {
                val inputStream = bluetoothSocket.inputStream
                while (true) {
                    val bytes = inputStream.read(buffer)
                    val message = buffer.decodeToString(0, bytes)
                    withContext(Dispatchers.Main) {
                        receivedMessagesTextView.append("\n$message")
                    }
                }
            } catch (e: IOException) {
                e.printStackTrace()
                withContext(Dispatchers.Main) {
                    Toast.makeText(this@MainActivity, "Failed to receive message", Toast.LENGTH_SHORT).show()
                }
            }
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        when (requestCode) {
            REQUEST_LOCATION_PERMISSION -> {
                if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    connectToDevice()
                } else {
                    Toast.makeText(
                        this,
                        "Location permission required to connect to Bluetooth devices",
                        Toast.LENGTH_SHORT
                    ).show()
                }
            }
        }
    }
}
