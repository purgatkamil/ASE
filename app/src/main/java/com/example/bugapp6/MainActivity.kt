@file:Suppress("DEPRECATION")

package com.example.bugapp6

import android.animation.Animator
import android.animation.ValueAnimator
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.VibrationEffect
import android.os.Vibrator
import android.util.Log
import android.view.View
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.FrameLayout
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.ListView
import android.widget.RelativeLayout
import android.widget.ScrollView
import android.widget.TextView
import androidx.activity.result.ActivityResultLauncher
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import java.io.InputStream
import java.util.UUID
import android.view.ViewGroup
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.cardview.widget.CardView

@SuppressLint("MissingPermission")
class MainActivity : AppCompatActivity() {

    private val requestEnableBT = 1
    private var isStartButtonPressed = false
    private val bluetoothAdapter: BluetoothAdapter? = BluetoothAdapter.getDefaultAdapter()
    private var bluetoothSocket: BluetoothSocket? = null
    private lateinit var deviceListView: ListView
    private lateinit var logTextView: TextView
    private lateinit var logScrollView: ScrollView
    private lateinit var toggleLogButton: Button
    private lateinit var latestMessageTextView: TextView
    private lateinit var startButton: Button
    private lateinit var stopButton: Button
    private lateinit var timeTextView: TextView

    private val logBuilder = StringBuilder()
    private val bluetoothDeviceList: MutableList<BluetoothDevice> = mutableListOf()
    private val deviceListAdapter: ArrayAdapter<String> by lazy {
        ArrayAdapter(this, android.R.layout.simple_list_item_1, bluetoothDeviceList.map { it.name ?: "Unknown Device" })
    }
    private val requestPermissionsLauncher: ActivityResultLauncher<Array<String>> =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
            if (permissions[android.Manifest.permission.ACCESS_FINE_LOCATION] == true) {
                scanForDevices()
            }
        }

    private lateinit var discoverButton: Button
    private lateinit var pulsingIcon: ImageView

    @SuppressLint("SetTextI18n")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        deviceListView = findViewById(R.id.deviceList)
        logTextView = findViewById(R.id.logTextView)
        logScrollView = findViewById(R.id.logScrollView)
        toggleLogButton = findViewById(R.id.toggleLogButton)
        latestMessageTextView = findViewById(R.id.latestMessageTextView)

        discoverButton = findViewById(R.id.discoverButton)
        pulsingIcon = findViewById(R.id.pulsingIcon)
        timeTextView = findViewById(R.id.time)

        if (bluetoothAdapter?.isEnabled == false) {
            val enableBtIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
            startActivityForResult(enableBtIntent, requestEnableBT)
        }

        discoverButton.setOnClickListener{
            startDiscoveryAnimation()
        }

        if (ContextCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
            scanForDevices()
        } else {
            requestPermissionsLauncher.launch(arrayOf(android.Manifest.permission.ACCESS_FINE_LOCATION))
        }

        toggleLogButton.setOnClickListener {
            if (logScrollView.visibility == View.GONE) {
                expandView(logScrollView)
                //logScrollView.visibility = View.VISIBLE
                toggleLogButton.text = "Hide logs"
            } else {
                //logScrollView.visibility = View.GONE
                collapseView(logScrollView)
                toggleLogButton.text = "Show logs"
            }
        }
    }

    private fun scanForDevices() {
        bluetoothAdapter?.bondedDevices?.let { devices ->
            bluetoothDeviceList.clear()
            bluetoothDeviceList.addAll(devices)
            deviceListView.adapter = deviceListAdapter
            deviceListView.visibility = View.VISIBLE
            deviceListView.setOnItemClickListener { _, _, position, _ ->
                val device = bluetoothDeviceList[position]
                connectToBluetoothDevice(device)
            }
        }
    }

    private fun connectToBluetoothDevice(device: BluetoothDevice?) {
        if (device == null) return
        val uuid: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
        bluetoothSocket = device.createRfcommSocketToServiceRecord(uuid)
        bluetoothSocket?.apply {
            Thread {
                try {
                    connect()
                    runOnUiThread {
                        collapseDeviceList()
                        //hideDeviceListView()
                    }
                    startListeningForLogs()
                } catch (e: Exception) {
                    Log.e("Bluetooth", "Error connecting to device", e)
                }
            }.start()
        }
    }

    private fun startListeningForLogs() {
        val inputStream: InputStream? = bluetoothSocket?.inputStream
        val buffer = ByteArray(1024)
        var bytes: Int

        Thread {
            while (true) {
                try {
                    bytes = inputStream?.read(buffer) ?: 0
                    if (bytes > 0) {
                        val log = String(buffer, 0, bytes).trim()
                        Log.d("Bluetooth", "Raw received data: $log")

                        runOnUiThread{
                            processIncomingLog(log)
                            processIncomingLogTime(log)
                        }
                    }
                } catch (e: Exception) {
                    Log.e("Bluetooth", "Error reading from input stream", e)
                    break
                }
            }
        }.start()
    }

    private fun sendBluetoothMessage(byteValue: Int){
        /*try{
            bluetoothSocket?.outputStream?.write(message.toByteArray())
            Log.d("Bluetooth", "Message sent: $message")
        } catch (e: Exception){
            Log.d("Bluetooth", "Error while sending message", e)
            Toast.makeText(this, "Error while sending message to ESP32", Toast.LENGTH_SHORT).show()
        }*/

        try{
            val byteArray = byteArrayOf(byteValue.toByte())
            bluetoothSocket?.outputStream?.write(byteArray)
            Log.d("Bluetooth", "Byte sent: 0x${String.format("%02X", byteValue)}")
        } catch (e: Exception) {
            Log.d("Bluetooth", "Error while sending byte", e)
            Toast.makeText(this, "Error while sending message to ESP32", Toast.LENGTH_SHORT).show()
        }
    }

    private fun processIncomingLog(log: String) {
        val messagePattern = "(WANDER|CELEBRATE|STOP|IDLE)".toRegex()
        val matchResult = messagePattern.find(log)
        val lastMessage = matchResult?.value ?: return

        Log.d("Bluetooth", "Last message: $lastMessage")

        runOnUiThread{
            updateTextView(log)
            updateLatestMessageTextView(lastMessage)

            if(lastMessage == "CELEBRATE") {
                findViewById<TextView>(R.id.test11).visibility = View.VISIBLE
                findViewById<TextView>(R.id.time).visibility = View.VISIBLE
                
                showCelebrateDialog()
            }
        }
    }

    private fun showCelebrateDialog(){
        val dialogView = layoutInflater.inflate(R.layout.dialog_celebrate, null)

        val dialog = AlertDialog.Builder(this)
            .setView(dialogView)
            .setCancelable(true)
            .create()

        vibratePhone()

        val closeIcon = dialogView.findViewById<ImageView>(R.id.closeIcon)
        closeIcon.setOnClickListener{
            dialog.dismiss()
        }

        val closeButton = dialogView.findViewById<Button>(R.id.endMissionButton)
        closeButton.setOnClickListener{
            dialog.dismiss()
        }

        dialog.show()
    }

    private fun vibratePhone() {
        val vibrator = getSystemService(Context.VIBRATOR_SERVICE) as Vibrator

        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.O) {
            val vibrationEffect = VibrationEffect.createOneShot(500, VibrationEffect.DEFAULT_AMPLITUDE)
            vibrator.vibrate(vibrationEffect)
        } else {
            vibrator.vibrate(500)
        }
    }

    @SuppressLint("SetTextI18n")
    private fun processIncomingLogTime(log: String) {
        val messagePattern = "(<.*?>)".toRegex()
        val matchResult = messagePattern.find(log)
        val lastMessage = matchResult?.value ?: return

        Log.d("Bluetooth", "Last time message: $lastMessage")

        isStartButtonPressed = false
        startButton.isEnabled = true
        stopButton.isEnabled = false

        val messageContent = lastMessage.removePrefix("<").removeSuffix(">")
        findViewById<TextView>(R.id.test11).visibility = View.VISIBLE
        findViewById<TextView>(R.id.time).visibility = View.VISIBLE

        val timeInt = messageContent.toInt()
        val timeInSeconds = timeInt / 1000
        val timeInSecondsText = timeInSeconds.toString()

        timeTextView.text = "$timeInSecondsText s"
        //timeTextView.text = "$messageContent s"
    }

    private fun updateTextView(message: String) {
        runOnUiThread {
            logBuilder.append(message)
            logTextView.text = logBuilder.toString()
        }
    }

    private fun updateLatestMessageTextView(message: String) {
        runOnUiThread {
            latestMessageTextView.text = message
        }
    }

    private fun startDiscoveryAnimation(){
        discoverButton.visibility = View.GONE
        pulsingIcon.visibility = View.VISIBLE

        pulsingIcon.animate().rotationBy(360f * 6)
            .setDuration(3000)
            .withEndAction{
                pulsingIcon.visibility = View.GONE
                findViewById<FrameLayout>(R.id.startScreen).visibility = View.GONE
                showOriginalLayout()
            }
            .start()
    }

    @SuppressLint("CutPasteId")
    private fun showOriginalLayout(){
        findViewById<LinearLayout>(R.id.logContainer).visibility = View.VISIBLE
        findViewById<TextView>(R.id.textView7).visibility = View.VISIBLE
        findViewById<RelativeLayout>(R.id.deviceListViewContainer).visibility = View.VISIBLE
        findViewById<RelativeLayout>(R.id.deviceListViewContainer).visibility = View.VISIBLE
        findViewById<TextView>(R.id.textView7).visibility = View.VISIBLE
        findViewById<CardView>(R.id.controlPanel).visibility = View.VISIBLE
        findViewById<LinearLayout>(R.id.ll1).visibility = View.VISIBLE
        findViewById<LinearLayout>(R.id.ll2).visibility = View.VISIBLE
        findViewById<Button>(R.id.stopButton).visibility = View.VISIBLE
        findViewById<Button>(R.id.startButton).visibility = View.VISIBLE
        findViewById<TextView>(R.id.textView8).visibility = View.VISIBLE
        findViewById<ImageView>(R.id.imageView).visibility = View.VISIBLE
        findViewById<TextView>(R.id.latestMessageTextView).visibility = View.VISIBLE
        findViewById<TextView>(R.id.test11).visibility = View.GONE

        startButton = findViewById(R.id.startButton)
        stopButton = findViewById(R.id.stopButton)
        startButton.isEnabled = true
        stopButton.isEnabled = false

        startButton.setOnClickListener{
            if (!isStartButtonPressed){
                sendBluetoothMessage(0x01) //start

                isStartButtonPressed = true
                startButton.isEnabled = false

                stopButton.isEnabled = true

                timeTextView.visibility = View.GONE
                findViewById<TextView>(R.id.test11).visibility = View.GONE
            }
        }

        stopButton.setOnClickListener{
            sendBluetoothMessage(0xFF) //stop

            isStartButtonPressed = false
            startButton.isEnabled = true

            stopButton.isEnabled = false
        }
    }

    private fun collapseDeviceList(){
        val listView = findViewById<ListView>(R.id.deviceList)
        val initialHeight = listView.measuredHeight

        val animator = ValueAnimator.ofInt(initialHeight, 0)
        animator.duration = 750
        animator.addUpdateListener { valueAnimator ->
            val animatedValue = valueAnimator.animatedValue as Int
            val layoutParams = listView.layoutParams
            layoutParams.height = animatedValue
            listView.layoutParams = layoutParams
        }

        animator.addListener(object : Animator.AnimatorListener {
            override fun onAnimationStart(animation: Animator) {}
            override fun onAnimationEnd(animation: Animator) {
                listView.visibility = View.GONE
            }
            override fun onAnimationCancel(animation: Animator) {}
            override fun onAnimationRepeat(animation: Animator) {}
        })

        animator.start()

        findViewById<TextView>(R.id.textView7).visibility = View.GONE
        findViewById<LinearLayout>(R.id.logContainer).visibility = View.VISIBLE
        findViewById<Button>(R.id.toggleLogButton).visibility = View.VISIBLE
        findViewById<ScrollView>(R.id.logScrollView).visibility = View.VISIBLE
        findViewById<TextView>(R.id.logTextView).visibility = View.VISIBLE
        findViewById<TextView>(R.id.latestMessageTextView).visibility = View.VISIBLE
    }

    private fun expandView(view: View) {
        view.measure(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT)
        val targetHeight = view.measuredHeight

        view.layoutParams.height = 0
        view.visibility = View.VISIBLE

        val animator = ValueAnimator.ofInt(0, targetHeight)
        animator.addUpdateListener { valueAnimator ->
            val height = valueAnimator.animatedValue as Int
            view.layoutParams.height = height
            view.requestLayout()
        }

        animator.duration = 300
        animator.start()
    }

    private fun collapseView(view: View) {
        val initialHeight = view.measuredHeight

        val animator = ValueAnimator.ofInt(initialHeight, 0)
        animator.addUpdateListener { valueAnimator ->
            val height = valueAnimator.animatedValue as Int
            view.layoutParams.height = height
            view.requestLayout()
        }

        animator.duration = 300
        animator.addListener(object : Animator.AnimatorListener {
            override fun onAnimationStart(animation: Animator) {}
            override fun onAnimationEnd(animation: Animator) {
                view.visibility = View.GONE
            }
            override fun onAnimationCancel(animation: Animator) {}
            override fun onAnimationRepeat(animation: Animator) {}
        })

        animator.start()
    }
}
