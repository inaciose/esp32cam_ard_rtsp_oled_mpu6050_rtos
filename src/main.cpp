#include "OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

#include "SimStreamer.h"
#include "OV2640Streamer.h"
#include "CRtspSession.h"

// ################################################################
// ###########     ATENTION - ATENTION     ########################
// ###########     ATENTION - ATENTION     ########################
// ###########     ATENTION - ATENTION     ########################
// ################################################################

// if give an error about BUFFER_LENGTH on compilation
// on library file MPU6050.cpp, insert the following line
// #define BUFFER_LENGTH 32
// at the top or just before the following line (near 2751)
// int8_t MPU6050::GetCurrentFIFOPacket(uint8_t *data, uint8_t length) { // overflow proof

// ################################################################
// ###########     ATENTION - ATENTION     ########################
// ###########     ATENTION - ATENTION     ########################
// ###########     ATENTION - ATENTION     ########################
// ################################################################

// mpu6050
#define M_PI 3.14159265358979323846
#define __PGMSPACE_H_ 1 // stop compile errors of redefined typedefs and defines with ESP32-Arduino

// ==================================================

// for led blink
//#define LED_BUILTIN 33
#ifdef LED_BUILTIN
bool ledon = false;
#endif

#define ENABLE_OLED //if want use oled ,turn on thi macro
// #define SOFTAP_MODE // If you want to run our own softap turn this on
#define ENABLE_WEBSERVER
#define ENABLE_RTSPSERVER

#ifdef ENABLE_OLED

#include <Wire.h>
// The pins for I2C are defined by the Wire-library, or in this case,
// in the following lines

// -----------------I2C-----------------
// -- NEED TO DEFINE PINS IN ESP32CAM --
// ----------- 3 NEW LINES -------------
#define I2C_SDA 14 // SDA Connected to GPIO 14
#define I2C_SCL 15 // SCL Connected to GPIO 15

// mpu6050
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

static int i2cCore = 1;

#define DMP_INTERRUPT_PIN GPIO_NUM_13

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

volatile int mpuDataCounter = 0;
int mpuDataCounterPrev = 0;

// oled
#include "SSD1306.h"
#define OLED_ADDRESS 0x3c
#define I2C_SDA 14
#define I2C_SCL 15
SSD1306Wire display(OLED_ADDRESS, I2C_SDA, I2C_SCL, GEOMETRY_128_64);
bool hasDisplay; // we probe for the device at runtime
#endif

OV2640 cam;

#ifdef ENABLE_WEBSERVER
WebServer server(80);
#endif

#ifdef ENABLE_RTSPSERVER
WiFiServer rtspServer(8554);
#endif

#ifdef SOFTAP_MODE
IPAddress apIP = IPAddress(192, 168, 1, 1);
#else
#include "wifikeys.h"
#endif

// mpu6050
static void IRAM_ATTR dmpDataReady(void * arg) {
    mpuInterrupt = true;
}


#ifdef ENABLE_WEBSERVER
void handle_jpg_stream(void)
{
    WiFiClient client = server.client();
    String response = "HTTP/1.1 200 OK\r\n";
    response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    server.sendContent(response);

    while (1)
    {
        cam.run();
        if (!client.connected())
            break;
        response = "--frame\r\n";
        response += "Content-Type: image/jpeg\r\n\r\n";
        server.sendContent(response);

        client.write((char *)cam.getfb(), cam.getSize());
        server.sendContent("\r\n");
        if (!client.connected())
            break;
    }
}

void handle_jpg(void)
{
    WiFiClient client = server.client();

    cam.run();
    if (!client.connected())
    {
        return;
    }
    String response = "HTTP/1.1 200 OK\r\n";
    response += "Content-disposition: inline; filename=capture.jpg\r\n";
    response += "Content-type: image/jpeg\r\n\r\n";
    server.sendContent(response);
    client.write((char *)cam.getfb(), cam.getSize());
}

void handleNotFound()
{
    String message = "Server is running!\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    server.send(200, "text/plain", message);
}
#endif

void lcdMessage(String msg)
{
  #ifdef ENABLE_OLED
    if(hasDisplay) {
        display.clear();
        display.drawString(128 / 2, 32 / 2, msg);
        display.display();
    }
  #endif
}


CStreamer *streamer;
CRtspSession *session;
WiFiClient client; // FIXME, support multiple clients

void videoTask( void * pvParameters )
{
    //cam.init(esp32cam_config);
    cam.init(esp32cam_aithinker_config);

    printf("init cam done");

#ifdef ENABLE_WEBSERVER
    server.on("/", HTTP_GET, handle_jpg_stream);
    server.on("/jpg", HTTP_GET, handle_jpg);
    server.onNotFound(handleNotFound);
    server.begin();

    printf("init web server done");

#endif

#ifdef ENABLE_RTSPSERVER
    rtspServer.begin();

    printf("init rtp server done");
#endif

    while(true)
    {
    #ifdef ENABLE_WEBSERVER
        server.handleClient();
    #endif

    #ifdef ENABLE_RTSPSERVER
        uint32_t msecPerFrame = 100;
        static uint32_t lastimage = millis();

        // If we have an active client connection, just service that until gone
        // (FIXME - support multiple simultaneous clients)
        if(session) {
            session->handleRequests(0); // we don't use a timeout here,
            // instead we send only if we have new enough frames

            uint32_t now = millis();
            if(now > lastimage + msecPerFrame || now < lastimage) { // handle clock rollover
                session->broadcastCurrentFrame(now);
                lastimage = now;

                // check if we are overrunning our max frame rate
                now = millis();
                if(now > lastimage + msecPerFrame)
                    printf("warning exceeding max frame rate of %d ms\n", now - lastimage);
            }

            if(session->m_stopped) {
                delete session;
                delete streamer;
                session = NULL;
                streamer = NULL;
            }
        }
        else {
            client = rtspServer.accept();

            if(client) {
                //streamer = new SimStreamer(&client, true);             // our streamer for UDP/TCP based RTP transport
                streamer = new OV2640Streamer(&client, cam);             // our streamer for UDP/TCP based RTP transport

                session = new CRtspSession(&client, streamer); // our threads RTSP session and state
            }
        }
    #endif
    }
}



void sensorTask( void * pvParameters ) {

  // ================== SETUP ==================

  delay(100);

  String taskMessage = "sensorTask running on core ";
  taskMessage = taskMessage + xPortGetCoreID();

  Serial.println(taskMessage);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  //Wire.begin(14, 15, 4000000);

  delay(1000);

  // initialize device
  Serial.println(F("First MPU6050 initialization ..."));
  mpu.initialize();

  delay(100);

  mpu.reset(); //help startup reliably - doesn't always work though.
  // maybe can also reset i2c on esp32?
  Serial.println(F("MPU6050 reset..."));

  delay(100);

  mpu.resetI2CMaster();
  Serial.println(F("MPU6050 resetI2CMaster..."));

  delay(100);

  // initialize device
  Serial.println(F("Final MPU6050 initialization..."));
  mpu.initialize();
  pinMode(DMP_INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  //    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //    while (Serial.available() && Serial.read()); // empty buffer
  //    while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for someone else's test chip
  //  mpu.setZAccelOffset(0); // 1688 factory default for someone else's test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable interrupt detection
    /*
    Serial.print(F("Enabling interrupt detection (mcu external interrupt "));
    Serial.print(digitalPinToInterrupt(DMP_INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(DMP_INTERRUPT_PIN), dmpDataReady, RISING);
    */

    esp_err_t err;

    err = gpio_isr_handler_add(DMP_INTERRUPT_PIN, &dmpDataReady, (void *) 13);
    if (err != ESP_OK) {
        Serial.printf("handler add failed with error 0x%x \r\n", err);
    }

    err = gpio_set_intr_type(DMP_INTERRUPT_PIN, GPIO_INTR_POSEDGE);
    if (err != ESP_OK) {
        Serial.printf("set intr type failed with error 0x%x \r\n", err);
    }

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  delay(100);

  // ================== LOOP ==================

  while (true) {
    //if (connected) {

      // if programming failed, don't try to do anything
      if (!dmpReady) {
        Serial.println("dmpNotReady");
        delay(1);
      }

      // wait for MPU interrupt or extra packet(s) available
      while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        //
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
      }

      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();

      // get current FIFO count
      fifoCount = mpu.getFIFOCount();

      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        //OUTPUT_READABLE_YAWPITCHROLL
        //mpu.dmpGetQuaternion(&q, fifoBuffer);
        //mpu.dmpGetGravity(&gravity, &q);
        //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // full -180 to 180 deg Pitch, but other od behavior
        // mpu.dmpGetYawPitchRollBeng27(ypr, &q, &gravity);
        
        // for different board orientation. But Yaw doesn't work properly with this.
        // mpu.dmpGetYawPitchRollVertical(ypr, &q, &gravity);

        mpuDataCounter++;

      }
  } // end of loop
} // end sensorTask



void setup()
{
    
    // INIT SERIAL (debug purposes)
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }
    Serial.println(F("Serial connected"));

    //Wire.begin(I2C_SDA, I2C_SCL, 4000000);

  #ifdef ENABLE_OLED
    
    hasDisplay = display.init();
    if(hasDisplay) {
        display.flipScreenVertically();
        display.setFont(ArialMT_Plain_16);
        display.setTextAlignment(TEXT_ALIGN_CENTER);
    }
    
  #endif
    
#ifdef LED_BUILTIN
    // led blink
    pinMode(LED_BUILTIN, OUTPUT);
#endif

    lcdMessage("booting");

/*
    //cam.init(esp32cam_config);
    cam.init(esp32cam_aithinker_config);
*/
    IPAddress ip;

#ifdef SOFTAP_MODE
    const char *hostname = "devcam";
    // WiFi.hostname(hostname); // FIXME - find out why undefined
    lcdMessage("starting softAP");
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    bool result = WiFi.softAP(hostname, "12345678", 1, 0);
    if (!result)
    {
        Serial.println("AP Config failed.");
        return;
    }
    else
    {
        Serial.println("AP Config Success.");
        Serial.print("AP MAC: ");
        Serial.println(WiFi.softAPmacAddress());

        ip = WiFi.softAPIP();
    }
#else
    lcdMessage(String("join ") + ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(F("."));
    }
    ip = WiFi.localIP();
    Serial.println(F("WiFi connected"));
    Serial.println("");
    Serial.println(ip);
#endif

    lcdMessage(ip.toString());

/*
#ifdef ENABLE_WEBSERVER
    server.on("/", HTTP_GET, handle_jpg_stream);
    server.on("/jpg", HTTP_GET, handle_jpg);
    server.onNotFound(handleNotFound);
    server.begin();
#endif

#ifdef ENABLE_RTSPSERVER
    rtspServer.begin();
#endif
*/

    // video
    delay(100);

    Serial.print("Creating video task on core ");
    Serial.println(0);
    xTaskCreatePinnedToCore(
        videoTask,   /* Function to implement the task */
        "videoTask", /* Name of the task */
        10000,      /* Stack size in words */
        NULL,       /* Task input parameter */
        20,          /* Priority of the task */
        NULL,       /* Task handle. */
        0);  /* Core where the task should run */

    Serial.println("video task created.");

    // mpu6050
    delay(100);

    Serial.print("Creating i2c task on core ");
    Serial.println(i2cCore);
    xTaskCreatePinnedToCore(
        sensorTask,   /* Function to implement the task */
        "coreTask", /* Name of the task */
        10000,      /* Stack size in words */
        NULL,       /* Task input parameter */
        20,          /* Priority of the task */
        NULL,       /* Task handle. */
        i2cCore);  /* Core where the task should run */

    Serial.println("i2c task created.");

}

/*
CStreamer *streamer;
CRtspSession *session;
WiFiClient client; // FIXME, support multiple clients
*/

void loop()
{
    /*
#ifdef ENABLE_WEBSERVER
    server.handleClient();
#endif
    */

#ifdef LED_BUILTIN
    // led blink control
    static uint32_t ledtimer = millis();
    if(millis() > ledtimer) {
    ledtimer = millis()+1000;
        if(ledon) {
            digitalWrite(LED_BUILTIN, LOW);
            ledon = false;
        } else {
            digitalWrite(LED_BUILTIN, HIGH);
            ledon = true;  
        }
    }
#endif

/*
#ifdef ENABLE_RTSPSERVER
    uint32_t msecPerFrame = 100;
    static uint32_t lastimage = millis();

    // If we have an active client connection, just service that until gone
    // (FIXME - support multiple simultaneous clients)
    if(session) {
        session->handleRequests(0); // we don't use a timeout here,
        // instead we send only if we have new enough frames

        uint32_t now = millis();
        if(now > lastimage + msecPerFrame || now < lastimage) { // handle clock rollover
            session->broadcastCurrentFrame(now);
            lastimage = now;

            // check if we are overrunning our max frame rate
            now = millis();
            if(now > lastimage + msecPerFrame)
                printf("warning exceeding max frame rate of %d ms\n", now - lastimage);
        }

        if(session->m_stopped) {
            delete session;
            delete streamer;
            session = NULL;
            streamer = NULL;
        }
    }
    else {
        client = rtspServer.accept();

        if(client) {
            //streamer = new SimStreamer(&client, true);             // our streamer for UDP/TCP based RTP transport
            streamer = new OV2640Streamer(&client, cam);             // our streamer for UDP/TCP based RTP transport

            session = new CRtspSession(&client, streamer); // our threads RTSP session and state
        }
    }
#endif
*/
    // mpu6050
    if (mpuDataCounter != mpuDataCounterPrev) {

      Serial.print(" mpu ");
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);

      mpuDataCounterPrev = mpuDataCounter;
    }
}


/*

CStreamer *streamer;
CRtspSession *session;
WiFiClient client; // FIXME, support multiple clients

void loop()
{
#ifdef ENABLE_WEBSERVER
    server.handleClient();
#endif

#ifdef LED_BUILTIN
    // led blink control
    static uint32_t ledtimer = millis();
    if(millis() > ledtimer) {
    ledtimer = millis()+1000;
        if(ledon) {
            digitalWrite(LED_BUILTIN, LOW);
            ledon = false;
        } else {
            digitalWrite(LED_BUILTIN, HIGH);
            ledon = true;  
        }
    }
#endif

#ifdef ENABLE_RTSPSERVER
    uint32_t msecPerFrame = 100;
    static uint32_t lastimage = millis();

    // If we have an active client connection, just service that until gone
    // (FIXME - support multiple simultaneous clients)
    if(session) {
        session->handleRequests(0); // we don't use a timeout here,
        // instead we send only if we have new enough frames

        uint32_t now = millis();
        if(now > lastimage + msecPerFrame || now < lastimage) { // handle clock rollover
            session->broadcastCurrentFrame(now);
            lastimage = now;

            // check if we are overrunning our max frame rate
            now = millis();
            if(now > lastimage + msecPerFrame)
                printf("warning exceeding max frame rate of %d ms\n", now - lastimage);
        }

        if(session->m_stopped) {
            delete session;
            delete streamer;
            session = NULL;
            streamer = NULL;
        }
    }
    else {
        client = rtspServer.accept();

        if(client) {
            //streamer = new SimStreamer(&client, true);             // our streamer for UDP/TCP based RTP transport
            streamer = new OV2640Streamer(&client, cam);             // our streamer for UDP/TCP based RTP transport

            session = new CRtspSession(&client, streamer); // our threads RTSP session and state
        }
    }
#endif

    // mpu6050
    if (mpuDataCounter != mpuDataCounterPrev) {

      Serial.print(" mpu ");
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);

      mpuDataCounterPrev = mpuDataCounter;
    }
}
*/