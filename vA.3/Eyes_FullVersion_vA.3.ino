//NOTES

//260524  Updated EYEs control, incorporating Ai supported SD card functions.
//        Started from tested SD functional code that interpeted the SD commands.
//        and decoded commands. Then various code was transferred from 
//        Pauls_Eyes_Control_01_SD.
//260526  Added Ai assisted SUBs.
//260530  Updated from ESP8266 to ESP32-C3
//260531  Changed to ESP32-S3. C3 didn't work because of too many boot related pins.
//260608  updated eyes and brow parts with new commands.
//260610  added mouth functions
//260613  added mouth talk movements and additional mouth positions
//260613  added back in all SD and sequence functionality.
//260615  added remote code update functionality

 
// @@@@@@@@@@@@@@@@@@@@@
// @@@@ Definitions @@@@
// @@@@@@@@@@@@@@@@@@@@@

  #define Revision " Eyes_FullVersion_vA.3"
  #define ROBOT_MODEL "EYES_FullVersion"
  #define LOCAL_VERSION "A.3"

  #include <SPI.h>
  #include <SdFat.h>
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  
  #include <WiFi.h>
  #include <HTTPClient.h>
  #include <Update.h>
  #include <ArduinoJson.h>
  #include <WebServer.h>
  #include <Preferences.h>
  #include <WiFiClientSecure.h>
  
//...................................................

  WebServer server(80);
  Preferences prefs;
  String ssid = "";
  String password = "";
  const char* versionURL = "https://raw.githubusercontent.com/PDaytonP/Eyes-Releases/main/version.json";
  String firmwareBase = "https://raw.githubusercontent.com/PDaytonP/Eyes-Releases/main/";

  SdFat SD;
  SdFile file;

  //............................................
  // --- Function Prototypes ---

/* --- Function Prototypes (FINAL, COMPLETE) --- */

  // WiFi & OTA
  String wifiPage();
  void handleRoot();
  void handleSave();
  bool tryConnectSavedWiFi();
  void startAPMode();
  void setupWiFi();
  void wifiLoop();
  void checkForUpdates();

  // SD & sequence & commands
  bool CheckCardStillPresent();
  void CheckSeqControl();
  bool readCommand(String &line);
  void ScanForSubs();
  bool JumpToSub(const String &nameRaw);
  void StartSequenceFromRemote();
  String cleanName(const String &s);
  void ProcessSdData();
  bool ProcessCommand(const String &lineIn);
  void ProcessComplex(float params[]);
  uint32_t extractNumber(const String &s);
  bool CleanCommand(const String &lineIn, String &cmdOut, float params[], int &paramCount);
  void ExecuteCommand(const String &cmd, float params[], int paramCount);
  void ProcessManualCommand(String cmd);
  void CheckForSD(); //my new

  // Display / messages
  void ShowStatusScreen(const String &title, const String &message, bool halt);
  void ShowSimpleMessage(const String &line1, const String &line2, uint16_t ms);
  void ShowBigMessage(const char* line1, const char* line2, int holdMs);

  // Eyes & brows & mouth
  void ProcessEyes();
  void SmoothMoveEyes(int startX, int startY, int targetX, int targetY);
  void SmoothFloatMove(float &value, float target, int steps);
  void DrawAnalogBrows();
  void SetBrowL_Tilt(float y1Norm, float y2Norm);
  void SetBrowR_Tilt(float y1Norm, float y2Norm);
  void MoveBrowL_Unit(float normAmount);
  void MoveBrowR_Unit(float normAmount);
  void MoveBrows_Unit(float normAmount);
  void SmoothBrowMove(float &brow, float target);
  void SmoothBrowsTogether(float targetL, float targetR);
  void DrawMouth();
  void UpdateMouthTalk();
  void UpdateTalkBrows();
  void UpdateTalkBlink();
  void UpdateTalkEyes();
  void SetEyes(int EyePosX,int EyePosY);
  void WinkEye(int WhichEye);
  void SmoothDilation(int startP, int targetP, int startI, int targetI);
  void drawSmoothCircle(int x, int y, int r, int color);
  void ExprTilt(float L1n, float L2n, float R1n, float R2n);
  void ExprNormal();
  void ExprHappy();
  void ExprSad();
  void ExprAngry();
  void ExprSurprised();
  void ExprSkeptical();
  void ExprConfused();

  //...................................................

  bool sdPresent = false;      // true if SD is currently inserted & initialized
  bool sdWasPresent = false;   // used to detect insertion/removal events

  uint32_t loopCount = 0;
  uint32_t Vsd = 0;          // 0 = infinite loops
  uint32_t loopStartPos = 0; // byte position of line 2
  bool EndOfFileReached = false;   // <--- ADD THIS HERE

  String remoteBuf = "";  // put this at top of file  
  String cmd;
  bool remoteActive = false;
  String serialLine = "";

  bool SeqStart  = false;   // request to start or resume
  bool SeqStop =   false;   //true to stop, false once stopped 
  bool SeqPause  = false;   // request to pause
  bool SeqResume = false;   // request to resume
  bool IsPaused  = false;   // actual state
  bool IsStopped = false;   // actual state
 
  //error messages
  enum ErrorCode {
    ERR_NONE = 0,
    ERR_BAD_FORMAT = 1,
    ERR_UNKNOWN_COMMAND = 2,
    ERR_SUB_NOT_FOUND = 3,
    ERR_SD_READ_FAIL = 4,
    ERR_REPEAT_LINE_INVALID = 5,
    ERR_PARAM_INVALID = 6,
    ERR_UNEXPECTED_EOF = 7
  };
  void ShowErrorMessage(ErrorCode err);

//...................................................



  // ============================================================
  // SEQUENCE STATE MACHINE
  // ============================================================

  enum SeqState : uint8_t {
    STOPPED = 0,        // idle, ready to auto-start
    RUNNING = 1,
    PAUSED = 2,
    STOPPED_EOF = 3,    // finished file, no auto-start
    STOPPED_MANUAL = 4  // user stopped, no auto-start
  };

  SeqState seqState = STOPPED;
  
  // Subroutine table
  struct SubEntry {
    String name;        // uppercased
    uint32_t position;  // file position at first line AFTER "SUB name;"
  };

  const int MAX_SUBS = 10;
  SubEntry subs[MAX_SUBS];
  int subCount = 0;
  //..............................

  String line;
  String firstLine;
  
//...................................................
  //OLED and EYE Related...
  
  #define SCREEN_WIDTH 128  // OLED display width, in pixels
  #define SCREEN_HEIGHT 64  // OLED display height, in pixels

  // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
  #define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
  #define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

  //Change to invert axis
  #define INVERT_Y false
  #define INVERT_X false
  #define SWAP_XY false

  //  0  ─────────────────────────── top
  //  8–12   ─── brows baseline
  //  0–20   ─── full brow slope range
  //  28–32  ─── eyes (moved up)
  //  34–38  ─── nose (moved up)
  //  48–60  ─── mouth region (new)
  //  64 ─────────────────────────── bottom
  
  //face positions
  
  //eye positions
  int eyeRad=16;      //Radius of Sclera (outer white
  int irisRad=8;     // iris (inner black)
  int pupilRad =2;    // pupil (inner white)

  #define eyeXL 32    //Center X of Left eye 32
  #define eyeXR 84    //Center X of Right eye 84
  #define eyeY  32    //Center Y of both eyes 32
 
  // Brow endpoints (screen Y coordinates)

  #define BrowYBase 7
  float browL_y1 = BrowYBase;   // left brow, left endpoint
  float browL_y2 = BrowYBase;   // left brow, right endpoint
  float browR_y1 = BrowYBase;   // right brow, left endpoint
  float browR_y2 = BrowYBase;   // right brow, right endpoint

  float leftBrow  = 0.5;
  float rightBrow = 0.5;
  float leftBrowTilt  = 0.0;   // -1.0 to +1.0
  float rightBrowTilt = 0.0;   // -1.0 to +1.0  
  
  // NOSE POSITION (move up between eyes)
  int noseX = 58;       // centered
  int noseY = 49;       // was 45, 47
  #define NoseXD  4     //width of nose
  #define NoseYD  13    //hight of nose

  // Mouth related
  float mouthShift = 0.0;     // -1.0 left, 0 center, +1.0 right
  int mouthState = 0;         // 0=neutral,1=smile,2=frown,3=O,4=talk
  unsigned long mouthTalkEnd = 0;
  bool mouthTalkFrame = false;
  
  // Mouth Y position (fixed)
  const int mouthY = 60;      // fits new face geometry was 58
  #define MouthCtrX 58
  
  // Natural Talk Engine v1
  unsigned long nextTalkFlip = 0;
  bool talkPaused   = false;
  int  talkShape    = 0;   // 0=closed,1=open,2=O,3=half-open (optional)

  int offsetXmx = 12; //this is how far eye moves
  int offsetYmx = 12;
  int offsetXmn = 6; //this is how far eye moves
  int offsetYmn = 6;
  int xPosition = 0;
  int yPosition = 0;
//...................................................

  int16_t lookXL, lookXR, lookYL, lookYR;
  bool blink = false;
  int WinkE=0;
  int browSpeed = 8;      // default steps for smooth movement
  int eyeSpeed = 10;      // default steps for SmoothMoveEyes
  int dilateSpeed = 20;   // default number of steps for dilation  
  int dilateDelay = 20;   // ms per dilation step
 
  int DelayStat=0;  //delay steering
  int Xeye=0;       //delay steering
  int SdStat=0;     //no SD=0, SD=1   

  //Command ID definitions
  #define EyesID                1  
  #define EyesCenterID          2
  #define EyesLeftID            3
  #define EyesRightID           4
  #define EyesUpID              5
  #define EyesDownID            6
  #define EyesUpLeftID          7  
  #define EyesDownLeftID        8
  #define EyesDownRightID       9 
  #define EyeSpeedID            10
  #define DelayID               11
  #define BlinkID               12
  #define BlinkTimeID           13
  #define WinkID                14
  #define WinkLTimeID           15
  #define WinkRTimeID           16
  #define CrossEyeCenterID      17 
  #define CrossEyeCenterTimeID  18
  #define CrossEyeUpID          19
  #define CrossEyeUpTimeID      20
  #define CrossEyeDownID        21
  #define CrossEyeDownTimeID    22
  #define RollEyesCwID          23
  #define RollEyesCcwID         24
  #define DilateID              25
  #define DilateSpeedID         26
  #define DilateDelayID         27
  #define BrowsID               28  
  #define BrowsTimeID           29
  #define BrowLID               30
  #define BrowLTiltID           31
  #define BrowLTimeID           32
  #define BrowRID               33
  #define BrowRTiltID           34
  #define BrowRTimeID           35
  #define BrowSpeedID           36
  #define ExprNormalID          37
  #define ExprHappyID           38
  #define ExprSadID             39
  #define ExprAngryID           40
  #define ExprSurpriseID        41
  #define ExprSkepticalID       42
  #define ExprConfusedID        43
  #define ExprTimedID           44
  #define MouthNeutralID        45
  #define MouthSmileID          46
  #define MouthFrownID          47
  #define MouthOID              48
  #define MouthOpenID           49
  #define MouthTalkID           50
  #define MouthShiftID          51
  #define MouthTimeID           52
  #define MouthSmirkLID         53
  #define MouthSmirkRID         54
  #define MouthDiagLID          55
  #define MouthDiagRID          56
  #define MouthPeakID           57

  #define NormalID              0
  #define SmileID               1
  #define FrownlID              2
  #define OhID                  3
  #define TalkID                4
  #define OpenID                5
  #define SmirkLID              6
  #define SmirkRID              7
  #define DiagLID               8
  #define DiagRID               9
  #define PeakID                10
      
// ============================================================
// GLOBALS (UNIFIED PARAM SYSTEM + EYES + BROWS)
// ============================================================

  float params[10];      // THE ONLY PARAM ARRAY
  int paramCount = 0;    // number of parameters parsed
  
  int SwV = 0;           // command switch ID  
  int EyeCmd=0;
  
// ============================================================


//...................................................
//Software Serial Related (if used)

  int Storage[12];
  int OutData[6];
  int Length=0;
  boolean DataError=false;
  #define FunctionSelect 0  //This allows for different function sets
  
  //If pin serial is used, enable the next two lines.
  //#include <SoftwareSerial.h>
  //SoftwareSerial CommandSerial(D3,D4);  //RX,TX

  #define onboardLED 13

//_______________________________
  int packetSize;
  byte f, v1, v2, v3, v4;
  int i, j, x, y, z;
//_______________________________

//...................................................


// @@@@@@@@@@@@@@@@
// @@@@ Setups @@@@
// @@@@@@@@@@@@@@@@

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("Robot S3 Eyes Starting...");
  Serial.println(Revision);
  Serial.println();

  // -----------------------------
  // 0. CHECK FOR UPDATES
  // -----------------------------
    // Connect WiFi
    setupWiFi();
    // Check for updates
    checkForUpdates();
    
  // -----------------------------
  // 1. I2C + OLED FIRST
  // -----------------------------
  Wire.begin(8, 9);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true) delay(10);
  }
  display.clearDisplay();
  display.display();

  // -----------------------------
  // 2. SPI + SD
  // -----------------------------
  SPI.begin();
  if (!SD.begin(10, SD_SCK_MHZ(10))) {
    Serial.println("SD init failed!");
    SdStat = 0;
    ShowStatusScreen("INFO", "Insert SD", false);
    return;
  }
  SdStat = 1;
  Serial.println("SD OK");

  // -----------------------------
  // 3. Open file
  // -----------------------------
  if (!file.open("RobotEyes.txt", O_READ)) {
    Serial.println("File open failed!");
    SdStat = 0;
    ShowStatusScreen("INFO", "File not found", false);
    return;
  }
  Serial.println("RobotEyes.txt opened!");

  // -----------------------------
  // 4. Read REPEAT line
  // -----------------------------
  while (true) {
    int c = file.peek();
    if (c == 'R') break;
    if (c < 0) break;
    file.read();
  }
  if (!readCommand(firstLine)) {
    ShowErrorMessage(ERR_REPEAT_LINE_INVALID);
    return;
  }
  Vsd = extractNumber(firstLine);
  loopStartPos = file.curPosition();

  // -----------------------------
  // 5. Scan SUBs
  // -----------------------------
  file.seekSet(0);
  ScanForSubs();

  // -----------------------------
  // 6. Seek to main program start
  // -----------------------------
  file.seekSet(loopStartPos);

  // -----------------------------
  // 7. Initialize eyes
  // -----------------------------
  lookXL = eyeXL;
  lookXR = eyeXR;
  lookYL = eyeY;
  lookYR = eyeY;
  SetEyes(2,2);
  ProcessEyes();
  ProcessManualCommand("DILATE,2");

  // -----------------------------
  // 8. Initialize mouth
  // -----------------------------
  randomSeed(esp_random());   // or analogRead on a floating pin

  SeqStart = true;
  

} //>>>>>>>>>> END SETUP  <<<<<<<<<< 

//......................................................................

//********************************************************************************
//********************************************************************************
//**********                   PROGRAM  START                           **********
//********************************************************************************
//********************************************************************************
// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

    wifiLoop();
    
    // 1. Always read remote commands first
    CheckSeqControl();

    // 2. SD STATE MACHINE
    switch (seqState) {

        case STOPPED:
            if (SeqStart) {
                SeqStart = false;
                file.close();
                seqState = RUNNING;
                StartSequenceFromRemote();
                return;
            }
            SeqPause = SeqResume = SeqStop = false;
            CheckForSD();
            break;

        case STOPPED_EOF:
            if (SeqStart) {
                SeqStart = false;
                file.close();
                seqState = RUNNING;
                StartSequenceFromRemote();
                return;
            }
            SeqPause = SeqResume = SeqStop = false;
            CheckForSD();
            break;

        case STOPPED_MANUAL:
            if (SeqStart) {
                SeqStart = false;
                file.close();
                seqState = RUNNING;
                StartSequenceFromRemote();
                return;
            }
            SeqPause = SeqResume = SeqStop = false;
            CheckForSD();
            break;

        case PAUSED:
            if (SeqResume) {
                SeqResume = false;
                seqState = RUNNING;
                ShowBigMessage("RESUMING", "", 0);
                return;
            }
            if (SeqStop) {
                SeqStop = false;
                file.close();
                seqState = STOPPED_MANUAL;
                ShowBigMessage("SEQUENCE", "STOPPED", 2000);
                ShowBigMessage("SEND START", "OR RESET", 0);
                return;
            }
            SeqStart = SeqPause = false;
            CheckForSD();
            break;

        case RUNNING:
            if (SeqStop) {
                SeqStop = false;
                file.close();
                seqState = STOPPED_MANUAL;
                ShowBigMessage("SEQUENCE", "STOPPED", 2000);
                ShowBigMessage("START", "2 CONTINUE", 0);
                return;
            }
            if (SeqPause) {
                SeqPause = false;
                seqState = PAUSED;
                ShowBigMessage("RESUME", "2 CONTINUE", 0);
                ShowBigMessage("PAUSED", "   SEND     RESUME", 0);
                return;
            }

            SeqStart = SeqResume = false;

            // Process next SD command
            ProcessSdData();
            break;
    }

    // 3. ANIMATION (only when allowed)
    bool allowAnimation = false;

    if (seqState == RUNNING) allowAnimation = true;
    if (mouthState == TalkID) allowAnimation = true;

    if (allowAnimation) {
        UpdateMouthTalk();
        ProcessEyes();
    }

    delay(20);
} //Loop Again

// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFFFFFFF                     FUNCTIONS                       FFFFFFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF


//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF Web Page Related FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

//--------------------------
// Mobile‑friendly WiFi setup page
String wifiPage() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
  body {
    font-family: Arial, sans-serif;
    text-align: center;
    padding: 20px;
    font-size: 22px;
  }
  input {
    width: 90%;
    padding: 14px;
    margin: 12px 0;
    font-size: 22px;
  }
  button {
    width: 95%;
    padding: 16px;
    font-size: 24px;
    background-color: #4CAF50;
    color: white;
    border: none;
    border-radius: 8px;
  }
</style>
</head>
<body>
  <h2>Robot Eyes WiFi Setup</h2>
  <form action="/save">
    <input type="text" name="ssid" placeholder="WiFi Name (SSID)" required>
    <input type="password" name="pass" placeholder="Password" required>
    <button type="submit">Save & Reboot</button>
  </form>
</body>
</html>
)rawliteral";
}

//--------------------------
void handleRoot() {
  server.send(200, "text/html", wifiPage());
}

//--------------------------
void handleSave() {
  ssid = server.arg("ssid");
  password = server.arg("pass");

  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", password);
  prefs.end();

  server.send(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
  body {
    font-family: Arial, sans-serif;
    text-align: center;
    padding: 40px;
    font-size: 26px;
  }
  h2 {
    font-size: 32px;
    margin-bottom: 20px;
  }
</style>
</head>
<body>
  <h2>WiFi Saved!</h2>
  <p>Rebooting and connecting to your WiFi...</p>
  <p>You will be redirected automatically.</p>

  <script>
    setTimeout(function() {
      window.location.href = "http://roboteyes.local";
    }, 5000);
  </script>
</body>
</html>
)rawliteral");


  delay(1000);
  ESP.restart();
} //END handleSave

//--------------------------
bool tryConnectSavedWiFi() {
    prefs.begin("wifi", true);
    ssid = prefs.getString("ssid", "");
    password = prefs.getString("pass", "");
    prefs.end();

    if (ssid == "") return false;

    WiFi.begin(ssid.c_str(), password.c_str());

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
        delay(200);
        Serial.print(".");
    }

    return WiFi.status() == WL_CONNECTED;
} //END tryConnectSavedWiFi

//--------------------------
void startAPMode() {
    Serial.println("Starting AP mode...");
    WiFi.softAP("RobotEyes_Setup");

    server.on("/", handleRoot);
    server.on("/save", handleSave);
    server.begin();

    Serial.println("Connect to WiFi: RobotEyes_Setup");
    Serial.println("Open browser: http://192.168.4.1");
} //END startAPMode

//--------------------------
void setupWiFi() {
    Serial.println("Connecting to saved WiFi...");

    if (tryConnectSavedWiFi()) {
        Serial.println("\nConnected!");
        return;
    }

    Serial.println("\nNo saved WiFi or failed to connect.");
    startAPMode();
} //END setupWiFi

//--------------------------
void wifiLoop() {
    if (WiFi.getMode() == WIFI_AP) {
        server.handleClient();
    }
} //END wifiLoop

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF checkForUpdates FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void checkForUpdates() {
    Serial.println("Checking for OTA updates...");

    // --- Secure client for HTTPS ---
    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

    // --- Download version.json ---
    http.begin(client, versionURL);
    int httpCode = http.GET();

    if (httpCode != 200) {
        Serial.printf("Failed to download version.json, code: %d\n", httpCode);
        return;
    }

    String payload = http.getString();
    http.end();

    // --- Parse JSON ---
    StaticJsonDocument<4096> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
        Serial.println("JSON parse failed");
        return;
    }

    // --- Validate model entry ---
    JsonVariant model = doc[ROBOT_MODEL];
    if (model.isNull()) {
        Serial.println("Model entry missing in JSON");
        return;
    }

    // --- Extract version + bin name safely ---
    const char* remoteVersion = model["version"] | "";
    const char* binName       = model["bin"]     | "";

    if (strlen(remoteVersion) == 0 || strlen(binName) == 0) {
        Serial.println("Missing version or bin in JSON");
        return;
    }

    Serial.printf("Local version: %s\n", LOCAL_VERSION);
    Serial.printf("Remote version: %s\n", remoteVersion);

    if (String(remoteVersion) == LOCAL_VERSION) {
        Serial.println("Already up to date.");
        return;
    }

    Serial.println("New version available! Starting OTA...");

    // --- Build firmware URL ---
    String firmwareURL = firmwareBase + "v" + remoteVersion + "/" + binName;
    Serial.println("Firmware URL: " + firmwareURL);

    // --- Download firmware ---
    http.begin(client, firmwareURL);
    httpCode = http.GET();

    if (httpCode != 200) {
        Serial.printf("Failed to download firmware, code: %d\n", httpCode);
        return;
    }

    int len = http.getSize();
    WiFiClient *stream = http.getStreamPtr();

    if (!Update.begin(len)) {
        Serial.println("Update.begin() failed");
        return;
    }

    size_t written = Update.writeStream(*stream);
    if (written != len) {
        Serial.printf("OTA write failed: wrote %u of %u bytes\n",
                      (unsigned)written, (unsigned)len);
        return;
    }

    if (!Update.end()) {
        Serial.println("OTA end failed");
        return;
    }

    Serial.println("OTA update complete! Rebooting...");
    delay(1000);
    ESP.restart();
} //END checkForUpdates

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF CheckCardStillPresent FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

bool CheckCardStillPresent() {

    // Try to re-init SD card
    if (!SD.begin(10, SD_SCK_MHZ(10))) {
        // Card is gone → allow restart next time
        EndOfFileReached = false;
        return false;
    }

    return true;  // card present
} //END CheckCardStillPresent

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF CheckSeqControl FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void CheckSeqControl() {

    if (!Serial.available()) return;

    // Read full command (HEX or ASCII) up to semicolon OR newline
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    // ============================================================
    // PRIMARY SEQUENCE CONTROL COMMANDS
    // ============================================================
    if (cmd == "START"  || cmd == "START;")  { SeqStart  = true; return; }
    if (cmd == "STOP"   || cmd == "STOP;")   { SeqStop   = true; return; }
    if (cmd == "PAUSE"  || cmd == "PAUSE;")  { SeqPause  = true; return; }
    if (cmd == "RESUME" || cmd == "RESUME;") { SeqResume = true; return; }

    // ============================================================
    // MANUAL MODE COMMANDS
    // Only allowed when SD is NOT running a sequence
    // ============================================================
    if (seqState == STOPPED ||
        seqState == STOPPED_EOF ||
        seqState == STOPPED_MANUAL) {

        // If command ends with semicolon, treat as ASCII manual command
        if (cmd.endsWith(";")) {
            ProcessManualCommand(cmd);
            return;
        }

        // Otherwise treat as HEX command
        ProcessManualCommand(cmd);
        return;
    }

    // ============================================================
    // If SD is running or paused, ignore manual commands
    // ============================================================
} //END CheckSeqControl 

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF readCommand FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
// UNIVERSAL LINE READER (handles CRLF, LF, CR, missing newline)

bool readCommand(String &line) {
  line = "";
  bool started = false;

  while (true) {
    int c = file.read();

    // SD missing, file closed, or EOF
    if (c < 0) {
      return false;
    }

    // Accept only printable ASCII
    if (c >= 32 && c <= 126) {
      started = true;
      line += (char)c;

      if (c == ';') {
        return true;
      }
    }
    else {
      // Skip garbage BEFORE command starts
      if (!started) continue;

      // Skip garbage AFTER command starts
      continue;
    }
  }
  
} //END readCommand

//************************************************************************

//FFFFFFFFffffffffffffFFFFFFFFFFFF
//FFFFF ScanForSubs FUNCTION FFFFF
//FFFFFFFFFffffffffffffFFFFFFFFFFF

void ScanForSubs() {

  subCount = 0;

  // Safety: SD removed?

  if (!file.isOpen()){ 
    SdStat = 0;
    return;
  }

  file.seekSet(0);

  // Skip UTF‑8 BOM if present
  if (file.peek() == 0xEF) {
      file.read(); file.read(); file.read();
  }

  String line;

  while (readCommand(line)) {

      String upper = line;
      upper.trim();
      upper.toUpperCase();

      if (upper.startsWith("SUB ")) {

          // Extract raw name
          String name = line;
          name.trim();
          name.remove(0, 4);   // remove "SUB "
          name.trim();

          // Clean the name (remove semicolon, spaces, NBSP, etc.)
          name = cleanName(name);

          uint32_t subStartPos = file.curPosition();

          if (subCount < MAX_SUBS) {
              subs[subCount].name = name;
              subs[subCount].position = subStartPos;
              subCount++;

              Serial.print("Found SUB: ");
              Serial.print(name);
              Serial.print(" at pos ");
              Serial.println(subStartPos);
          }

          // Skip until END SUB
          String dummy;
          while (readCommand(dummy)) {
              String u = dummy;
              u.trim();
              u.toUpperCase();
              if (u.startsWith("END SUB")) break;
          }
      }
  }

  // Return to main program start
  if (sdPresent && file.isOpen()) {
      file.seekSet(loopStartPos);
  }
  
} //END ScanForSubs

//************************************************************************

//FFFFFFFFFFffffffffffFFFFFFFFFF
//FFFFF JumpToSub FUNCTION FFFFF
//FFFFFFFFFFffffffffffFFFFFFFFFF

bool JumpToSub(const String &nameRaw) {

  // Clean the incoming name
  String name = cleanName(nameRaw);

  for (int i = 0; i < subCount; i++) {
    if (subs[i].name == name) {
      file.seekSet(subs[i].position);
      return true;
    }
  }
  Serial.print("ERROR: SUB not found: ");
  Serial.println(nameRaw);  // show original for debugging
  ShowErrorMessage(ERR_SUB_NOT_FOUND);
  return false;
  
} //END JumpToSub

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF StartSequenceFromRemote FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void StartSequenceFromRemote() {

  if (!file.open("RobotEyes.txt", O_READ)) {
    ShowBigMessage("FORMAT", "ERROR!", 2000);
    seqState = STOPPED;
    return;
  }

  seqState = RUNNING;

  if (!readCommand(firstLine)) {
    ShowBigMessage("FORMAT", "ERROR!", 2000);
    file.close();
    seqState = STOPPED;
    return;
  }

  Vsd = extractNumber(firstLine);
  loopStartPos = file.curPosition();

  file.seekSet(0);
  ScanForSubs();
  file.seekSet(loopStartPos);

  ShowBigMessage("REMOTE", "SEQ START", 1000);
  SetEyes(2,2);
  ProcessEyes();
  
} //END StartSequenceFromRemote

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF cleanName FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
// Remove ALL non-alphanumeric characters and uppercase the result.
// This eliminates hidden spaces, tabs, NBSP, BOM, punctuation, etc.

String cleanName(const String &s) {
  String out = "";
  for (int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (isAlphaNumeric(c)) {
      out += (char)toupper(c);
    }
  }
  return out;
  
} //END cleanName

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF ProcessSdData FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void ProcessSdData() {
  Serial.println("....ENTER ProcessSdData()");

  // EARLY SD REMOVAL CHECK
  if (!CheckCardStillPresent()) {
    file.close();
    seqState = STOPPED_EOF;
    ShowBigMessage("SD REMOVED", "", 4000);
    ShowBigMessage("START", "or MANUAL", 0);
//    SetEyes(2,2);
//    ProcessEyes();
    return;
  }

  // MAIN READ LOOP
  while (true) {

    // 1. SD REMOVAL inside loop
    if (!CheckCardStillPresent()) {
      file.close();
      seqState = STOPPED_EOF;
      ShowBigMessage("SD REMOVED", "", 4000);
      ShowBigMessage("START", "or MANUAL", 0);
//      SetEyes(2,2);
//      ProcessEyes();
      return;
    }

    // 2. STOP or PAUSE during read
    if (SeqStop) {
      Serial.println("....DEBUG: STOP inside ProcessSdData()");
      SeqStop = false;
      file.close();
      seqState = STOPPED_EOF;
      return;
    }

    if (SeqPause) {
      Serial.println("....DEBUG: PAUSE inside ProcessSdData()");
      SeqPause = false;
      seqState = PAUSED;
      return;
    }

    // 3. READ NEXT COMMAND LINE
    if (!readCommand(line)) {

      // natural EOF
      if (file.curPosition() >= file.fileSize()) {
        Serial.println("....DEBUG: EOF block executed");
        break;
      }

      // SD removed mid-read
      file.close();
      seqState = STOPPED_EOF;
      ShowBigMessage("SD REMOVED", "", 4000);
      ShowBigMessage("START", "or MANUAL", 0);
//      SetEyes(2,2);
//      ProcessEyes();
      return;
    }

    line.trim();
    if (line.length() == 0) continue;

    String upper = line;
    upper.toUpperCase();

    // FILE END HANDLING
    if (upper.startsWith("FILE END")) {
      Serial.println("....Reached FILE END in main program.");

      loopCount++;

      if (Vsd > 0 && loopCount >= Vsd) {
        file.close();
        Serial.println("....DEBUG: file.close() executed at EOF");
        ShowBigMessage("SEQUENCE", "COMPLETE", 3000);
        ShowBigMessage("START", "or MANUAL", 0);

        seqState = STOPPED_EOF;
        loopCount = 0;
//        SetEyes(2,2);
//        ProcessEyes();
        return;

      } else {
        // loop again from start position
        file.seekSet(loopStartPos);
        break;
      }
    }

    // IGNORE SUB BLOCKS
    if (upper.startsWith("SUB ")) {

      String dummy;
      while (true) {

        if (!readCommand(dummy)) {

          if (file.curPosition() >= file.fileSize()) {
            Serial.println("....DEBUG: REAL EOF handler executed");
            file.close();
            seqState = STOPPED_EOF;
            loopCount = 0;
//            SetEyes(2,2);
//            ProcessEyes();
            return;
          }
          break;
        }
        dummy.trim();
        String u = dummy;
        u.toUpperCase();
        if (u.startsWith("END SUB")) break;
      }
      continue;
    }

    // NORMAL COMMAND PROCESSING
    ProcessCommand(line);

  } // END while(true)

  // NATURAL EOF (no FILE END)
  if (file.curPosition() >= file.fileSize()) {
    Serial.println("....DEBUG: REAL EOF handler executed");
    file.close();
    seqState = STOPPED_EOF;
    loopCount = 0;
//    SetEyes(2,2);
//    ProcessEyes();
    return;
  }

  // POST-LOOP SD CHECK
  if (!CheckCardStillPresent()) {
    file.close();
    seqState = STOPPED_EOF;
    ShowBigMessage("SD REMOVED", "Command Mode", 4000);
//    SetEyes(2,2);
//    ProcessEyes();
  }
  
} //END ProcessSdData 

//************************************************************************

//FFFFFFFFFFFFFFFFfffffffffffffffFFFF
//FFFFF ProcessCommand FUNCTION FFFFF
//FFFFFFFFFFFFFFFfffffffffffffffFFFFF

bool ProcessCommand(const String &lineIn) {
    String line = lineIn;
    line.trim();

    // Uppercase copy for keyword tests
    String upper = line;
    upper.toUpperCase();

    // ----------------------------------------------------
    // GOSUB HANDLER (from SD version)
    // ----------------------------------------------------
    if (upper.startsWith("GOSUB ")) {

        // Extract subroutine name
        String name = line.substring(6);
        name = cleanName(name);

        // Save return position
        uint32_t returnPos = file.curPosition();

        // Jump to SUB
        if (!JumpToSub(name)) {
            Serial.print("ERROR: SUB not found: ");
            Serial.println(name);
            ShowErrorMessage(ERR_SUB_NOT_FOUND);
            return true;
        }

        // Execute lines inside SUB
        String subLine;
        while (readCommand(subLine)) {
            subLine.trim();
            if (subLine.length() == 0) continue;

            String u = subLine;
            u.toUpperCase();

            if (u.startsWith("END SUB")) break;

            // RECURSE using the NEW ProcessCommand()
            ProcessCommand(subLine);
        }

        // Return to caller
        file.seekSet(returnPos);
        return true;
    }

    // ----------------------------------------------------
    // NORMAL COMMAND PROCESSING (new version)
    // ----------------------------------------------------
    String cmd;

    if (CleanCommand(line, cmd, params, paramCount)) {
        ExecuteCommand(cmd, params, paramCount);
    }
    else {
        Serial.print("....Invalid command: ");
        Serial.print("->"); Serial.print(line); Serial.println("<-");
        ShowErrorMessage(ERR_UNKNOWN_COMMAND);
    }

    return true;
} // END ProcessCommand

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF extractNumber FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
// EXTRACT NUMBER FROM FIRST LINE (REPEAT,n;)

uint32_t extractNumber(const String &s) {
  int comma = s.indexOf(',');
  int semi  = s.indexOf(';');
  if (comma < 0 || semi < 0) return 0;
  return s.substring(comma + 1, semi).toInt();
  
} //ENDextractNumber

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF ShowErrorMessage FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void ShowErrorMessage(ErrorCode err) {
  String msg;

  switch (err) {
    case ERR_BAD_FORMAT:          msg = "Bad command format"; break;
    case ERR_UNKNOWN_COMMAND:     msg = "Unknown command"; break;
    case ERR_SUB_NOT_FOUND:       msg = "SUB not found"; break;
    case ERR_SD_READ_FAIL:        msg = "SD read failure"; break;
    case ERR_REPEAT_LINE_INVALID: msg = "Invalid REPEAT line"; break;
    case ERR_PARAM_INVALID:       msg = "Invalid parameter"; break;
    case ERR_UNEXPECTED_EOF:      msg = "Unexpected EOF"; break;
    default:                      msg = "Unknown error"; break;
  }
  ShowStatusScreen("ERROR", msg, true);
  
} //END ShowErrorMessage

//************************************************************************

//FFFFFFFfffffffffffffffffFFFFFFFFFFFFF
//FFFFF ShowStatusScreen FUNCTION FFFFF
//FFFFFFFfffffffffffffffffFFFFFFFFFFFFF
void ShowStatusScreen(const String &title, const String &message, bool halt) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // -----------------------------
  // Rounded box (same style everywhere)
  // -----------------------------
  int boxX = 4;
  int boxY = 4;
  int boxW = SCREEN_WIDTH - 8;
  int boxH = SCREEN_HEIGHT - 8;
  int radius = 10;

  display.drawRoundRect(boxX, boxY, boxW, boxH, radius, SSD1306_WHITE);
  display.drawRoundRect(boxX + 1, boxY + 1, boxW - 2, boxH - 2, radius - 1, SSD1306_WHITE);

  // -----------------------------
  // Title (big text)
  // -----------------------------
  display.setTextSize(2);
  int16_t x1, y1;
  uint16_t w1, h1;
  display.getTextBounds(title, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_WIDTH - w1) / 2, boxY + 10);
  display.print(title);

  // -----------------------------
  // Message (small text)
  // -----------------------------
  display.setTextSize(1);
  int16_t x2, y2;
  uint16_t w2, h2;
  display.getTextBounds(message, 0, 0, &x2, &y2, &w2, &h2);
  display.setCursor((SCREEN_WIDTH - w2) / 2, boxY + 40);
  display.print(message);

  display.display();

  // -----------------------------
  // Freeze system if requested
  // -----------------------------
  if (halt) {
    while (true) {
        delay(10);  // keep WDT happy
        yield();    // REQUIRED on ESP8266
    }
  }

} //END ShowStatusScreen

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF ShowSimpleMessage FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void ShowSimpleMessage(const String &line1, const String &line2, uint16_t ms) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  int16_t x1, y1;
  uint16_t w1, h1;

  // ----- Line 1 -----
  display.getTextBounds(line1, 0, 0, &x1, &y1, &w1, &h1);
  int xCentered1 = (SCREEN_WIDTH - w1) / 2;
  int yCentered1 = 20;

  display.setCursor(xCentered1, yCentered1);
  display.println(line1);

  // ----- Line 2 -----
  if (line2.length() > 0) {
    int16_t x2, y2;
    uint16_t w2, h2;
    display.getTextBounds(line2, 0, 0, &x2, &y2, &w2, &h2);
    int xCentered2 = (SCREEN_WIDTH - w2) / 2;
    int yCentered2 = 36;

    display.setCursor(xCentered2, yCentered2);
    display.println(line2);
  }

  display.display();
  delay(ms);
  
} //END ShowSimpleMessage

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF ShowBigMessage FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void ShowBigMessage(const char* line1, const char* line2, int holdMs) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);

    int16_t x1, y1;
    uint16_t w1, h1;

    // ----- Line 1 -----
    display.getTextBounds(line1, 0, 0, &x1, &y1, &w1, &h1);
    int xCentered1 = (SCREEN_WIDTH - w1) / 2;
    int yCentered1 = 10;   // adjust vertical position as needed
    display.setCursor(xCentered1, yCentered1);
    display.print(line1);

    // ----- Line 2 -----
    display.getTextBounds(line2, 0, 0, &x1, &y1, &w1, &h1);
    int xCentered2 = (SCREEN_WIDTH - w1) / 2;
    int yCentered2 = yCentered1 + 22;  // spacing for size 2 text
    display.setCursor(xCentered2, yCentered2);
    display.print(line2);

    display.display();
    if (holdMs > 0) delay(holdMs);
    
} //END ShowBigMessage

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF CheckForSD FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void CheckForSD() {

  // Only auto-start if STOPPED
  if (seqState != STOPPED) {
    return;
  }

  // Reset SD interface
  SD.end();
  delay(10);

  // Try to initialize SD
  if (!SD.begin(10, SD_SCK_MHZ(10))) {
    return;   // no card present
  }

  Serial.println("SD inserted!");
  ShowSimpleMessage("SD INSERTED", "Loading...", 2000);

  // Try to open file
  if (!file.open("RobotEyes.txt",  O_READ)) {
    Serial.println("File open failed after insert!");
    ShowSimpleMessage("File not found", "", 3000);
    return;
  }

  // Skip garbage until REPEAT line
  while (true) {
    int c = file.peek();
    if (c == 'R') break;
    if (c < 0) break;
    file.read();
  }

  // Read REPEAT line
  if (!readCommand(firstLine)) {
    ShowErrorMessage(ERR_REPEAT_LINE_INVALID);
    file.close();
    return;
  }

  Vsd = extractNumber(firstLine);
  loopStartPos = file.curPosition();

  // Scan SUBs
  file.seekSet(0);
  ScanForSubs();

  // Jump to first command after REPEAT
  file.seekSet(loopStartPos);

  // AUTO-START SEQUENCE
  seqState = RUNNING;
  SeqStart = false;   // clear any pending manual start

  ShowBigMessage("AUTO", "SEQ START", 1000);
//  SetEyes(2,2);
//  ProcessEyes();
  
} //END CheckForSD

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF CleanCommand FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//This routime parses line, by removing "'"s and ";", makes all characters
//uppercase. Then converts numbers to integers, and store in param array.

// UNIVERSAL COMMAND PARSER
// Supports:
//   BLINK,n;
//   BLINK TIME,n;
//   WINK,n;
//   WINK TIME,n;
//   EYES,n,n;

bool CleanCommand(const String &lineIn, String &cmdOut, float params[], int &paramCount) {

    paramCount = 0;
    cmdOut = "";

    int end = lineIn.indexOf(';');
    if (end < 0) return false;

    String s = lineIn.substring(0, end);
    s.trim();
    s.toUpperCase();

    // remove spaces so "EXPR HAPPY" → "EXPRHAPPY"
    s.replace(" ", "");

    // find first comma
    int comma = s.indexOf(',');

    // no comma → no numeric params
    if (comma < 0) {
        cmdOut = s;
        return true;
    }

    // command name before first comma
    cmdOut = s.substring(0, comma);

    // parse numeric params
    int last = comma + 1;

    while (true) {
        comma = s.indexOf(',', last);
        if (comma < 0) {
            params[paramCount++] = s.substring(last).toFloat();
            break;
        }
        params[paramCount++] = s.substring(last, comma).toFloat();
        last = comma + 1;
    }

    return true;
} // END CleanCommand

//************************************************************************



  //.................................................
  //...... MY STUFF FOLLOWS .........................
  //.................................................


//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF SmoothMoveEyes FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void SmoothMoveEyes(int startX, int startY, int targetX, int targetY) {

    float dx = (targetX - startX) / float(eyeSpeed);
    float dy = (targetY - startY) / float(eyeSpeed);

    for (int i = 1; i <= eyeSpeed; i++) {
        lookXL = startX + dx * i;
        lookXR = lookXL + (eyeXR - eyeXL);
        lookYL = startY + dy * i;
        lookYR = lookYL;
        ProcessEyes();
        delay(20);   // you can also make this configurable later
    }
} //END SmoothMoveEyes

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF SmoothFloatMove FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void SmoothFloatMove(float &value, float target, int steps) {
    float start = value;
    for (int i = 1; i <= steps; i++) {
        value = start + (target - start) * (i / (float)steps);
        ProcessEyes();
        delay(15);
    }
    
} //END SmoothFloatMove

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF DrawAnalogBrows FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void DrawAnalogBrows() {

    // LEFT BROW: from (20, browL_y1) to (48, browL_y2)
    display.drawLine(20, (int)browL_y1, 48, (int)browL_y2, SSD1306_WHITE);

    // RIGHT BROW: from (72, browR_y1) to (100, browR_y2)
    display.drawLine(72, (int)browR_y1, 100, (int)browR_y2, SSD1306_WHITE);

} //END DrawAnalogBrows

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF SetBrowL_Tilt FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void SetBrowL_Tilt(float y1Norm, float y2Norm) {

    // clamp inputs to 0.0–1.0
    if (y1Norm < 0.0) y1Norm = 0.0;
    if (y1Norm > 1.0) y1Norm = 1.0;
    if (y2Norm < 0.0) y2Norm = 0.0;
    if (y2Norm > 1.0) y2Norm = 1.0;

    float targetY1 = BrowYBase - y1Norm * 10.0;
    float targetY2 = BrowYBase - y2Norm * 10.0;

    // clamp final Y
    if (targetY1 < 0) targetY1 = 0;
    if (targetY1 > BrowYBase) targetY1 = BrowYBase;
    if (targetY2 < 0) targetY2 = 0;
    if (targetY2 > BrowYBase) targetY2 = BrowYBase;

    SmoothFloatMove(browL_y1, targetY1, browSpeed);
    SmoothFloatMove(browL_y2, targetY2, browSpeed);
    
} //END SetBrowR_Tilt

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF SetBrowR_Tilt FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void SetBrowR_Tilt(float y1Norm, float y2Norm) {

    // clamp inputs to 0.0–1.0
    if (y1Norm < 0.0) y1Norm = 0.0;
    if (y1Norm > 1.0) y1Norm = 1.0;
    if (y2Norm < 0.0) y2Norm = 0.0;
    if (y2Norm > 1.0) y2Norm = 1.0;

    float targetY1 = BrowYBase - y1Norm * 10.0;
    float targetY2 = BrowYBase - y2Norm * 10.0;

    // clamp final Y
    if (targetY1 < 0) targetY1 = 0;
    if (targetY1 > BrowYBase) targetY1 = BrowYBase;
    if (targetY2 < 0) targetY2 = 0;
    if (targetY2 > BrowYBase) targetY2 = BrowYBase;

    SmoothFloatMove(browR_y1, targetY1, browSpeed);
    SmoothFloatMove(browR_y2, targetY2, browSpeed);
}

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF MoveBrowL_Unit FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void MoveBrowL_Unit(float normAmount) {
    float delta = normAmount * 10.0;  // scale to pixels

    float new_y1 = browL_y1 - delta;
    float new_y2 = browL_y2 - delta;

    // clamp so higher endpoint never goes off-screen
    float minY = (new_y1 < new_y2) ? new_y1 : new_y2;
    if (minY < 0) {
        float overshoot = -minY;
        new_y1 += overshoot;
        new_y2 += overshoot;
    }

    SmoothFloatMove(browL_y1, new_y1, browSpeed);
    SmoothFloatMove(browL_y2, new_y2, browSpeed);
    
} //END MoveBrowL_Unit

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF MoveBrowR_Unit FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void MoveBrowR_Unit(float normAmount) {
    float delta = normAmount * 10.0;

    float new_y1 = browR_y1 - delta;
    float new_y2 = browR_y2 - delta;

    float minY = (new_y1 < new_y2) ? new_y1 : new_y2;
    if (minY < 0) {
        float overshoot = -minY;
        new_y1 += overshoot;
        new_y2 += overshoot;
    }

    SmoothFloatMove(browR_y1, new_y1, browSpeed);
    SmoothFloatMove(browR_y2, new_y2, browSpeed);
    
} //END MoveBrowR_Unit

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF SmoothBrowPair FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void SmoothBrowPair(float &y1a, float target1a,
                    float &y2a, float target2a,
                    float &y1b, float target1b,
                    float &y2b, float target2b,
                    int steps)
{
    float s1a = y1a, s2a = y2a;
    float s1b = y1b, s2b = y2b;

    for (int i = 1; i <= steps; i++) {

        float t = i / (float)steps;

        y1a = s1a + (target1a - s1a) * t;
        y2a = s2a + (target2a - s2a) * t;

        y1b = s1b + (target1b - s1b) * t;
        y2b = s2b + (target2b - s2b) * t;

        ProcessEyes();
        delay(15);
    }
}//END SmoothBrowPair

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF MoveBrows_Unit FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void MoveBrows_Unit(float normAmount) {

    float delta = normAmount * 10.0;

    float newL1 = browL_y1 - delta;
    float newL2 = browL_y2 - delta;
    float newR1 = browR_y1 - delta;
    float newR2 = browR_y2 - delta;

    // clamp highest endpoint
    float minY = newL1;
    if (newL2 < minY) minY = newL2;
    if (newR1 < minY) minY = newR1;
    if (newR2 < minY) minY = newR2;

    if (minY < 0) {
        float overshoot = -minY;
        newL1 += overshoot;
        newL2 += overshoot;
        newR1 += overshoot;
        newR2 += overshoot;
    }

    SmoothBrowPair(
        browL_y1, newL1, browL_y2, newL2,
        browR_y1, newR1, browR_y2, newR2,
        browSpeed
    );

    ProcessEyes();
    
} //END MoveBrows_Unit

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF BrowTimed FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void BrowTimed(float &brow, float target, int holdMs) {
    SmoothBrowMove(brow, target);
    delay(holdMs);
    SmoothBrowMove(brow, 0.5);
} //END BrowTimed

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF SmoothBrowMove FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void SmoothBrowMove(float &brow, float target) {

    float start = brow;

    for (int i = 1; i <= browSpeed; i++) {
        brow = start + (target - start) * (i / (float)browSpeed);
        ProcessEyes();
        delay(15);
    }
} //END SmoothBrowMove

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF SmoothDilation FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void SmoothDilation(int startP, int targetP, int startI, int targetI) {

    for (int i = 1; i <= dilateSpeed; i++) {

        pupilRad = startP + (targetP - startP) * (i / (float)dilateSpeed);
        irisRad  = startI + (targetI - startI) * (i / (float)dilateSpeed);

        ProcessEyes();
        delay(dilateDelay);   // user‑adjustable
    }
} //END SmoothDilation

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF drawSmoothCircle FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void drawSmoothCircle(int x, int y, int r, int color) {
    for (int i = 0; i < 2; i++) {   // 2‑pixel thickness
        display.drawCircle(x, y, r - i, color);
    }
} //END drawSmoothCircle

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF ExprTilt FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void ExprTilt(float L1n, float L2n, float R1n, float R2n) {

    float L1 = BrowYBase - L1n * 10.0;
    float L2 = BrowYBase - L2n * 10.0;
    float R1 = BrowYBase - R1n * 10.0;
    float R2 = BrowYBase - R2n * 10.0;

    SmoothBrowPair(
        browL_y1, L1, browL_y2, L2,
        browR_y1, R1, browR_y2, R2,
        browSpeed
    );
} //END ExprTilt

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF EXPRESSION FUNCTIONS FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void ExprNormal()    { ExprTilt(0.5, 0.5, 0.5, 0.5); }
void ExprHappy()     { ExprTilt(0.6, 0.4, 0.4, 0.6); }
void ExprSad()       { ExprTilt(0.3, 0.7, 0.7, 0.3); }
void ExprAngry()     { ExprTilt(0.8, 0.1, 0.1, 0.8); }
void ExprSurprised() { ExprTilt(0.3, 0.3, 0.3, 0.3); }
void ExprSkeptical() { ExprTilt(0.3, 0.7, 0.5, 0.5); }
void ExprConfused()  { ExprTilt(0.4, 0.2, 0.2, 0.4); }

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF SmoothBrowsTogether FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void SmoothBrowsTogether(float targetL, float targetR) {

    float startL = leftBrow;
    float startR = rightBrow;

    for (int i = 1; i <= browSpeed; i++) {

        leftBrow  = startL + (targetL - startL) * (i / (float)browSpeed);
        rightBrow = startR + (targetR - startR) * (i / (float)browSpeed);

        ProcessEyes();
        delay(15);
    }
} //END SmoothBrowsTogether

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF DrawMouth FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void DrawMouth() {

    int centerX = MouthCtrX + (int)(mouthShift * 12);  // shift left/right

    switch (mouthState) {

        case NormalID: // NEUTRAL ----
            display.drawLine(centerX - 8, mouthY, centerX + 8, mouthY, SSD1306_WHITE);
        break;

        case SmileID: // SMILE  "\__/"
            display.drawLine(centerX - 8, mouthY - 4,
                             centerX - 4, mouthY, SSD1306_WHITE);
            display.drawLine(centerX + 4, mouthY,
                             centerX + 8, mouthY - 4, SSD1306_WHITE);
            display.drawLine(centerX - 4, mouthY,
                             centerX + 4, mouthY, SSD1306_WHITE);
        break;

        case FrownlID: // FROWN  "/‾‾\"   
            display.drawLine(centerX - 8, mouthY,
                             centerX - 4, mouthY - 4,SSD1306_WHITE);
            display.drawLine(centerX + 8, mouthY,
                             centerX + 4, mouthY - 4, SSD1306_WHITE);
            display.drawLine(centerX - 4, mouthY - 4,
                             centerX + 4, mouthY - 4, SSD1306_WHITE);
        break;
        
        case OpenID: // open mouth
            //left side
            display.drawLine(centerX - 4, mouthY,
                             centerX - 8, mouthY - 2, SSD1306_WHITE);
            display.drawLine(centerX - 4, mouthY - 4,
                             centerX - 8, mouthY - 2, SSD1306_WHITE);

            //right side
            display.drawLine(centerX + 4, mouthY,
                             centerX + 8, mouthY - 2, SSD1306_WHITE);
            display.drawLine(centerX + 4, mouthY - 4,
                             centerX + 8, mouthY - 2, SSD1306_WHITE);
         
            //top & bottom
            display.drawLine(centerX - 4, mouthY - 4,
                             centerX + 4, mouthY - 4, SSD1306_WHITE);
            display.drawLine(centerX - 4, mouthY,
                             centerX + 4, mouthY, SSD1306_WHITE);
        break;

        case OhID: // O mouth
            display.drawCircle(centerX, mouthY -1, 4, SSD1306_WHITE);
        break;

        case TalkID: // Natural TALK
          switch (talkShape) {
            case 0: // closed
              display.drawLine(centerX - 8, mouthY,
                             centerX + 8, mouthY, SSD1306_WHITE);
            break;

            case 1: // open (your oval mouth)
              // left side
              display.drawLine(centerX - 4, mouthY,
                               centerX - 8, mouthY - 2, SSD1306_WHITE);
              display.drawLine(centerX - 4, mouthY - 4,
                               centerX - 8, mouthY - 2, SSD1306_WHITE);
              // right side
              display.drawLine(centerX + 4, mouthY,
                               centerX + 8, mouthY - 2, SSD1306_WHITE);
              display.drawLine(centerX + 4, mouthY - 4,
                               centerX + 8, mouthY - 2, SSD1306_WHITE);
              // top & bottom
              display.drawLine(centerX - 4, mouthY - 4,
                               centerX + 4, mouthY - 4, SSD1306_WHITE);
              display.drawLine(centerX - 4, mouthY,
                               centerX + 4, mouthY, SSD1306_WHITE);
            break;

            case 2: // O
              display.drawCircle(centerX, mouthY - 1, 4, SSD1306_WHITE);
            break;

            case 3: // half-open (optional, softer look)
            display.drawLine(centerX - 6, mouthY,
                             centerX + 6, mouthY, SSD1306_WHITE);
            display.drawLine(centerX - 3, mouthY - 4,
                             centerX + 3, mouthY - 4, SSD1306_WHITE);
            display.drawLine(centerX - 3, mouthY - 4,
                             centerX - 6, mouthY, SSD1306_WHITE);
            display.drawLine(centerX + 3, mouthY - 4,
                             centerX + 6, mouthY, SSD1306_WHITE);
            break;
/*
            case 3: // half-open (optional, softer look)
              display.drawLine(centerX - 6, mouthY,
                             centerX + 6, mouthY, SSD1306_WHITE);
              display.drawLine(centerX - 4, mouthY - 2,
                             centerX + 4, mouthY - 2, SSD1306_WHITE);
            break;
*/
          }
        break;

        case SmirkLID: // SMIRKL  "^  "   
            display.drawLine(centerX + 4, mouthY,
                             centerX + 6, mouthY, SSD1306_WHITE);
            display.drawLine(centerX + 6, mouthY,
                             centerX + 8, mouthY - 2, SSD1306_WHITE);
            display.drawLine(centerX + 8, mouthY - 2,
                             centerX + 8, mouthY - 4, SSD1306_WHITE);
        break;

        case SmirkRID: // SMIRKR  "  ^"   
            display.drawLine(centerX - 4, mouthY,
                             centerX - 6, mouthY, SSD1306_WHITE);
            display.drawLine(centerX - 6, mouthY,
                             centerX - 8, mouthY - 2, SSD1306_WHITE);
            display.drawLine(centerX - 8, mouthY - 2,
                             centerX - 8, mouthY - 4, SSD1306_WHITE);
        break;

        case PeakID: // PEAK  "/‾\"   
            display.drawLine(centerX - 6, mouthY,
                             centerX + 6, mouthY, SSD1306_WHITE);
            display.drawLine(centerX - 3, mouthY - 4,
                             centerX + 3, mouthY - 4, SSD1306_WHITE);
            display.drawLine(centerX - 3, mouthY - 4,
                             centerX - 6, mouthY, SSD1306_WHITE);
            display.drawLine(centerX + 3, mouthY - 4,
                             centerX + 6, mouthY, SSD1306_WHITE);
        break;

        case DiagLID: // DIAGL  "^  "   
            display.drawLine(centerX, mouthY,
                             centerX + 8, mouthY - 4, SSD1306_WHITE);
        break;

        case DiagRID: // DIAGR  "  ^"   
            display.drawLine(centerX, mouthY,
                             centerX - 8, mouthY - 4, SSD1306_WHITE);
        break;
        
    } //END SWITCH (mouthState) 
    
} //END DrawMouth

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF UpdateMouthTalk FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void UpdateMouthTalk() {

  if (mouthState == TalkID && millis() < mouthTalkEnd) {

    // time to flip syllable / pause?
    if (millis() > nextTalkFlip) {

      // occasional micro-pause
      if (random(0, 10) == 0) {   // ~10% chance
        talkPaused = true;
        mouthTalkFrame = false; // force closed
        nextTalkFlip = millis() + random(80, 200);  // pause length
      }
      else{
        talkPaused = false;
        mouthTalkFrame = !mouthTalkFrame;           // flip open/closed

        // choose a shape for this syllable
        if (mouthTalkFrame) {
          // open-phase: pick one of several shapes
          int r = random(0, 3);   // 0,1,2
          talkShape = r + 1;      // 1=open,2=O,3=half-open (if you use it)
        } 
        else {
          // closed-phase
          talkShape = 0;
        }

        // next syllable duration
        nextTalkFlip = millis() + random(70, 190);  // tweak to taste
      }
    }
  }
  else if (mouthState == TalkID && millis() >= mouthTalkEnd) {
    // reset mouth
    mouthState = NormalID;
    mouthTalkFrame = false;
    talkShape = 0;
    talkPaused = false;

    // reset brows
    SmoothBrowPair(
      browL_y1, BrowYBase - 0.5 * 10,
      browL_y2, BrowYBase - 0.5 * 10,
      browR_y1, BrowYBase - 0.5 * 10,
      browR_y2, BrowYBase - 0.5 * 10,
      browSpeed
    );
    // reset eyes
    SmoothMoveEyes(lookXL, lookYL, eyeXL, eyeY);    // center

    // reset blink
    blink = 0;

    // reset micro-movement offsets
    lookXL = eyeXL;
    lookXR = eyeXR;
    lookYL = eyeY;
    lookYR = eyeY;

    // force a clean redraw
    ProcessEyes();    
  }
} //END UpdateMouthTalk

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF ProcessManualCommand FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//This function checks source of command input. It could fead from
//  serial monitor, serial pin, SD card, voice module, or phone app.

void ProcessManualCommand(String cmd) {

    String raw = cmd;

    if (CleanCommand(raw, cmd, params, paramCount)) {
        ExecuteCommand(cmd, params, paramCount);
    } 
//    else {
//        Serial.print("Invalid command: ");
//        Serial.println(cmd);
//    }
         
  //--------------------------------------
  //This inputs data from phone app

  //ProcessPhone;    //processes phone command
  
  //--------------------------------------
  //This inputs data from Serial pin  
/*
  else
  if(CommandSerial.available()!=0){
    Serial.println();Serial.println(" ...Received from Serial!");
    line=CommandSerial.readString();
  }
*/
//....................................................
  
}   //END ProcessManualCommand

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF ExecuteCommand FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//This function decodes commands, and converts to function ID, and 
// includes any associated data, and sends to ProcessComplex(params).
// ProcessComplex(params) carries out the EYE function.

void ExecuteCommand(const String &cmd, float params[], int paramCount) {

  //------------- Decode Commands ------------------------------------
  
  if (cmd == "EYES") {
    if (paramCount >= 2) {
      SwV=EyesID;
      ProcessComplex(params);
    } 
    else {
      Serial.println("EYES requires 2 parameters!");
    }
  }
  else if (cmd == "EYESPEED") {
    SwV=EyeSpeedID;
    ProcessComplex(params);
  }
  else if (cmd == "EYECENTER") {
    SwV=EyesCenterID;
    ProcessComplex(params);
  }
  else if (cmd == "EYESLEFT") {
    // EYES LEFT;
    Serial.println("Eyes LEFT");
    SwV=EyesLeftID;
    ProcessComplex(params);
  }
  else if (cmd == "EYESRIGHT") {
    // EYES RIGHT;
    Serial.println("Eyes RIGHT");
    SwV=EyesRightID;
    ProcessComplex(params);
  }
   else if (cmd == "EYESUP") {
    // EYES UP;
    Serial.println("Eyes UP");
    SwV=EyesUpID;
    ProcessComplex(params);
  }
   else if (cmd == "EYESDOWN") {
    // EYES DOWN; 
    Serial.println("Eyes DOWN");
    SwV=EyesDownID;
    ProcessComplex(params);
  }
   else if (cmd == "EYESUPLEFT") {
    // EYES UP LEFT;
    Serial.println("Eyes UP LEFT");
    SwV=EyesUpLeftID;
    ProcessComplex(params);
  }
   else if (cmd == "EYESDOWNLEFT") {
    // EYES DOWN LEFT;
    Serial.println("Eyes Down Left");
    SwV=EyesDownLeftID;
    ProcessComplex(params);
  }
   else if (cmd == "EYESDOWNRIGHT") {
    // EYES DOWN RIGHT;
    Serial.println("Eyes DOWN RIGHT");
    SwV=EyesDownRightID;
    ProcessComplex(params);
  }
  else if (cmd == "BLINK") {
    Serial.print("Blink=");
    Serial.println(params[0]);
    SwV=BlinkID;
    ProcessComplex(params);
  }
  else if (cmd == "BLINKTIME") {
    Serial.print("Setting blink time ");
    Serial.println(params[0]);
    SwV=BlinkTimeID;
    ProcessComplex(params);
  }
  else if (cmd == "WINK") {
    Serial.print("Wink=");
    Serial.println(params[0]);
    SwV=WinkID;
    ProcessComplex(params);
  }
  else if (cmd == "WINKLTIME") {
    Serial.print("WinkLTime=");
    Serial.println(params[0]);
    SwV=WinkLTimeID;
    ProcessComplex(params);
  }
  else if (cmd == "WINKRTIME") {
    Serial.print("WinkRTime=");
    Serial.println(params[0]);
    SwV=WinkRTimeID;
    ProcessComplex(params);
  }
  else if (cmd == "CROSSEYESCENTER") {
    Serial.print("CrossEyesCenter=");
    Serial.println(params[0]);
    SwV=CrossEyeCenterID;
    ProcessComplex(params);
  }
  else if (cmd == "CROSSEYESCENTERTIME") {
    Serial.print("CrossEyesCenterTime=");
    Serial.println(params[0]);
    SwV=CrossEyeCenterTimeID;
    ProcessComplex(params);
  }
  else if (cmd == "CROSSEYESUP") {
    Serial.print("CrossEyesUp=");
    Serial.println(params[0]);
    SwV=CrossEyeUpID;
    ProcessComplex(params);
  }
  else if (cmd == "CROSSEYESUPTIME") {
    Serial.print("CrossEyesUpTime=");
    Serial.println(params[0]);
    SwV=CrossEyeUpTimeID;
    ProcessComplex(params);
  }
  else if (cmd == "CROSSEYESDOWN") {
    Serial.print("CrossEyesDown=");
    Serial.println(params[0]);
    SwV=CrossEyeDownID;
    ProcessComplex(params);
  }
  else if (cmd == "CROSSEYESDOWNTIME") {
    Serial.print("CrossEyesDownTime=");
    Serial.println(params[0]);
    SwV=CrossEyeDownTimeID;
    ProcessComplex(params);
  }
  else if (cmd == "ROLLEYESCW") {
    Serial.print("RollEyesCW=");
    Serial.println(params[0]);
    SwV=RollEyesCwID;
    ProcessComplex(params);
  }
  else if (cmd == "ROLLEYESCCW") {
    Serial.print("RollEyesCCW=");
    Serial.println(params[0]);
    SwV=RollEyesCcwID;
    ProcessComplex(params);
  }
  else if (cmd == "DELAY") {
    SwV=DelayID;
    ProcessComplex(params);
  }
  else if (cmd == "DILATE") {
    Serial.print("Dilate=");
    Serial.println(params[0]);
    SwV = DilateID;   // new ID for dilation
    ProcessComplex(params);
  }
  else if (cmd == "DILATESPEED") {
    SwV = DilateSpeedID; 
    ProcessComplex(params);
  }
  else if (cmd == "DILATEDELAY") {
    SwV = DilateDelayID; 
    ProcessComplex(params);
  }
  else if (cmd == "BROWL") {
    SwV = BrowLID; 
    ProcessComplex(params);
  }
  else if (cmd == "BROWLTILT") {
    SwV = BrowLTiltID; 
    ProcessComplex(params);
  }
  else if (cmd == "BROWLTIME") {
    Serial.print("BrowLTime=");
    Serial.print(params[0]);Serial.print(",");
            Serial.print(params[1]);
    SwV=BrowLTimeID;
    ProcessComplex(params);
  }
  else if (cmd == "BROWR") {
    SwV = BrowRID; 
    ProcessComplex(params);
  }
  else if (cmd == "BROWRTILT") {
    SwV = BrowRTiltID; 
    ProcessComplex(params);
  }
  else if (cmd == "BROWRTIME") {
    Serial.print("BrowLRime=");
    Serial.print(params[0]);Serial.print(",");
            Serial.print(params[1]);
    SwV=BrowRTimeID;
    ProcessComplex(params);
  }
  else if (cmd == "BROWS") {
    SwV = BrowsID; 
    ProcessComplex(params);
  }
  else if (cmd == "BROWSPEED") {
    SwV = BrowSpeedID; 
    ProcessComplex(params);
  }
  else if (cmd == "EXPRNORMAL"){
     SwV = ExprNormalID;   
     ProcessComplex(params); 
  }
  else if (cmd == "EXPRHAPPY"){ 
    SwV = ExprHappyID;    
    ProcessComplex(params); 
  }
  else if (cmd == "EXPRSAD"){
    SwV = ExprSadID;      
    ProcessComplex(params); 
  }
  else if (cmd == "EXPRANGRY"){
    SwV = ExprAngryID;    
    ProcessComplex(params); 
  }
  else if (cmd == "EXPRSURPRISE"){
    SwV = ExprSurpriseID; 
    ProcessComplex(params); 
  }
  else if (cmd == "EXPRSKEPTICAL"){
    SwV = ExprSkepticalID;
    ProcessComplex(params); 
  }
  else if (cmd == "EXPRCONFUSED"){ 
    SwV = ExprConfusedID; 
    ProcessComplex(params); 
  }  
  else if (cmd == "EXPRTIME") {
    Serial.print("ExprTime=");
    Serial.print(params[0]); Serial.print(",");
    Serial.println(params[1]);
    SwV = ExprTimedID;
    ProcessComplex(params);
  } 
  else if (cmd == "MOUTHNEUTRAL") {
    SwV = MouthNeutralID;
    ProcessComplex(params);
  } 
  else if (cmd == "MOUTHSMILE") {
    SwV = MouthSmileID;
    ProcessComplex(params);
  } 
  else if (cmd == "MOUTHFROWN") {
    SwV = MouthFrownID;
    ProcessComplex(params);
  } 
  else if (cmd == "MOUTHO") {
    SwV = MouthOID;
    ProcessComplex(params);
  } 
  else if (cmd == "MOUTHOPEN") {
    SwV = MouthOpenID;
    ProcessComplex(params);
  } 
  else if (cmd == "MOUTHSMIRKL") {
    SwV = MouthSmirkLID;
    ProcessComplex(params);
  } 
  else if (cmd == "MOUTHSMIRKR") {
    SwV = MouthSmirkRID;
    ProcessComplex(params);
  } 
  else if (cmd == "MOUTHTALK") {
    SwV = MouthTalkID;
    ProcessComplex(params);
  } 
  else if (cmd == "MOUTHSHIFT") {
    SwV = MouthShiftID;
    ProcessComplex(params);
  }
  else if (cmd == "MOUTHTIME") {
    SwV = MouthTimeID;
    ProcessComplex(params);
  }
  else if (cmd == "MOUTHDIAGL") {
    SwV = MouthDiagLID;
    ProcessComplex(params);
  }
  else if (cmd == "MOUTHDIAGR") {
    SwV = MouthDiagRID;
    ProcessComplex(params);
  }
  else if (cmd == "MOUTHPEAK") {
    SwV = MouthPeakID;
    ProcessComplex(params);
  }


  //..................................................
  
  ProcessEyes();
  
} //END ExecuteCommand

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF ProcessComplex FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//This proccesses complex decoded functions. Essentially it makes changes
//  to various EYE control variables. EYEs are then updated including
//  any updated variables.

void ProcessComplex(float params[]){
  
//************************************************
  switch(SwV){
         
      //---------------------------
      case EyesID:
      {     
        //---- Eyes ---------------
        Serial.println("Processing Eyes...");

        // 1. Save current positions
        int startXL = lookXL;
        int startXR = lookXR;
        int startYL = lookYL;
        int startYR = lookYR;

        // 2. Compute target using your existing logic
        SetEyes(params[0], params[1]);

        // 3. Smoothly animate from start → target
        SmoothMoveEyes(startXL, startYL, lookXL, lookYL);
      }
      break;
         
      //---------------------------
      case EyeSpeedID:   // EYE SPEED
      {
        eyeSpeed = params[0];
        if (eyeSpeed < 1)  eyeSpeed = 1;
        if (eyeSpeed > 40) eyeSpeed = 40;
        Serial.print("Eye speed set to ");
        Serial.println(eyeSpeed);
      }
      break;  

      //---------------------------
      case EyesCenterID:     
        //---- Eyes Center --------
        Serial.println("Processing Eyes Center...");
        SetEyes(2,2);
        ProcessEyes();
      break;
     
      //---------------------------
      case EyesLeftID:     
        //---- Eyes Left ----------
        Serial.println("Processing Eyes Left...");
        SetEyes(0,2);
        ProcessEyes();
      break;
     
      //---------------------------
      case EyesRightID:     
        //---- Eyes Right ---------
        Serial.println("Processing Eyes Right...");
        SetEyes(4,2);
        ProcessEyes();
      break;
     
      //---------------------------
      case EyesUpID:     
        //---- Eyes Up ------------
        Serial.println("Processing Eyes Up...");
        SetEyes(2,0);
        ProcessEyes();
      break;
     
      //---------------------------
      case EyesDownID:     
        //---- Eyes Down ---------
        Serial.println("Processing Eyes Down...");
        SetEyes(2,4);
        ProcessEyes();
      break;
     
      //---------------------------
      case EyesUpLeftID:     
        //---- Eyes Up Left -------
        Serial.println("Processing Eyes Up Left...");
        SetEyes(1,1);
        ProcessEyes();
      break;
     
      //---------------------------
      case EyesDownLeftID:     
        //---- Eyes Down Left -----
        Serial.println("Processing Eyes Down Left...");
        SetEyes(1,3);
        ProcessEyes();
      break;
     
      //---------------------------
      case EyesDownRightID:     
        //---- Eyes Down Right ----
        Serial.println("Processing Eyes Down Right...");
        SetEyes(3,3);
        ProcessEyes();
      break;
     
      //---------------------------
      case BlinkID:     
        //---- Blink --------------
        Serial.println("Processing Blink...");
        blink=params[0];
        ProcessEyes();
      break;
     
      //---------------------------
      case BlinkTimeID:     
        //---- Blink Time ---------
        Serial.println("Processing Blink Time...");
        blink=1;
        ProcessEyes();
        delay(params[0]);
        blink=0;
        ProcessEyes();
      break;
     
      //---------------------------   
      case WinkID:     
        //---- Wink ---------------
        Serial.println("Processing Wink...");
        WinkEye(params[0]);
        break;
     
      //---------------------------     
      case WinkLTimeID:     
        //---- Wink Time --------
        Serial.println("Processing Wink Left Time...");
        WinkEye(1);
        delay(params[0]);
        WinkEye(0);
      break;
     
      //---------------------------     
      case WinkRTimeID:     
        //---- Eyes Center --------
        Serial.println("Processing Wink Right Time...");
        WinkEye(2);
        delay(params[0]);
        WinkEye(0);
      break;
     
      //---------------------------
      case CrossEyeCenterID:     
        //---- CROSS EYES --------------
        if(params[0]==1){            
          //setup eye position for cross
          //left eye 4: 
          lookYL = eyeY;
          lookXL = eyeXL + offsetXmx;    
          //right eye 0: 
          lookYR = eyeY;
          lookXR = eyeXR - offsetXmx; 
          ProcessEyes();
        }
        else
        if(params[0]==0){   //setup eye normal position
          SetEyes(2,2);
          ProcessEyes();
        }
      break;

      //---------------------------
      case CrossEyeCenterTimeID:     
        //setup eye position for cross
        //left eye 4: 
        lookYL = eyeY;
        lookXL = eyeXL + offsetXmx;    
        //right eye 0: 
        lookYR = eyeY;
        lookXR = eyeXR - offsetXmx; 
        ProcessEyes();
        delay(params[0]);
        //setup eye position for center
        Xeye=0;
        SetEyes(2,2);
        ProcessEyes();
      break;

      //---------------------------
      case CrossEyeUpID:     
        //setup eye position for cross up
        // NOTES:
        //  L  3,1  3: lookXL = eyeXL + offsetXmn;        
        //      1: lookYL = eyeY - offsetYmn;
        //
        //  R  1,1  1: lookXR = eyeXR - offsetXmn;
        //      1: lookYR = eyeY - offsetYmn;

        if(params[0]==0){   //setup eye normal position
          SetEyes(2,2);
          ProcessEyes();
        }
        else if(params[0]==1){            
          //setup eye position for cross up
          //left eye 3,1: 
          lookXL = eyeXL + offsetXmn;        
          lookYL = eyeY - offsetYmn;
          //right eye 1,1: 
          lookXR = eyeXR - offsetXmn;
          lookYR = eyeY - offsetYmn;
          ProcessEyes();
        } //END else
      break;

      //---------------------------
      case CrossEyeUpTimeID:     
        //setup eye position for cross up time
        //  L  3,1  3: lookXL = eyeXL + offsetXmn;        
        //    1: lookYL = eyeY - offsetYmn;
        // R  1,1  1: lookXR = eyeXR - offsetXmn;
        //    1: lookYR = eyeY - offsetYmn;
                   
        //left eye 3,1: 
        lookXL = eyeXL + offsetXmn;        
        lookYL = eyeY - offsetYmn;
        //right eye 1,1: 
        lookXR = eyeXR - offsetXmn;
        lookYR = eyeY - offsetYmn;
        ProcessEyes();
        delay(params[0]);
        //setup eye position for center
        Xeye=0;
        DelayStat=0;
        SetEyes(2,2);
        ProcessEyes();
      break;

      //---------------------------
      case CrossEyeDownID:  //eyes cross down
        //setup eye position for cross down
        //   L  3,3  3: lookXL = eyeXL + offsetXmn;        
        //      3: lookYL = eyeY + offsetYmn;

        //   R  1,3  1: lookXR = eyeXR - offsetXmn;
        //      1: lookYR = eyeY + offsetYmn;

        if(params[0]==0){   //setup eye normal position
          SetEyes(2,2);
          ProcessEyes();
        }
        else if(params[0]==1){            
          //setup eye position for cross down
          //left eye 3,3: 
          lookXL = eyeXL + offsetXmn;        
          lookYL = eyeY + offsetYmn;
          //right eye 1,3: 
          lookXR = eyeXR - offsetXmn;
          lookYR = eyeY + offsetYmn;
          ProcessEyes();
        } //END else
      break;

      //---------------------------
      case CrossEyeDownTimeID:  //eyes cross down time
        //setup eye position for cross down time
        //   L  3,3  3: lookXL = eyeXL + offsetXmn;        
        //      3: lookYL = eyeY + offsetYmn;

        //   R  1,3  1: lookXR = eyeXR - offsetXmn;
        //      1: lookYR = eyeY + offsetYmn;
      
        //left eye 3,3: 
        lookXL = eyeXL + offsetXmn;        
        lookYL = eyeY + offsetYmn;
        //right eye 1,3: 
        lookXR = eyeXR - offsetXmn;
        lookYR = eyeY + offsetYmn;
        ProcessEyes();
        delay(params[0]);
        //setup eye position for center
        Xeye=0;
        DelayStat=0;
        SetEyes(2,2);
        ProcessEyes();
      break;
          
    //---------------------------
    case RollEyesCwID:     
      Serial.println("Processing ROLL EYES CW...");
      for(i=1;i<=params[0];i++){
        SetEyes(3,1);
        ProcessEyes();
        delay(20);
           
        SetEyes(4,2);
        ProcessEyes();
        delay(20);
            
        SetEyes(3,3);
        ProcessEyes();
        delay(20);
            
        SetEyes(2,4 );
        ProcessEyes();
        delay(20);
           
        SetEyes(1,3);
        ProcessEyes();
        delay(20);
           
        SetEyes(0,2);
        ProcessEyes();
        delay(20);
            
        SetEyes(1,1);
        ProcessEyes();
        delay(20);
            
        SetEyes(2,0);
        ProcessEyes();
        delay(20);
           
        SetEyes(3,1);
        ProcessEyes();
        delay(20);                   
      } //END for

      //eyes center
      SetEyes(2,2);
      ProcessEyes();
      delay(100);
    break;
        
    //---------------------------
    case RollEyesCcwID:     
      Serial.println("Processing ROLL EYES CCW...");
        for(i=1;i<=params[0];i++){
          SetEyes(1,1);
          ProcessEyes();
          delay(20);
            
          SetEyes(0,2);
          ProcessEyes();
          delay(20);
            
          SetEyes(1,3);
          ProcessEyes();
          delay(20);
            
          SetEyes(2,4 );
          ProcessEyes();
          delay(20);
            
          SetEyes(3,3);
          ProcessEyes();
          delay(20);
            
          SetEyes(4,2);
          ProcessEyes();
          delay(20);
            
          SetEyes(3,1);
          ProcessEyes();
          delay(20);
            
          SetEyes(2,0);
          ProcessEyes();
          delay(20);
            
          SetEyes(1,1);
          ProcessEyes();
          delay(20);         
          
        } //END for

        //eyes center
        SetEyes(2,2);
        ProcessEyes();
        delay(100);
    break;
    
    //---------------------------
    case DilateID:   // DILATE
    {
      Serial.println("Processing Dilation...");

      int targetP = params[0];

      // clamp pupil to desired range
      if (targetP < 1) targetP = 1;
      if (targetP > 12) targetP = 12;

      // compute matching iris radius
      // select appropriate dilation curve
      //int targetI = map(targetP, 1, 12, 8, 15);   //initial
      //int targetI = map(targetP, 1, 12, 6, 15);   //gentle
      //int targetI = map(targetP, 1, 12, 5, 15);   //medium
      int targetI = map(targetP, 1, 12, 5, 16);   //medium-1
      //int targetI = map(targetP, 1, 12, 4, 15);   //dramatic
      //int targetI = map(targetP, 1, 12, 4, 14);   //dramatic-1
      
      //Serial.print("targetP=");Serial.println(targetP);
      //Serial.print("targetI=");Serial.println(targetI);
      // animate both
      SmoothDilation(pupilRad, targetP, irisRad, targetI);
    }
    break;
    
    //---------------------------
    case DilateSpeedID:   // DILATE SPEED
    {
      dilateSpeed = params[0];
      if (dilateSpeed < 1)  dilateSpeed = 1;
      if (dilateSpeed > 40) dilateSpeed = 40;

      Serial.print("Dilation speed set to ");
      Serial.println(dilateSpeed);
    }
    break;  
    
    //---------------------------
    case DilateDelayID:   // DILATE DELAY
    {
      dilateDelay = params[0];
      if (dilateDelay < 0)  dilateDelay = 0;
      if (dilateDelay > 200) dilateDelay = 200;

      Serial.print("Dilation delay set to ");
      Serial.println(dilateDelay);
    }
    break;  

    //---------------------------      
    case BrowLID:
    {
      Serial.println("Processing BrowL...");
      float amount = params[0];   // 0.0–1.0
      if(amount<0)amount=0.0;
      MoveBrowL_Unit(amount);
      ProcessEyes();
    }
    break;

    //---------------------------      
    case BrowLTimeID:
    {
      Serial.println("Processing BrowLTime...");

      float y1 = params[0];   // normalized 0.0–1.0
      float y2 = params[1];
      int holdMs = (int)params[2];

      // Save original shape
      float orig_y1 = browL_y1;
      float orig_y2 = browL_y2;

      // Move to new shape
      SetBrowL_Tilt(y1, y2);

      // Hold
      delay(holdMs);

      // Return to original
      SmoothFloatMove(browL_y1, orig_y1, browSpeed);
      SmoothFloatMove(browL_y2, orig_y2, browSpeed);

      ProcessEyes();
    }
    break;
    
    //---------------------------
    case BrowLTiltID:   // BROW LEFT TILT
    {
      float y1 = params[0];   // 0.0–1.0
      if(y1<0)y1=0;
      float y2 = params[1];
      if(y2<0)y2=0;
      SetBrowL_Tilt(y1, y2);
    }
    break;  

    //---------------------------      
    case BrowRID:
    {
      Serial.println("Processing BrowR...");
      float amount = params[0];   // 0.0–1.0
      if(amount<0)amount=0.0;
      MoveBrowR_Unit(amount);
      ProcessEyes();
    }
    break;

    //---------------------------      
    case BrowRTimeID:
    {
      Serial.println("Processing BrowRTime...");

      float y1 = params[0];
      float y2 = params[1];
      int holdMs = (int)params[2];

      float orig_y1 = browR_y1;
      float orig_y2 = browR_y2;

      SetBrowR_Tilt(y1, y2);
      delay(holdMs);

      SmoothFloatMove(browR_y1, orig_y1, browSpeed);
      SmoothFloatMove(browR_y2, orig_y2, browSpeed);

      ProcessEyes();
    }
    break;
    
    //---------------------------
    case BrowRTiltID:   // BROW RIGHT TILT
    {
      float y1 = params[0];   // 0.0–1.0
      if(y1<0)y1=0;
      float y2 = params[1];
      if(y2<0)y2=0;
      SetBrowR_Tilt(y1, y2);
    }
    break;  

    //---------------------------      
    case BrowsID:   // BROWSS
    {
      float amount = params[0];   // 0.0–1.0
      if(amount<0)amount=0.0;
      MoveBrowL_Unit(amount);
      ProcessEyes();
    }
    break;  

    //---------------------------      
    case BrowsTimeID:
    {
      float target = params[0];
      int holdMs = (int)params[1];
      BrowTimed(leftBrow, target, holdMs);
      BrowTimed(rightBrow, target, holdMs);
    }
    break;
    
    //---------------------------
    case BrowSpeedID:   // BROW SPEED
    {
      browSpeed = params[0];
      if (browSpeed < 1) browSpeed = 1;
      if (browSpeed > 30) browSpeed = 30;
      Serial.print("Brow speed set to ");
      Serial.println(browSpeed);
    }
    break;  
    
    //---------------------------
    case ExprNormalID:    
  
      mouthState = NormalID;   // neutral    
    break;
    
    case ExprHappyID:
      ExprHappy();
      mouthState = SmileID;   // smile     
    break;
    
    case ExprSadID:
      ExprSad();
      mouthState = FrownlID;   // frown       
    break;
    
    case ExprAngryID:
      ExprAngry();
      mouthState = FrownlID;   // frown       
    break;
    
    case ExprSurpriseID:
      ExprSurprised();
      mouthState = OhID;   // O mouth 
    break;
    
    case ExprSkepticalID:
      ExprSkeptical();
      mouthState = SmirkLID;   // 6 smirk left 
    break;
    
    case ExprConfusedID:
      ExprConfused();
    break;
    
    case MouthNeutralID:
      mouthState = NormalID;
    break;
    
    case MouthSmileID:
      mouthState = SmileID;
    break;
    
    case MouthFrownID:
      mouthState = FrownlID;
    break;
    
    case MouthOID:
      mouthState = OhID;
    break;
    
    case MouthTalkID:
      Serial.println("Talk Start...");
      mouthState = TalkID;
      mouthTalkEnd = millis() + (int)params[0];
    break;
    
    case MouthOpenID:
      mouthState = OpenID;
    break;
    
    case MouthSmirkLID:
      mouthState = SmirkLID;
    break;
    
    case MouthSmirkRID:
      mouthState = SmirkRID;
    break;
    
    case MouthDiagLID:
      mouthState = DiagLID;
    break;
    
    case MouthDiagRID:
      mouthState = DiagRID;
    break;
    
    case MouthPeakID:
      mouthState = PeakID;
    break;
    
    case MouthShiftID:
      mouthShift = params[0];   // -1.0 to +1.0
      if (mouthShift < -1.0) mouthShift = -1.0;
      if (mouthShift >  1.0) mouthShift =  1.0;
    break;

    //---------------------------      
    case ExprTimedID:
    {
      int exprType = (int)params[0];
      int holdMs   = (int)params[1];

      switch (exprType) {
        case 1: ExprHappy();     break;
        case 2: ExprSad();       break;
        case 3: ExprAngry();     break;
        case 4: ExprSurprised(); break;
        case 5: ExprSkeptical(); break;
        case 6: ExprConfused();  break;
      }
      delay(holdMs);
      ExprNormal();
    }
    break;

    //---------------------------      
    case MouthTimeID:
    {
      int shape = (int)params[0];
      int holdMs = (int)params[1];

      int oldState = mouthState;
      mouthState = shape;
      ProcessEyes();
      delay(holdMs);

      mouthState = oldState;
      ProcessEyes();
    }
    break;
    
    //---------------------------
    case DelayID:   // DELAY
    {
      Serial.print("Delay=");
      Serial.println(params[0]);
      delay(params[0]);
    }
    break;  
    
    //---------------------------      


  } //END switch SwV
    
} //END ProcessComplex    

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF ProcessEyes FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//"ProcessEyes" uses parameters set in "ProcessComplex" to update the OLED display. 

void ProcessEyes() {

    display.clearDisplay();
    
    // -----------------------------------
    // Upeate mouth talk
    // -----------------------------------
    UpdateMouthTalk();
    UpdateTalkBrows();
    UpdateTalkBlink();
    UpdateTalkEyes();
    
    // -----------------------------------
    // ANALOG EYEBROWS (new system)
    // -----------------------------------
    DrawAnalogBrows();   // leftBrow & rightBrow (0.0 → 1.0)
    // -----------------------------------
    // BLINK HANDLING
    // -----------------------------------
    if (blink) {
        // closed eyes
        display.drawFastHLine(eyeXL - irisRad, eyeY, irisRad * 2, SSD1306_WHITE);
        display.drawFastHLine(eyeXR - irisRad, eyeY, irisRad * 2, SSD1306_WHITE);

        display.display();
        delay(1);
        return;
    }

    // -----------------------------------
    // OPEN EYES
    // -----------------------------------

    // 1. SCLERA (outer white, FIXED)
    if (WinkE == 0 || WinkE == 2)
        display.fillCircle(eyeXL, eyeY, eyeRad + 3, SSD1306_WHITE);
    if (WinkE == 0 || WinkE == 1)
        display.fillCircle(eyeXR, eyeY, eyeRad + 3, SSD1306_WHITE);

    // 2. IRIS (black, SCALES)
    if (!Xeye) {
        if (WinkE == 0 || WinkE == 2)
            display.fillCircle(lookXL, lookYL, irisRad, SSD1306_BLACK);
        if (WinkE == 0 || WinkE == 1)
            display.fillCircle(lookXR, lookYR, irisRad, SSD1306_BLACK);
    } else {
        if (WinkE == 0 || WinkE == 2)
            display.fillCircle(lookXL, lookYL, irisRad, SSD1306_BLACK);
        if (WinkE == 0 || WinkE == 1)
            display.fillCircle(lookXR, lookYR, irisRad, SSD1306_BLACK);
    }

    // 3. PUPIL (white, SCALES)
    if (!Xeye) {
        if (WinkE == 0 || WinkE == 2)
            display.fillCircle(lookXL, lookYL, pupilRad, SSD1306_WHITE);
        if (WinkE == 0 || WinkE == 1)
            display.fillCircle(lookXR, lookYR, pupilRad, SSD1306_WHITE);
    } else {
        if (WinkE == 0 || WinkE == 2)
            display.fillCircle(lookXL + 4, lookYL, pupilRad, SSD1306_WHITE);
        if (WinkE == 0 || WinkE == 1)
            display.fillCircle(lookXR + 4, lookYR, pupilRad, SSD1306_WHITE);
    }

        // 4. WINK LINES
    if (WinkE == 1)
      display.drawFastHLine(eyeXL - irisRad, eyeY, irisRad * 2, SSD1306_WHITE);
    else if (WinkE == 2)
      display.drawFastHLine(eyeXR - irisRad, eyeY, irisRad * 2, SSD1306_WHITE);

    // -----------------------------------
    // NOSE
    // -----------------------------------
    //display.fillTriangle(54, 61, 62, 61, 58, 46, SSD1306_WHITE);
    display.fillTriangle(noseX-NoseXD,noseY, noseX+NoseXD,noseY, noseX, noseY-NoseYD, SSD1306_WHITE);

    // -----------------------------------
    // Mouth
    // -----------------------------------
    DrawMouth();

    // PUSH BUFFER TO SCREEN
    display.display();
    
} //END ProcessEyes

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF  UpdateTalkBrows FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void UpdateTalkBrows() {
    if (mouthState == TalkID) {

        // occasional tiny brow lift
        if (random(0, 25) == 0) {   // ~4% chance per frame
            float bump = random(1, 4) * 0.01;  // 0.01–0.03
            MoveBrows_Unit(bump);
        }

        // occasional tiny brow drop
        if (random(0, 25) == 0) {
            float bump = random(1, 4) * 0.01;
            MoveBrows_Unit(-bump);
        }
    }
} //END UpdateTalkBrows

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF  UpdateTalkBlink FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void UpdateTalkBlink() {
    if (mouthState == TalkID) {
        if (random(0, 60) == 0) {   // ~1 blink every 1–2 seconds
            blink = 1;
            ProcessEyes();
            delay(80);
            blink = 0;
        }
    }
} //END UpdateTalkBlink

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF  UpdateTalkEyes FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF

void UpdateTalkEyes() {
    if (mouthState != TalkID) return;

    // occasional micro-dart
    if (random(0, 40) == 0) {

        int dx = 0;
        int dy = 0;

        // how far from center?
        int offsetX = lookXL - eyeXL;   // negative = left, positive = right
        int offsetY = lookYL - eyeY;    // negative = up,   positive = down

        // horizontal bias
        if (offsetX > 3)       dx = random(-3, -1);   // too far right → push left
        else if (offsetX < -3) dx = random(1, 3);     // too far left  → push right
        else                   dx = random(-2, 3);    // near center → full range

        // vertical bias
        if (offsetY > 3)       dy = random(-3, -1);   // too far down → push up
        else if (offsetY < -2) dy = random(1, 4);     // too far up   → push down
//        else if (offsetY < -3) dy = random(1, 3);     // too far up   → push down
        else                   dy = random(-2, 3);    // near center → full range

        // apply movement
        lookXL += dx;
        lookXR += dx;
        lookYL += dy;
        lookYR += dy;

        // symmetric clamp based on geometry
        int moveRad = eyeRad - irisRad;   // = 8

        // left eye
        lookXL = constrain(lookXL, eyeXL - moveRad, eyeXL + moveRad);
        lookYL = constrain(lookYL, eyeY  - moveRad, eyeY  + moveRad);

        // right eye
        lookXR = constrain(lookXR, eyeXR - moveRad, eyeXR + moveRad);
        lookYR = constrain(lookYR, eyeY  - moveRad, eyeY  + moveRad);
    }
    
    if (random(0, 200) == 0) {   // rare reset
      lookXL = eyeXL;
      lookXR = eyeXR;
      lookYL = eyeY;
      lookYR = eyeY;
    }
    
    // gentle return-to-center drift (very natural)
    if (random(0, 50) == 0) {
        lookXL += (eyeXL - lookXL) / 6;
        lookXR += (eyeXR - lookXR) / 6;
        lookYL += (eyeY  - lookYL) / 6;
        lookYR += (eyeY  - lookYR) / 6;
    }
} //END UpdateTalkEyes

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF  FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFF

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF  FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFF

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF  FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFF

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF  FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFF

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF SetEyes FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFF
//"SetEyes" sets up eye positions based upon passed X & Y values. 
//   Eyes have only 4 positions for X, and 4 positions for Y.

void  SetEyes(int EyePosX,int EyePosY){  

  if(EyePosX <=4 && EyePosY <=4){ //check for bad data
    lookXL = eyeXL;
    lookYL = eyeY;
    lookXR = eyeXR;
    lookYR = eyeY;

    if (INVERT_X){
      offsetXmx = -offsetXmx; 
      offsetXmn = -offsetXmn; 
    }
    if (INVERT_Y){
      offsetYmx = -offsetYmx; 
      offsetYmn = -offsetYmn; 
    }

    xPosition=EyePosX;
    yPosition=EyePosY;

    //setup Y
    if (yPosition==0){
      lookYL = eyeY - offsetYmx;    
      lookYR = eyeY - offsetYmx;    
    }
    else if (yPosition==1){
      lookYL = eyeY - offsetYmn;    
      lookYR = eyeY - offsetYmn;    
    }
    else if (yPosition==2){
      lookYL = eyeY;    
      lookYR = eyeY;    
    }
    else if (yPosition==3){
      lookYL = eyeY + offsetYmn;    
      lookYR = eyeY + offsetYmn;    
    }
    else if (yPosition==4){
      lookYL = eyeY + offsetYmx; 
      lookYR = eyeY + offsetYmx; 
    }

    //setup X
    if (xPosition==0){
      lookXL = eyeXL - offsetXmx;    
      lookXR = eyeXR - offsetXmx;    
    }
    else if (xPosition==1){
      lookXL = eyeXL - offsetXmn;    
      lookXR = eyeXR - offsetXmn;    
    }
    else if (xPosition==2){
      lookXL = eyeXL;    
      lookXR = eyeXR;    
    }
    else if (xPosition==3){
      lookXL = eyeXL + offsetXmn;    
      lookXR = eyeXR + offsetXmn;    
    }
    else if (xPosition==4){
      lookXL = eyeXL + offsetXmx;    
      lookXR = eyeXR + offsetXmx;    
    }
    
  } //Exit if bad data  
} //END SetEyes

//************************************************************************

//FFFFFFFFFFFFFFFFFFFFFFFFFFFF
//FFFFF WinkEye FUNCTION FFFFF
//FFFFFFFFFFFFFFFFFFFFFFFFFFFF

void WinkEye(int WhichEye){  //0=None, 1=Left, 2=Right

  if(WhichEye <=2){
    WinkE=WhichEye;    
    ProcessEyes();  
  } //EXIT if bad data
} //END WinkEye

//************************************************************************
