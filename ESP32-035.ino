#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <TFT_eSPI.h>
#include <SD.h>

#define ADS_GAIN GAIN_ONE
#define ADS_DATARATE RATE_ADS1115_128SPS

Adafruit_ADS1115 ads;
TFT_eSPI tft = TFT_eSPI();

long raw = 0;
double resolution = 0;
float vcc = 0, current = 0;
float power = 0, resistance = 0;

float lastVcc = -999, lastCurrent = -999;
float lastPower = -999, lastResistance = -999;
float lastmAh = -999, lastWh = -999;
float threshold = 0.001;

unsigned long lastTime = 0;
float mAh = 0.0;
float Wh = 0.0;

float currMaxV = 30.0f;
float currMaxI = 10.0f;

#define GRAPH_X 0
#define GRAPH_Y 0
#define GRAPH_WIDTH 380
#define GRAPH_HEIGHT 250
#define MAX_POINTS 30

float voltageData[MAX_POINTS];
float currentData[MAX_POINTS];
int dataIndex = 0;
unsigned long lastGraphUpdate = 0;
const int graphUpdateInterval = 50;

TFT_eSprite graphSprite(&tft);

// Biến cho min/max scale
float minVoltage = 0.0f;
float maxVoltage = 30.0f;
float minCurrent = 0.0f;
float maxCurrent = 10.0f;

// Biến cho cảm ứng và màn hình
int currentScreen = 0;
const int totalScreens = 3;
unsigned long lastButtonPress = 0;
const unsigned long buttonDebounce = 200;
bool wasTouchedLast = false;

#define SPACE_GRAY 0x4A49
#define SPACE_GRAY_LIGHT 0x6B4D
#define SPACE_GRAY_DARK 0x2945
#define ACCENT_COLOR 0x07FF

// Định nghĩa bitmap
static const unsigned char PROGMEM image_bluetooth_bits[] = {
    0x01,0x00,0x02,0x80,0x02,0x40,0x22,0x20,0x12,0x20,0x0a,0x40,0x06,0x80,
    0x03,0x00,0x06,0x80,0x0a,0x40,0x12,0x20,0x22,0x20,0x02,0x40,0x02,0x80,0x01,0x00,0x00,0x00
};

static const unsigned char PROGMEM image_arrow_left_bits[] = {0x00,0xc0,0x00,0xc0,0x00,0x30,0x00,0x30,0xff,0xfc,0xff,0xfc,0x00,0x30,0x00,0x30,0x00,0xc0,0x00,0xc0};

static const unsigned char PROGMEM image_Power_hvr_bits[] = {
    0x1f,0xff,0xfc,0x00,0x3f,0xff,0xfe,0x00,0x7f,0xff,0xff,0x00,0xff,0xff,0xff,0x80,
    0xff,0xf7,0xff,0x80,0xff,0xe3,0xff,0x80,0xff,0xe3,0xff,0x80,0xfe,0x63,0x3f,0x80,
    0xfc,0x63,0x1f,0x80,0xf8,0xe3,0x8f,0x80,0xf9,0xe3,0xcf,0x80,0xf9,0xe3,0xcf,0x80,
    0xf9,0xf7,0xcf,0x80,0xf9,0xff,0xcf,0x80,0xf9,0xff,0xcf,0x80,0xf9,0xff,0xcf,0x80,
    0xf8,0xff,0x8f,0x80,0xfc,0x7f,0x1f,0x80,0xfe,0x3e,0x3f,0x80,0xff,0x00,0x7f,0x80,
    0xff,0xc1,0xff,0x80,0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,
    0x7f,0xff,0xff,0x00,0x3f,0xff,0xfe,0x00,0x1f,0xff,0xfc,0x00
};

static const unsigned char PROGMEM image_Up_hvr_1_bits[] = {
    0x1f,0xff,0xfc,0x00,0x3f,0xff,0xfe,0x00,0x7f,0xff,0xff,0x00,0xff,0xff,0xff,0x80,
    0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,
    0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,0xf0,0x00,0x07,0x80,0xf0,0x00,0x07,0x80,
    0xf8,0x00,0x0f,0x80,0xfc,0x00,0x1f,0x80,0xfe,0x00,0x3f,0x80,0xff,0x00,0x7f,0x80,
    0xff,0x80,0xff,0x80,0xff,0xc1,0xff,0x80,0xff,0xe3,0xff,0x80,0xff,0xf7,0xff,0x80,
    0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,
    0x7f,0xff,0xff,0x00,0x3f,0xff,0xfe,0x00,0x1f,0xff,0xfc,0x00
};

static const unsigned char PROGMEM image_Up_hvr_2_bits[] = {
    0x1f,0xff,0xff,0x00,0x3f,0xff,0xff,0x80,0x7f,0xff,0xff,0xc0,0xff,0xff,0xff,0xe0,
    0xff,0xcf,0xff,0xe0,0xff,0xc7,0xff,0xe0,0xff,0xc3,0xff,0xe0,0xff,0xc1,0xff,0xe0,
    0xff,0xc0,0x7f,0xe0,0xff,0xc0,0x3f,0xe0,0xff,0xc0,0x1f,0xe0,0xff,0xc0,0x0f,0xe0,
    0xff,0xc0,0x1f,0xe0,0xff,0xc0,0x3f,0xe0,0xff,0xc0,0x7f,0xe0,0xff,0xc0,0xff,0xe0,
    0xff,0xc1,0xff,0xe0,0xff,0xc3,0xff,0xe0,0xff,0xc7,0xff,0xe0,0xff,0xcf,0xff,0xe0,
    0xff,0xff,0xff,0xe0,0x7f,0xff,0xff,0xc0,0x3f,0xff,0xff,0x80,0x1f,0xff,0xff,0x00
};

static const unsigned char PROGMEM image_Up_hvr_3_bits[] = {
    0x1f,0xff,0xfc,0x00,0x3f,0xff,0xfe,0x00,0x7f,0xff,0xff,0x00,0xff,0xff,0xff,0x80,
    0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,0xff,0xf7,0xff,0x80,
    0xff,0xe3,0xff,0x80,0xff,0xc1,0xff,0x80,0xff,0x80,0xff,0x80,0xff,0x00,0x7f,0x80,
    0xfe,0x00,0x3f,0x80,0xfc,0x00,0x1f,0x80,0xf8,0x00,0x0f,0x80,0xf0,0x00,0x07,0x80,
    0xf0,0x00,0x07,0x80,0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,
    0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,0xff,0xff,0xff,0x80,
    0x7f,0xff,0xff,0x00,0x3f,0xff,0xfe,0x00,0x1f,0xff,0xfc,0x00
};

static const unsigned char PROGMEM image_Up_hvr_bits[] = {
    0x1f,0xff,0xff,0x00,0x3f,0xff,0xff,0x80,0x7f,0xff,0xff,0xc0,0xff,0xff,0xff,0xe0,
    0xff,0xfe,0x7f,0xe0,0xff,0xfc,0x7f,0xe0,0xff,0xf8,0x7f,0xe0,0xff,0xf0,0x7f,0xe0,
    0xff,0xe0,0x7f,0xe0,0xff,0xc0,0x7f,0xe0,0xff,0x80,0x7f,0xe0,0xff,0x00,0x7f,0xe0,
    0xfe,0x00,0x7f,0xe0,0xff,0x00,0x7f,0xe0,0xff,0x80,0x7f,0xe0,0xff,0xc0,0x7f,0xe0,
    0xff,0xe0,0x7f,0xe0,0xff,0xf0,0x7f,0xe0,0xff,0xf8,0x7f,0xe0,0xff,0xfc,0x7f,0xe0,
    0xff,0xfe,0x7f,0xe0,0xff,0xff,0xff,0xe0,0x7f,0xff,0xff,0xc0,0x3f,0xff,0xff,0x80,
    0x1f,0xff,0xff,0x00
};

struct Button {
    int x, y, w, h;
    String label;
    uint16_t color;
    uint16_t textColor;
    uint16_t borderColor;
    const unsigned char* bitmap;
};

// Định nghĩa 5 nút mới
Button btnPower   = {413, 258, 25, 27, "PWR", SPACE_GRAY, TFT_WHITE, SPACE_GRAY_LIGHT, image_Power_hvr_bits};
Button btnUp      = {413, 236, 25, 27, "UP", SPACE_GRAY, TFT_WHITE, SPACE_GRAY_LIGHT, image_Up_hvr_3_bits};
Button btnDown    = {413, 280, 25, 27, "DN", SPACE_GRAY, TFT_WHITE, SPACE_GRAY_LIGHT, image_Up_hvr_1_bits};
Button btnVolUp   = {435, 259, 27, 25, "V+", SPACE_GRAY, TFT_WHITE, SPACE_GRAY_LIGHT, image_Up_hvr_2_bits};
Button btnVolDown = {389, 259, 27, 25, "V-", SPACE_GRAY, TFT_WHITE, SPACE_GRAY_LIGHT, image_Up_hvr_bits};

// Hàm phụ để vẽ hiển thị điện áp "IN:"
void drawVoltageDisplay() {
    tft.fillRoundRect(380, 19, 95, 26, 5, 0xFC00);
    tft.setTextFont(2);
    tft.setTextColor(0x0);
    tft.drawString("IN:", 385, 25);
    tft.setTextColor(0xFFFF);
    char voltStr[10];
    sprintf(voltStr, "%.2f", vcc);
    tft.drawString(voltStr, 422, 25);
}

void drawScreen3() {
    tft.drawLine(37, 35, 349, 35, 0x4208);
    tft.drawLine(37, 59, 349, 59, 0x4208);
    tft.drawLine(37, 84, 349, 84, 0x4208);
    tft.drawLine(37, 110, 349, 110, 0x4208);
    tft.drawLine(37, 135, 349, 135, 0x4208);
    tft.drawLine(37, 160, 349, 160, 0x4208);
    tft.drawLine(37, 185, 349, 185, 0x4208);
    tft.drawLine(91, 15, 91, 209, 0x4208);
    tft.drawLine(146, 15, 146, 209, 0x4208);
    tft.drawLine(200, 15, 200, 209, 0x4208);
    tft.drawLine(311, 15, 311, 209, 0x4208);
    tft.drawLine(256, 15, 256, 209, 0x4208);

    tft.setTextFont(2);
    tft.setTextColor(0x24BE);
    tft.drawString("A", 24, 215);
    tft.drawString("0", 26, 203);
    tft.drawString("1", 26, 178);
    tft.drawString("2", 25, 154);
    tft.drawString("3", 25, 129);
    tft.drawString("4", 25, 104);

    tft.setTextColor(0xF206);
    tft.drawString("V", 4, 215);
    tft.drawString("0", 9, 203);
    tft.drawString("1", 9, 179);
    tft.drawString("2", 9, 154);
    tft.drawString("3", 9, 129);
    tft.drawString("4", 9, 104);
    tft.drawString("5", 9, 78);
    tft.drawString("6", 9, 53);
    tft.drawString("7", 9, 29);
    
    tft.setTextColor(0x24BE);
    tft.drawLine(35, 209, 35, 10, 0xFC00);
    tft.drawLine(366, 210, 35, 210, 0xFC00);
    tft.drawLine(369, 0, 369, 229, 0x63F1);
    tft.drawLine(0, 230, 369, 230, 0x63F1);

    tft.setTextColor(0xFC00);
    tft.drawString("^", 32, 9);
    tft.drawString(">", 363, 202);
    

    tft.setTextColor(0xFE00);
    tft.drawString("TIME", 68, 215);
    tft.drawBitmap(99, 217, image_arrow_left_bits, 14, 10, 0xFE00);
}

void drawButtons() {
    // Vẽ nút Up
    tft.drawBitmap(btnUp.x, btnUp.y, btnUp.bitmap, btnUp.w, btnUp.h, 0xFC00);
    // Vẽ nút Power
    tft.drawBitmap(btnPower.x, btnPower.y, btnPower.bitmap, btnPower.w, btnPower.h, 0xFC00);
    // Vẽ nút Down
    tft.drawBitmap(btnDown.x, btnDown.y, btnDown.bitmap, btnDown.w, btnDown.h, 0xFC00);
    // Vẽ nút Vol Up
    tft.drawBitmap(btnVolUp.x, btnVolUp.y, btnVolUp.bitmap, btnVolUp.w, btnVolUp.h, 0xFC00);
    // Vẽ nút Vol Down
    tft.drawBitmap(btnVolDown.x, btnVolDown.y, btnVolDown.bitmap, btnVolDown.w, btnVolDown.h, 0xFC00);
    // Vẽ hình elip bao quanh các nút
    tft.drawEllipse(425, 271, 40, 40, 0xFC00);
}

int checkButtonPress(uint16_t x, uint16_t y) {
    // Dịch vùng cảm ứng sang phải 13 pixel để khớp với phần cứng
    if (x >= btnUp.x + 13 && x < (btnUp.x + 13 + btnUp.w) && 
        y >= btnUp.y && y < (btnUp.y + btnUp.h)) return 1; // Up
    if (x >= btnVolUp.x + 13 && x < (btnVolUp.x + 13 + btnVolUp.w) && 
        y >= btnVolUp.y && y < (btnVolUp.y + btnVolUp.h)) return 2; // Vol Up
    if (x >= btnPower.x + 13 && x < (btnPower.x + 13 + btnPower.w) && 
        y >= btnPower.y && y < (btnPower.y + btnPower.h)) return 3; // Power
    if (x >= btnDown.x + 13 && x < (btnDown.x + 13 + btnDown.w) && 
        y >= btnDown.y && y < (btnDown.y + btnDown.h)) return 4; // Down
    if (x >= btnVolDown.x + 13 && x < (btnVolDown.x + 13 + btnVolDown.w) && 
        y >= btnVolDown.y && y < (btnVolDown.y + btnVolDown.h)) return 5; // Vol Down
    return 0;
}

void handleButtonPress(int button) {
    switch(button) {
        case 1: // Up: Tăng currMaxV
            currMaxV += 0.5f;
            if (currMaxV > 50.0f) currMaxV = 50.0f; // Giới hạn tối đa
            Serial.print("currMaxV: "); Serial.println(currMaxV);
            lastButtonPress = millis();
            break;
        case 2: // Vol Up: Chuyển màn hình trước
            currentScreen--;
            if (currentScreen < 0) currentScreen = totalScreens - 1;
            switchScreen(currentScreen);
            lastButtonPress = millis();
            break;
        case 3: // Power: Reset (xử lý nhấn lâu trong handleTouch)
            break;
        case 4: // Down: Giảm currMaxV
            currMaxV -= 0.5f;
            if (currMaxV < 0.0f) currMaxV = 0.0f; // Giới hạn tối thiểu
            Serial.print("currMaxV: "); Serial.println(currMaxV);
            lastButtonPress = millis();
            break;
        case 5: // Vol Down: Chuyển màn hình sau
            currentScreen++;
            if (currentScreen >= totalScreens) currentScreen = 0;
            switchScreen(currentScreen);
            lastButtonPress = millis();
            break;
    }
}

void switchScreen(int screen) {
    currentScreen = screen;
    tft.fillRect(0, 0, 480, 320, TFT_BLACK);
    lastVcc = -999;
    lastCurrent = -999;
    lastPower = -999;
    lastResistance = -999;
    lastmAh = -999;
    lastWh = -999;
    if (currentScreen == 1) {
        calculateDataRange();
        drawGraphBackground();
        drawVoltageCurve(voltageData, dataIndex);
        drawCurrentCurve(currentData, dataIndex);
        graphSprite.pushSprite(GRAPH_X, GRAPH_Y);
    } else if (currentScreen == 2) {
        drawScreen3();
    }
    drawButtons();
}

int16_t ads_read_voltage() {
    long sumV = 0;
    for (int i = 0; i < 2; i++) sumV += ads.readADC_SingleEnded(0);
    return sumV / 2;
}

int16_t ads_read_current() {
    long sumA = 0;
    for (int i = 0; i < 2; i++) sumA += ads.readADC_SingleEnded(3);
    return sumA / 2;
}

void updateMeasurements() {
    raw = ads_read_voltage();
    resolution = 4.096 / 32768.0;
    float A0 = raw * resolution;
    vcc = (A0 * 11.0) - 0.015;
    if (vcc <= 0.005) vcc = 0;
    long rawA = ads_read_current();
    float A3 = rawA * resolution;
    current = (A3 - 0.1225) / (0.005 * 35.3);
    if (current <= 0.005) current = 0;
    current = constrain(current, 0.0, 10.0);
    power = vcc * current;
    resistance = (current > 0.005) ? vcc / current : 0;
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    mAh += current * dt / 3.6;
    Wh += vcc * current * dt / 3600.0;
}

void calculateDataRange() {
    minVoltage = voltageData[0];
    maxVoltage = voltageData[0];
    minCurrent = currentData[0];
    maxCurrent = currentData[0];
    for (int i = 0; i < MAX_POINTS; i++) {
        if (voltageData[i] < minVoltage) minVoltage = voltageData[i];
        if (voltageData[i] > maxVoltage) maxVoltage = voltageData[i];
        if (currentData[i] < minCurrent) minCurrent = currentData[i];
        if (currentData[i] > maxCurrent) maxCurrent = currentData[i];
    }
    float vRange = maxVoltage - minVoltage;
    if (vRange < 0.5f) vRange = 0.5f;
    minVoltage -= vRange * 0.15f;
    maxVoltage += vRange * 0.15f;
    if (minVoltage < 0) minVoltage = 0;
    float iRange = maxCurrent - minCurrent;
    if (iRange < 0.1f) iRange = 0.1f;
    minCurrent -= iRange * 0.15f;
    maxCurrent += iRange * 0.15f;
    if (minCurrent < 0) minCurrent = 0;
}

void drawGraphBackground() {
    graphSprite.fillSprite(TFT_BLACK);
    const int MARGIN_TOP = 40;
    const int MARGIN_BOTTOM = 40;
    const int MARGIN_LEFT = 50;
    const int MARGIN_RIGHT = 50;
    const int chartWidth = GRAPH_WIDTH - MARGIN_LEFT - MARGIN_RIGHT;
    const int chartHeight = GRAPH_HEIGHT - MARGIN_TOP - MARGIN_BOTTOM;
    graphSprite.drawRect(MARGIN_LEFT, MARGIN_TOP, chartWidth, chartHeight, 0x4208);
    const int gridRows = 5;
    const int gridCols = 14;
    for (int i = 1; i < gridRows; i++) {
        int y = MARGIN_TOP + (chartHeight * i / gridRows);
        for (int x = MARGIN_LEFT; x < MARGIN_LEFT + chartWidth; x += 6) {
            graphSprite.drawPixel(x, y, 0x2104);
        }
    }
    for (int i = 1; i < gridCols; i++) {
        int x = MARGIN_LEFT + (chartWidth * i / gridCols);
        for (int y = MARGIN_TOP; y < MARGIN_TOP + chartHeight; y += 6) {
            graphSprite.drawPixel(x, y, 0x2104);
        }
    }
    graphSprite.setTextFont(2);
    graphSprite.setTextColor(TFT_RED, TFT_BLACK);
    for (int i = 0; i <= 5; i++) {
        int y = MARGIN_TOP + (chartHeight * i / 5);
        float vValue = maxVoltage - (maxVoltage - minVoltage) * i / 5;
        graphSprite.drawFastHLine(MARGIN_LEFT - 5, y, 5, TFT_RED);
        char label[8];
        sprintf(label, "%.2f", vValue);
        graphSprite.drawString(label, 5, y - 8);
    }
    graphSprite.setTextColor(TFT_WHITE, TFT_RED);
    graphSprite.drawString(" V ", 8, 10);
    graphSprite.setTextColor(TFT_CYAN, TFT_BLACK);
    for (int i = 0; i <= 5; i++) {
        int y = MARGIN_TOP + (chartHeight * i / 5);
        float iValue = maxCurrent - (maxCurrent - minCurrent) * i / 5;
        graphSprite.drawFastHLine(MARGIN_LEFT + chartWidth, y, 5, TFT_CYAN);
        char label[8];
        sprintf(label, "%.2f", iValue);
        graphSprite.drawString(label, MARGIN_LEFT + chartWidth + 8, y - 8);
    }
    graphSprite.setTextColor(TFT_WHITE, TFT_CYAN);
    graphSprite.drawString(" A ", GRAPH_WIDTH - 35, 10);
    graphSprite.setTextColor(0x8410, TFT_BLACK);
    graphSprite.drawString("Time ->", MARGIN_LEFT + 5, GRAPH_HEIGHT - 25);
}

void drawVoltageCurve(float* data, int index) {
    const int MARGIN_TOP = 40;
    const int MARGIN_BOTTOM = 40;
    const int MARGIN_LEFT = 50;
    const int MARGIN_RIGHT = 50;
    const int chartWidth = GRAPH_WIDTH - MARGIN_LEFT - MARGIN_RIGHT;
    const int chartHeight = GRAPH_HEIGHT - MARGIN_TOP - MARGIN_BOTTOM;
    float step = float(chartWidth) / (MAX_POINTS - 1);
    float vRange = maxVoltage - minVoltage;
    if (vRange < 0.01f) vRange = 1.0f;
    for (int i = 1; i < MAX_POINTS; i++) {
        int p = (index - i + 1 + MAX_POINTS) % MAX_POINTS;
        int c = (index - i + MAX_POINTS) % MAX_POINTS;
        int x0 = MARGIN_LEFT + roundf((i - 1) * step);
        int x1 = MARGIN_LEFT + roundf(i * step);
        int y0 = MARGIN_TOP + chartHeight - roundf((data[p] - minVoltage) / vRange * chartHeight);
        int y1 = MARGIN_TOP + chartHeight - roundf((data[c] - minVoltage) / vRange * chartHeight);
        y0 = constrain(y0, MARGIN_TOP, MARGIN_TOP + chartHeight);
        y1 = constrain(y1, MARGIN_TOP, MARGIN_TOP + chartHeight);
        graphSprite.drawLine(x0, y0, x1, y1, TFT_RED);
        graphSprite.drawLine(x0, y0 + 1, x1, y1 + 1, TFT_RED);
    }
}

void drawCurrentCurve(float* data, int index) {
    const int MARGIN_TOP = 40;
    const int MARGIN_BOTTOM = 40;
    const int MARGIN_LEFT = 50;
    const int MARGIN_RIGHT = 50;
    const int chartWidth = GRAPH_WIDTH - MARGIN_LEFT - MARGIN_RIGHT;
    const int chartHeight = GRAPH_HEIGHT - MARGIN_TOP - MARGIN_BOTTOM;
    float step = float(chartWidth) / (MAX_POINTS - 1);
    float iRange = maxCurrent - minCurrent;
    if (iRange < 0.01f) iRange = 1.0f;
    for (int i = 1; i < MAX_POINTS; i++) {
        int p = (index - i + 1 + MAX_POINTS) % MAX_POINTS;
        int c = (index - i + MAX_POINTS) % MAX_POINTS;
        int x0 = MARGIN_LEFT + roundf((i - 1) * step);
        int x1 = MARGIN_LEFT + roundf(i * step);
        int y0 = MARGIN_TOP + chartHeight - roundf((data[p] - minCurrent) / iRange * chartHeight);
        int y1 = MARGIN_TOP + chartHeight - roundf((data[c] - minCurrent) / iRange * chartHeight);
        y0 = constrain(y0, MARGIN_TOP, MARGIN_TOP + chartHeight);
        y1 = constrain(y1, MARGIN_TOP, MARGIN_TOP + chartHeight);
        graphSprite.drawLine(x0, y0, x1, y1, TFT_CYAN);
    }
}

void handleTouch() {
    uint16_t x, y;
    bool isTouched = tft.getTouch(&x, &y);
    static bool centerButtonPressed = false;
    static unsigned long centerButtonPressTime = 0;
    const unsigned long longPressDuration = 1000;

    if (isTouched && !wasTouchedLast) {
        Serial.print("Touch at x: "); Serial.print(x);
        Serial.print(", y: "); Serial.println(y);
        if (millis() - lastButtonPress > buttonDebounce) {
            int button = checkButtonPress(x, y);
            if (button > 0) {
                if (button == 3) { // Power button
                    centerButtonPressed = true;
                    centerButtonPressTime = millis();
                } else {
                    handleButtonPress(button);
                }
            }
        }
    } else if (isTouched && centerButtonPressed && (millis() - centerButtonPressTime >= longPressDuration)) {
        mAh = 0.0;
        Wh = 0.0;
        lastmAh = -999;
        lastWh = -999;
        tft.fillRect(100, 180, 280, 50, TFT_BLACK);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("RESET SUCCESS!", 240, 200, 4);
        tft.setTextDatum(TL_DATUM);
        delay(1000);
        tft.fillRect(100, 180, 280, 50, TFT_BLACK);
        centerButtonPressed = false;
        lastButtonPress = millis();
    } else if (!isTouched) {
        centerButtonPressed = false;
    }
    wasTouchedLast = isTouched;
}

void setup() {
    Serial.begin(115200);
    SPI.begin(18, 19, 23);
    SD.begin(5);
    tft.init();
    tft.setRotation(1);
    tft.setSwapBytes(true);
    tft.fillScreen(TFT_BLACK);
    ads.begin();
    ads.setGain(ADS_GAIN);
    ads.setDataRate(ADS_DATARATE);
    lastTime = millis();
    lastGraphUpdate = millis();
    for (int i = 0; i < MAX_POINTS; i++) {
        voltageData[i] = 0;
        currentData[i] = 0;
    }
    graphSprite.setColorDepth(8);
    graphSprite.createSprite(GRAPH_WIDTH, GRAPH_HEIGHT);
    currentScreen = 0;
    drawButtons();
}

void loop() {
    handleTouch();
    updateMeasurements();

    if (currentScreen == 0) {
        if (fabs(vcc - lastVcc) > threshold) {
            tft.drawRoundRect(10, 10, 180, 80, 10, 0x4208);
            tft.setTextFont(2);
            tft.setTextColor(0xFD20, TFT_BLACK);
            tft.drawString("VOLTAGE", 20, 18);
            tft.setTextFont(7);
            tft.setTextColor(TFT_RED, TFT_BLACK);
            char voltStr[10];
            sprintf(voltStr, "%.2f", vcc);
            tft.drawString(voltStr, 20, 35);
            tft.setTextFont(4);
            tft.drawString("V", 140, 45);
            lastVcc = vcc;
        }
        if (fabs(current - lastCurrent) > threshold) {
            tft.drawRoundRect(195, 10, 180, 80, 10, 0x4208);
            tft.setTextFont(2);
            tft.setTextColor(0x04FF, TFT_BLACK);
            tft.drawString("CURRENT", 205, 18);
            tft.setTextFont(7);
            tft.setTextColor(TFT_CYAN, TFT_BLACK);
            char currStr[10];
            sprintf(currStr, "%.2f", current);
            tft.drawString(currStr, 205, 35);
            tft.setTextFont(4);
            tft.drawString("A", 325, 45);
            lastCurrent = current;
        }
        if (fabs(power - lastPower) > threshold) {
            tft.drawRoundRect(10, 95, 180, 73, 8, 0x4208);
            tft.setTextFont(2);
            tft.setTextColor(0x07E0, TFT_BLACK);
            tft.drawString("POWER", 20, 100);
            tft.setTextFont(6);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            char powStr[10];
            sprintf(powStr, "%.1f", power);
            tft.drawString(powStr, 20, 117);
            tft.setTextFont(2);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.drawString("W", 140, 146);
            lastPower = power;
        }
        if (fabs(resistance - lastResistance) > threshold) {
            tft.drawRoundRect(195, 95, 180, 73, 8, 0x4208);
            tft.setTextFont(2);
            tft.setTextColor(0xFFE0, TFT_BLACK);
            tft.drawString("RESISTANCE", 205, 100);
            tft.setTextFont(6);
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            char resStr[10];
            if (resistance < 100) {
                sprintf(resStr, "%.2f", resistance);
            } else if (resistance < 1000) {
                sprintf(resStr, "%.1f", resistance);
            } else {
                sprintf(resStr, "%.0f", resistance);
            }
            tft.drawString(resStr, 205, 117);
            tft.setTextFont(2);
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.drawString("Ohm", 305, 146);
            lastResistance = resistance;
        }
        if (fabs(Wh - lastWh) > threshold) {
            tft.drawRoundRect(10, 173, 180, 73, 8, 0x4208);
            tft.setTextFont(2);
            tft.setTextColor(0xF81F, TFT_BLACK);
            tft.drawString("ENERGY", 20, 178);
            tft.setTextFont(6);
            tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
            char whStr[15];
            sprintf(whStr, "%.2f", Wh);
            tft.drawString(whStr, 20, 195);
            tft.setTextFont(2);
            tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
            tft.drawString("Wh", 140, 225);
            lastWh = Wh;
        }
        if (fabs(mAh - lastmAh) > threshold) {
            tft.drawRoundRect(195, 173, 180, 73, 8, 0x4208);
            tft.setTextFont(2);
            tft.setTextColor(0xFBE0, TFT_BLACK);
            tft.drawString("CAPACITY", 205, 178);
            tft.setTextFont(6);
            tft.setTextColor(TFT_ORANGE, TFT_BLACK);
            char mahStr[10];
            if (mAh < 100) {
                sprintf(mahStr, "%.1f", mAh);
            } else {
                sprintf(mahStr, "%.0f", mAh);
            }
            tft.drawString(mahStr, 205, 195);
            tft.setTextFont(2);
            tft.setTextColor(TFT_ORANGE, TFT_BLACK);
            tft.drawString("mAh", 305, 225);
            lastmAh = mAh;
        }
    } else if (currentScreen == 1) {
        unsigned long now = millis();
        if (now - lastGraphUpdate >= graphUpdateInterval) {
            int ni = (dataIndex + 1) % MAX_POINTS;
            voltageData[ni] = vcc;
            currentData[ni] = current;
            dataIndex = ni;
            calculateDataRange();
            drawGraphBackground();
            drawVoltageCurve(voltageData, dataIndex);
            drawCurrentCurve(currentData, dataIndex);
            graphSprite.fillRect(GRAPH_WIDTH / 2 - 80, 5, 160, 25, 0x2104);
            graphSprite.drawRect(GRAPH_WIDTH / 2 - 80, 5, 160, 25, 0x4208);
            graphSprite.setTextFont(2);
            graphSprite.setTextColor(TFT_RED, 0x2104);
            graphSprite.drawFloat(vcc, 2, GRAPH_WIDTH / 2 - 70, 12);
            graphSprite.setTextColor(TFT_WHITE, 0x2104);
            graphSprite.drawString("V", GRAPH_WIDTH / 2 - 20, 12);
            graphSprite.setTextColor(TFT_CYAN, 0x2104);
            graphSprite.drawFloat(current, 2, GRAPH_WIDTH / 2 + 10, 12);
            graphSprite.setTextColor(TFT_WHITE, 0x2104);
            graphSprite.drawString("A", GRAPH_WIDTH / 2 + 60, 12);
            graphSprite.pushSprite(GRAPH_X, GRAPH_Y);
            lastGraphUpdate = now;
        }
    } else if (currentScreen == 2) {
        if (fabs(vcc - lastVcc) > threshold) {
            drawVoltageDisplay();
            lastVcc = vcc;
        }
    }
}