//---------------------------------------------------------
/*
 
 test.ino
 
 Program for writing to Newhaven Display's 160x128 Graphic Color OLED with SEPS525 controller.
 
 Pick one up today in the Newhaven Display shop!
 ------> http://www.newhavendisplay.com/nhd169160128ugc3-p-5603.html
 
 This code is written for the Arduino Uno R3.

 Copyright (c) 2015 - Newhaven Display International, LLC.
 w
 Newhaven Display invests time and resources providing this open source code,
 please support Newhaven Display by purchasing products from Newhaven Display!
 
  */
//---------------------------------------------------------

// The 8 bit data bus is connected to PORTD[7..0]
#include <math.h>
#include <Adafruit_GFX.h>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define WIDTH      159
#define HEIGHT     127 
#define   SDI_PIN   23    // SDI (serial mode) signal connected to pin 6
#define   SCL_PIN   18    // SCL (serial mdoe) signal connected to pin 7
#define    RS_PIN    5    // RS signal connected to pin 8
#define    RW_PIN    9    // R/W (6800 mode) signal connected to pin 9
#define    WR_PIN    9    // /WR (8080 mode) signal connected to pin 9
#define     E_PIN    9    // E (6800 mode) signal connected to pin 10
#define    RD_PIN    9    // /RD (8080 mode) signal connected to pin 10
#define   RES_PIN    0    // /RES signal connected to pin 11
#define    CS_PIN   15    // /CS signal connected to pin 12
#define    PS_PIN    9    // PS signal connected to pin A0
#define   CPU_PIN    9    // CPU signal connected to pin A1
#define   LVL_DIR    9    // DIR (direction control) signal of level shifter IC connected to pin A2
#define   LVL_OEN    9    // /OE (output enable) signal of level shifter IC connected to pin A3

#define    RED  0x0000FF
#define  GREEN  0x00FF00
#define   BLUE  0xFF0000
#define  WHITE  0xFFFFFF
#define  BLACK  0x000000

class Package
{
  public:
    Package(int src_x, int src_y, int dest_x, int dest_y, int pace, int len, int color);
    void Tick();
    bool HasReachDestination(); 
    void Draw(GFXcanvas16* canvas);
    unsigned char rect_x1;
    unsigned char rect_y1;
    unsigned char rect_x2;
    unsigned char rect_y2;
    int _color;
  private:
    int _current;
    int _distance;
    int _src_x;
    int _src_y;
    int _dest_x;
    int _dest_y;
    int _speed;
    float _angle;
    int _len;
    
    
};

Package::Package(int src_x, int src_y, int dest_x, int dest_y, int pace, int len, int c)
{
  _current = 0;
  _src_x = src_x;
  _src_y = src_y;
  _speed = pace;
  _len = len;
  _color = c;
  _dest_x = dest_x;
  _dest_y = dest_y;
  int x=dest_x - src_x;
  int y=dest_y - src_y;
  _distance = sqrt((x*x) + (y*y));  
  _angle = atan2(y, x);
  rect_x1 = MAX(0, src_x - len);
  rect_y1 = MAX(0, src_y - len);
  rect_x2 = MIN(WIDTH, src_x + len);
  rect_y2 = MIN(HEIGHT, src_y + len);
}


void Package::Tick(){
  if (HasReachDestination()){
    _current = 0;
  }else {
    _current += _speed;  
  }
}

void Package::Draw(GFXcanvas16* canvas){
  int x = _current * cos(_angle) + _src_x;
  int y = _current * sin(_angle) + _src_y;
  rect_x1 = MAX(0, x - _len);
  rect_y1 = MAX(0, y - _len);
  rect_x2 = MIN(WIDTH, x + _len);
  rect_y2 = MIN(HEIGHT, y + _len);
  canvas->fillCircle(x, y, _len, _color);
  //canvas->drawRect(rect_x1, rect_y1, _len*2, _len*2, _color);
}

bool Package::HasReachDestination(){
  return _current > _distance;
}


static GFXcanvas16 canvas = GFXcanvas16(WIDTH+1, HEIGHT+1);
static GFXcanvas16 canvas_bg = GFXcanvas16(WIDTH+1, HEIGHT+1);


/*********************************/
/****** LOW LEVEL FUNCTIONS ******/
/************* START *************/
/*********************************/

void OLED_Command_160128RGB(unsigned char c)        // send command to OLED
{
    unsigned char i;
    unsigned char mask = 0x80;
    
    digitalWrite(CS_PIN, LOW);
    digitalWrite(RS_PIN, LOW);
    for(i=0;i<8;i++)
    {
        digitalWrite(SCL_PIN, LOW);
        if((c & mask) >> 7 == 1)
        {
            digitalWrite(SDI_PIN, HIGH);
        }
        else
        {
            digitalWrite(SDI_PIN, LOW);
        }
        digitalWrite(SCL_PIN, HIGH);
        c = c << 1;
    }
    digitalWrite(CS_PIN, HIGH);
}

void OLED_Data_160128RGB(unsigned char d)        // send data to OLED
{
    unsigned char i;
    unsigned char mask = 0x80;
    
    digitalWrite(CS_PIN, LOW);
    digitalWrite(RS_PIN, HIGH);
    for(i=0;i<8;i++)
    {
        digitalWrite(SCL_PIN, LOW);
        if((d & mask) >> 7 == 1)
        {
            digitalWrite(SDI_PIN, HIGH);
        }
        else
        {
            digitalWrite(SDI_PIN, LOW);
        }
        digitalWrite(SCL_PIN, HIGH);
        d = d << 1;
    }
    digitalWrite(CS_PIN, HIGH);
}

void OLED_SerialPixelData_160128RGB(unsigned char d)    // serial write for pixel data
{
    unsigned char i;
    unsigned char mask = 0x80;
    digitalWrite(CS_PIN, LOW);
    digitalWrite(RS_PIN, HIGH);
    for(i=0;i<6;i++)
    {
        digitalWrite(SCL_PIN, LOW);
        if((d & mask) >> 7 == 1)
        {
            digitalWrite(SDI_PIN, HIGH);
        }
        else
        {
            digitalWrite(SDI_PIN, LOW);
        }
        digitalWrite(SCL_PIN, HIGH);
        d = d << 1;
    }
    digitalWrite(CS_PIN, HIGH);
}

void OLED_SetColumnAddress_160128RGB(unsigned char x_start, unsigned char x_end)    // set column address start + end
{
    OLED_Command_160128RGB(0x17);
    OLED_Data_160128RGB(x_start);
    OLED_Command_160128RGB(0x18);
    OLED_Data_160128RGB(x_end);
}

void OLED_SetRowAddress_160128RGB(unsigned char y_start, unsigned char y_end)    // set row address start + end
{
    OLED_Command_160128RGB(0x19);
    OLED_Data_160128RGB(y_start);
    OLED_Command_160128RGB(0x1A);
    OLED_Data_160128RGB(y_end);
}

void OLED_WriteMemoryStart_160128RGB(void)    // write to RAM command
{
    OLED_Command_160128RGB(0x22);
}

void OLED_Pixel_160128RGB(unsigned long color)    // write one pixel of a given color
{

    OLED_SerialPixelData_160128RGB((color>>16));
    OLED_SerialPixelData_160128RGB((color>>8));
    OLED_SerialPixelData_160128RGB(color);
}

void OLED_Pixel_160128RGB565(int16_t color)    // write one pixel of a given color
{

    OLED_SerialPixelData_160128RGB(((color & 0x1F)) << 3);
    OLED_SerialPixelData_160128RGB(((color & 0x7E0) >> 5) << 2);
    OLED_SerialPixelData_160128RGB(((color & 0xF800) >> 11) << 3);
    
}
void OLED_SetPosition_160128RGB(unsigned char x_pos, unsigned char y_pos)    // set x,y address
{
    OLED_Command_160128RGB(0x20);
    OLED_Data_160128RGB(x_pos);
    OLED_Command_160128RGB(0x21);
    OLED_Data_160128RGB(y_pos);
}

void OLED_FillScreen_160128RGB(unsigned long color)    // fill screen with a given color
{
    unsigned int i;
    OLED_SetPosition_160128RGB(0,0);
    OLED_WriteMemoryStart_160128RGB();
    for(i=0;i<20480;i++)
    {
        OLED_Pixel_160128RGB(color);
    }
}

void OLED_FillScreen_160128RGB565(int16_t color)    // fill screen with a given color
{
    unsigned int i;
    OLED_SetPosition_160128RGB(0,0);
    OLED_WriteMemoryStart_160128RGB();
    for(i=0;i<20480;i++)
    {
        OLED_Pixel_160128RGB565(color);
    }
}

void OLED_FillRectRGB565(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2, GFXcanvas16* fcanvas)
{
    unsigned int i,j;
    int x = x2 - x1;
    int y = y2 - y1;
    if (x <= 0 || y <= 0){
      return;
    }
    OLED_SetPosition_160128RGB(x1,y1);
    OLED_SetColumnAddress_160128RGB(x1, x2-1);
    OLED_SetRowAddress_160128RGB(y1, y2-1);
    OLED_WriteMemoryStart_160128RGB();
    
    for(j=0;j<y;j++)
    {
      for(i=0;i<x;i++){        
        int pixel = fcanvas->getPixel(i+x1,j+y1);
        OLED_Pixel_160128RGB565(pixel);  
      }
      
    }
}

void draw_canvas_bg()
{
    unsigned int i;
    OLED_SetPosition_160128RGB(0,0);
    OLED_SetColumnAddress_160128RGB(0, WIDTH);
    OLED_SetRowAddress_160128RGB(0, HEIGHT);
    OLED_WriteMemoryStart_160128RGB();
    uint16_t* buffer = canvas_bg.getBuffer();
    for(i=0;i<20480;i++)
    {
        OLED_Pixel_160128RGB565(buffer[i]);
    }
}

/*===============================*/
/*===== LOW LEVEL FUNCTIONS =====*/
/*============= END =============*/
/*===============================*/

/*********************************/
/******** INITIALIZATION *********/
/************ START **************/
/*********************************/
void ESP_32_Init_Pins(){
    pinMode(LVL_OEN, OUTPUT);                       // configure LVL_OEN as output
    digitalWrite(LVL_OEN, LOW);
    pinMode(LVL_DIR, OUTPUT);                       // configure LVL_DIR as output
    digitalWrite(LVL_DIR, HIGH);
    //DDRD = 0xFF;                                    // configure PORTD as output
    pinMode(RS_PIN, OUTPUT);                        // configure RS_PIN as output
    pinMode(RES_PIN, OUTPUT);                       // configure RES_PIN as output
    pinMode(CS_PIN, OUTPUT);                        // configure CS_PIN as output
    pinMode(PS_PIN, OUTPUT);                        // configure PS_PIN as output
    pinMode(CPU_PIN, OUTPUT);                       // configure CPU_PIN as output
    digitalWrite(LVL_OEN, LOW);
    digitalWrite(CS_PIN, HIGH);                     // set CS_PIN
    
    pinMode(SDI_PIN, OUTPUT);                   // configure SDI_PIN as output
    pinMode(SCL_PIN, OUTPUT);                   // configure SCL_PIN as output
    //PORTD = 0x00;                               // reset SDI_PIN and SCL_PIN, ground DB[5..0] of the display
    digitalWrite(PS_PIN, LOW);                  // reset PS_PIN
}
void OLED_Init_160128RGB(void)      //OLED initialization
{
    digitalWrite(RES_PIN, LOW);
    delay(500);
    digitalWrite(RES_PIN, HIGH);
    delay(500);
    
    OLED_Command_160128RGB(0x04);// Set Normal Driving Current
    OLED_Data_160128RGB(0x03);// Disable Oscillator Power Down
    delay(2);
    OLED_Command_160128RGB(0x04); // Enable Power Save Mode
    OLED_Data_160128RGB(0x00); // Set Normal Driving Current
    delay(2); // Disable Oscillator Power Down
    OLED_Command_160128RGB(0x3B);
    OLED_Data_160128RGB(0x00);
    OLED_Command_160128RGB(0x02);
    OLED_Data_160128RGB(0x01); // Set EXPORT1 Pin at Internal Clock
    // Oscillator operates with external resister.
    // Internal Oscillator On
    OLED_Command_160128RGB(0x03);
    OLED_Data_160128RGB(0x90); // Set Clock as 90 Frames/Sec
    OLED_Command_160128RGB(0x80);
    OLED_Data_160128RGB(0x01); // Set Reference Voltage Controlled by External Resister
    OLED_Command_160128RGB(0x08);// Set Pre-Charge Time of Red
    OLED_Data_160128RGB(0x04);
    OLED_Command_160128RGB(0x09);// Set Pre-Charge Time of Green
    OLED_Data_160128RGB(0x05);
    OLED_Command_160128RGB(0x0A);// Set Pre-Charge Time of Blue
    OLED_Data_160128RGB(0x05);
    OLED_Command_160128RGB(0x0B);// Set Pre-Charge Current of Red
    OLED_Data_160128RGB(0x9D);
    OLED_Command_160128RGB(0x0C);// Set Pre-Charge Current of Green
    OLED_Data_160128RGB(0x8C);
    OLED_Command_160128RGB(0x0D);// Set Pre-Charge Current of Blue
    OLED_Data_160128RGB(0x57);
    OLED_Command_160128RGB(0x10);// Set Driving Current of Red
    OLED_Data_160128RGB(0x56);
    OLED_Command_160128RGB(0x11);// Set Driving Current of Green
    OLED_Data_160128RGB(0x4D);
    OLED_Command_160128RGB(0x12);// Set Driving Current of Blue
    OLED_Data_160128RGB(0x46);
    OLED_Command_160128RGB(0x13);
    OLED_Data_160128RGB(0xA0); // Set Color Sequence
    OLED_Command_160128RGB(0x14);
    OLED_Data_160128RGB(0x01); // Set MCU Interface Mode
    OLED_Command_160128RGB(0x16);
    OLED_Data_160128RGB(0x76); // Set Memory Write Mode
    OLED_Command_160128RGB(0x28);
    OLED_Data_160128RGB(0x7F); // 1/128 Duty (0x0F~0x7F)
    OLED_Command_160128RGB(0x29);
    OLED_Data_160128RGB(0x00); // Set Mapping RAM Display Start Line (0x00~0x7F)
    OLED_Command_160128RGB(0x06);
    OLED_Data_160128RGB(0x01); // Display On (0x00/0x01)
    OLED_Command_160128RGB(0x05); // Disable Power Save Mode
    OLED_Data_160128RGB(0x00); // Set All Internal Register Value as Normal Mode
    OLED_Command_160128RGB(0x15);
    OLED_Data_160128RGB(0x00); // Set RGB Interface Polarity as Active Low
    OLED_SetColumnAddress_160128RGB(0, WIDTH);
    OLED_SetRowAddress_160128RGB(0, HEIGHT);
}


/*===============================*/
/*======= INITIALIZATION ========*/
/*============= END =============*/
/*===============================*/
int package_size = 5;
Package package[5] = {
  Package(0, 0, 0, 0, 0, 5, 0xF800),
  Package(5, 30, 115, 5, 1, 5, 0x2D85),
  Package(5, 30, 155, 30, 2, 5, 0x2D85),//good
  Package(5, 96, 115, 123, 3, 5, 0xFFE0),
  Package(5, 96, 155, 96, 4, 5, 0xFFE0)//good
};
  
void setup()                                       // for Arduino, runs first at power on
{
    Serial.begin(115200);
    ESP_32_Init_Pins();
    OLED_Init_160128RGB();   
    canvas_bg.fillScreen(0x00);
    canvas_bg.fillRect(30, 0, 20, 5, 0x8430);
    canvas_bg.fillRect(110, 0, 20, 5, 0x8430);
    canvas_bg.fillRect(30, 123, 20, 5, 0x8430);
    canvas_bg.fillRect(110, 123, 20, 5, 0x8430);
    
    canvas_bg.fillRect(0, 22, 5, 20, 0x8430);
    canvas_bg.fillRect(0, 86, 5, 20, 0x8430);
    canvas_bg.fillRect(155, 22, 5, 20, 0x8430);
    canvas_bg.fillRect(155, 86, 5, 20, 0x8430);

    canvas_bg.drawLine(80, 0, 80, 128 ,0x8430);
    canvas_bg.drawLine(0, 64, 160, 64 ,0x8430);
    draw_canvas_bg();
}

void loop()                                         // main loop, runs after "setup()"
{
      
      for(int i=0; i<package_size; i++){
        
        OLED_FillRectRGB565(package[i].rect_x1, package[i].rect_y1, package[i].rect_x2, package[i].rect_y2, &canvas_bg); 
        package[i].Tick();
        canvas.fillScreen(0x00);
        package[i].Draw(&canvas); 
        OLED_FillRectRGB565(package[i].rect_x1, package[i].rect_y1, package[i].rect_x2, package[i].rect_y2, &canvas); 
      }
    
}
