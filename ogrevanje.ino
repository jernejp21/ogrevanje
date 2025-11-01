/* Includes start */
#include <Arduino.h>
#include <U8g2lib.h>
#include <MUIU8g2.h>
#include <SPI.h>
#include <Versatile_RotaryEncoder.h>
#include <SD.h>

#include "sistem.h"
/* Includes end */

/* Defines start */
#define SD_CS PA11
#define LCD_CS 10
#define LCD_DC 9
#define LCD_RES 8
#define ENC_A 7
#define ENC_B 6
#define ENC_BT 5
#define ENKODER_ROTACIJA_BREZ 0
#define ENKODER_ROTACIJA_URA 1
#define ENKODER_ROTACIJA_KONT_URA -1
#define TEMP_ZALOG_AIN A3
#define TEMP_KROG_1_AIN A4
#define TEMP_KROG_2_AIN A5
#define CRPAKLA_KROG_1_DOUT 4
#define MES_VENT_HLAD_KROG_1_DOUT 3
#define MES_VENT_TOPL_KROG_1_DOUT 2
#define CRPAKLA_KROG_2_DOUT A0
#define MES_VENT_HLAD_KROG_2_DOUT A1
#define MES_VENT_TOPL_KROG_2_DOUT A2
#define TERMOSTAT_VKLOP_KROG1_DIN PC1
#define TERMOSTAT_VKLOP_KROG2_DIN PC0

#define DISPLAY_LED PD9
#define EKRAN_GLAVNI_ZASLON 0
#define EKRAN_MENI_ZASLON 1

#define STATE_IDLE 0
#define STATE_ACTIVE_SCREEN 1
#define STATE_MENU_SCREEN 2
#define STATE_LEAVE_MENU 3

#define MS_V_MIN 1000*60
#define KP_FAKTOR 10
#define KI_FAKTOR 0.1
#define KD_FAKTOR 100

#define UPOR 1000 // omov
#define ADC_MAX_VREDNOST 4096
/* Defines end */

/* Typedefs start */
typedef struct
{
  //uint8_t mes_vent_hlad;
  //uint8_t mes_vent_topl;
  uint8_t mes_vent_hlad_pin;
  uint8_t mes_vent_topl_pin;
  //uint8_t crpalka;
  uint8_t crpalka_pin;
  uint8_t termostat_vklop;
  uint8_t termostat_vklop_pin;
  float temp_kroga;
  uint8_t temp_kroga_pin;
  uint8_t temp_zelena;
  uint8_t Kp;
  uint8_t Ki;
  uint8_t Kd;
  float integral;
  float odvod;
  uint8_t mrtvi_hod;
  float napaka_prej;
  int8_t ventil_smer;
} ogrevalni_krog_t;
/* Typedefs end */

/* Global variables declarations start */
// Orientation, CS, DC(A0), Reset
U8G2_ST7567_OS12864_1_4W_HW_SPI u8g2(U8G2_R2, LCD_CS, LCD_DC, LCD_RES);

// Pin A, Pin B, Button Pin
Versatile_RotaryEncoder encoder(ENC_A, ENC_B, ENC_BT);

int8_t enkoder_smer;
int8_t enkoder_gumb_pritisnjen;

ogrevalni_krog_t krog1, krog2;
int8_t temp_hranilnika = 40;
uint8_t cas_zakasnitve = 1; // v minutah
uint8_t cas_vzorcenja = 1;
float ki_omejitev = 0;

uint8_t state_machine;
unsigned long int prejsnji_cas;
uint8_t ali_narisem = 1;
unsigned long gumb_cas;
uint8_t g_prikazi_stran;
/* Global variables declarations end */

/* Functions prototypes start */
uint8_t mui_hrule(mui_t *ui, uint8_t msg);
uint8_t izhod_iz_menija(mui_t *ui, uint8_t msg);
void obravnavaj_vrtenje(int8_t rotation);
void obravnavaj_gumb(void);
void narisi_glaven_zaslon(void);
void izrisi_stran(void);
void zatemnitev_zaslona();
void handle_state_machine();
void preberi_vhodne_signale();
void dobi_temperaturo();
uint8_t pid_zanka(ogrevalni_krog_t *krog);
void narisi_zaslon();
void krmiljenje_ventilov(ogrevalni_krog_t *krog);
void shrani_dnevnik(ogrevalni_krog_t *krog);
/* Functions prototypes end */

/* MUI definition */
MUIU8G2 mui;

muif_t muif_list[] = {
  MUIF_U8G2_FONT_STYLE(0, u8g2_font_helvR08_te),
  MUIF_U8G2_FONT_STYLE(1, u8g2_font_helvB08_te),

  MUIF_RO("HR", mui_hrule),
  MUIF_U8G2_LABEL(),
  MUIF_RO("GP", mui_u8g2_goto_data),
  MUIF_BUTTON("GC", mui_u8g2_goto_form_w1_pi),

  MUIF_U8G2_U8_MIN_MAX("P1", &krog1.Kp, 1, 10, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("I1", &krog1.Ki, 1, 10, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("D1", &krog1.Kd, 0, 10, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("T1", &krog1.temp_zelena, 15, 80, mui_u8g2_u8_min_max_wm_mud_pi),

  MUIF_U8G2_U8_MIN_MAX("P2", &krog2.Kp, 1, 10, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("I2", &krog2.Ki, 1, 10, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("D2", &krog2.Kd, 0, 10, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("T2", &krog2.temp_zelena, 15, 80, mui_u8g2_u8_min_max_wm_mud_pi),

  MUIF_U8G2_U8_MIN_MAX("TZ", &cas_zakasnitve, 0, 60, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_S8_MIN_MAX("TH", &temp_hranilnika, 20, 80, mui_u8g2_u8_min_max_wm_mud_pi),

  /* a button for the menu... */
  MUIF_BUTTON("GO", mui_u8g2_btn_goto_wm_fi),
  MUIF_BUTTON("BK", mui_u8g2_btn_back_wm_fi),
  MUIF_RO("EX", izhod_iz_menija)
};


fds_t fds_data[] =
  MUI_FORM(1)
  MUI_STYLE(1)
  MUI_LABEL(5, 8, "Glavni meni")
  MUI_STYLE(0)
  MUI_XY("HR", 0, 11)
  MUI_DATA("GP",
    MUI_20 "Krog 1|"
    MUI_21 "Krog 2|"
    MUI_30 "Nastavitve|"
    MUI_250 "Izhod")
  MUI_XYA("GC", 5, 24, 0)
  MUI_XYA("GC", 5, 36, 1)
  MUI_XYA("GC", 5, 48, 2)
  MUI_XYA("GC", 5, 60, 3)
  
  MUI_FORM(20)
  MUI_STYLE(1)
  MUI_LABEL(5, 8, "Krog 1")
  MUI_STYLE(0)
  MUI_XY("HR", 0, 11)
  MUI_LABEL(5, 23, "P konstanta:")
  MUI_LABEL(5, 35, "I konstanta:")
  MUI_LABEL(5, 47, "D konstanta:")
  MUI_LABEL(5, 59, "Temp. kroga:")
  MUI_XY("P1", 70, 23)
  MUI_XY("I1", 70, 35)
  MUI_XY("D1", 70, 47)
  MUI_XY("T1", 70, 59)
  MUI_XYT("BK", 110, 59, " Nazaj ")
  
  MUI_FORM(21)
  MUI_STYLE(1)
  MUI_LABEL(5, 8, "Krog 2")
  MUI_STYLE(0)
  MUI_XY("HR", 0, 11)
  MUI_LABEL(5, 23, "P konstanta:")
  MUI_LABEL(5, 35, "I konstanta:")
  MUI_LABEL(5, 47, "D konstanta:")
  MUI_LABEL(5, 59, "Temp. kroga:")
  MUI_XY("P2", 70, 23)
  MUI_XY("I2", 70, 35)
  MUI_XY("D2", 70, 47)
  MUI_XY("T2", 70, 59)
  MUI_XYT("BK", 110, 59, " Nazaj ")
  
  MUI_FORM(30)
  MUI_STYLE(1)
  MUI_LABEL(5, 8, "Nastavitve")
  MUI_STYLE(0)
  MUI_XY("HR", 0, 11)
  MUI_LABEL(5, 23, "Čas zaslona:")
  MUI_LABEL(5, 35, "Temp. hranilnika:")
  MUI_XY("TZ", 70, 23)
  MUI_XY("TH", 90, 35)
  MUI_XYT("BK", 20, 60, " Nazaj ")
  
  MUI_FORM(250)
  MUI_STYLE(0)
  MUI_AUX("EX")
;
/* MUI definition end*/

uint8_t mui_hrule(mui_t *ui, uint8_t msg) {
  u8g2_t *u8g2 = mui_get_U8g2(ui);
  switch (msg) {
    case MUIF_MSG_DRAW:
      u8g2_DrawHLine(u8g2, 0, mui_get_y(ui), u8g2_GetDisplayWidth(u8g2));
      break;
  }
  return 0;
}

uint8_t izhod_iz_menija(mui_t *ui, uint8_t msg) {
  state_machine = STATE_LEAVE_MENU;
  return 0;
}

void obravnavaj_vrtenje(int8_t rotation) {
  gumb_cas = millis();
  ali_narisem = 1;

  if (state_machine == STATE_IDLE) {
    state_machine = STATE_ACTIVE_SCREEN;
  }

  if (state_machine == STATE_MENU_SCREEN) {
    if (rotation == ENKODER_ROTACIJA_KONT_URA) {
      mui.prevField();
    }
    if (rotation == ENKODER_ROTACIJA_URA) {
      mui.nextField();
    }
  }
}

void obravnavaj_gumb() {
  gumb_cas = millis();
  ali_narisem = 1;

  if (state_machine == STATE_MENU_SCREEN) {
    mui.sendSelect();
  }
  if (state_machine != STATE_MENU_SCREEN) {
    state_machine = STATE_MENU_SCREEN;
  }
}

void narisi_glaven_zaslon() {
  u8g2.setFont(u8g2_font_helvR08_te);
  u8g2.drawXBMP(0, 0, LCD_BITMAP_WIDTH, LCD_BITMAP_HEIGHT, lcd_shema);
  u8g2.setCursor(0, 8);
  u8g2.print("T3= ");
  u8g2.print(temp_hranilnika);
  u8g2.print("°C");
  u8g2.print(" T4= ");
  u8g2.print((int)(krog1.temp_kroga));
  u8g2.print("°C");
  u8g2.print(" T6= ");
  u8g2.print((int)(krog2.temp_kroga));
  u8g2.print("°C");
}

void izrisi_stran(void) {
  static uint8_t is_next_page = 0;

  // call to first page, if required
  if (is_next_page == 0) {
    u8g2.firstPage();
    is_next_page = 1;
  }

  // draw our screen
  narisi_glaven_zaslon();

  // call to next page
  if (u8g2.nextPage() == 0) {
    is_next_page = 0;  // ensure, that first page is called
  }
}

void zatemnitev_zaslona() {
  if ((millis() - gumb_cas) > cas_zakasnitve * MS_V_MIN && state_machine != STATE_IDLE) {
    state_machine = STATE_IDLE;
    digitalWrite(DISPLAY_LED, 0);
  }
}

void handle_state_machine() {
  switch (state_machine) {
    case STATE_IDLE:
      g_prikazi_stran = EKRAN_GLAVNI_ZASLON;
      digitalWrite(DISPLAY_LED, 0);
      break;
    
    case STATE_ACTIVE_SCREEN:
      g_prikazi_stran = EKRAN_GLAVNI_ZASLON;
      digitalWrite(DISPLAY_LED, 1);
      break;

    case STATE_MENU_SCREEN:
      g_prikazi_stran = EKRAN_MENI_ZASLON;
      digitalWrite(DISPLAY_LED, 1);
      if (!mui.isFormActive()) {
        mui.gotoForm(1, 0);
      }
      break;

    case STATE_LEAVE_MENU:
      if (mui.isFormActive()) {
        mui.leaveForm();
      }
      state_machine = STATE_ACTIVE_SCREEN;
      ali_narisem = 1;
      break;

    default:
      break;
  }
}

void preberi_vhodne_signale() {

  krog1.termostat_vklop = digitalRead(TERMOSTAT_VKLOP_KROG1_DIN);
  krog2.termostat_vklop = digitalRead(TERMOSTAT_VKLOP_KROG2_DIN);
  encoder.ReadEncoder();
}
uint32_t ain;
float upor;
void dobi_temperaturo() {
  //uint32_t ain;

  analogReadResolution(12);
  ain = analogRead(krog1.temp_kroga_pin);
  //Serial.print("Senzor1: ");
  //Serial.println(ain);
  upor = (float)(ain * UPOR) / (float)(ADC_MAX_VREDNOST - ain);
  krog1.temp_kroga = (upor - 1000) * 100 / 385;

  analogReadResolution(12);
  ain = analogRead(krog2.temp_kroga_pin);
  //Serial.print("Senzor2: ");
  //Serial.println(ain);
  upor = (float)(ain * UPOR) / (float)(ADC_MAX_VREDNOST - ain);
  krog2.temp_kroga = (upor - 1000) * 100 / 385;
}

float pid_temp;
uint8_t pid_zanka(ogrevalni_krog_t *krog) {
  uint8_t ukaz = 0;
  float napaka = (float)krog->temp_zelena - (float)krog->temp_kroga;

  krog->integral += napaka;

  ki_omejitev = 100 / krog->Ki / KI_FAKTOR;

  if (krog->integral > ki_omejitev) {
    krog->integral = ki_omejitev;
  }
  if (krog->integral < -ki_omejitev) {
    krog->integral = -ki_omejitev;
  }

  //der = der + 0.05 *(Kd * (napaka - napaka_prej) - der)
  krog->odvod = napaka - krog->napaka_prej;

  pid_temp = (float)(napaka * krog->Kp * KP_FAKTOR) + (float)(krog->integral * krog->Ki * KI_FAKTOR) + (float)(krog->odvod * krog->Kd * KD_FAKTOR);

  if (abs(krog->temp_kroga - pid_temp) < krog->mrtvi_hod) {
    ukaz = 0;
  }

  else if (pid_temp > krog->temp_kroga) {
    ukaz = 1;
  }

  else if (pid_temp < krog->temp_kroga) {
    ukaz = -1;
  }
  
  krog->napaka_prej = napaka;

  return ukaz;
}

void narisi_zaslon() {
  if (ali_narisem) {
    u8g2.firstPage();
    do {
      switch (g_prikazi_stran) {
        case EKRAN_GLAVNI_ZASLON:
          narisi_glaven_zaslon();
          break;

        case EKRAN_MENI_ZASLON:
          mui.draw();
          break;

        default:
          break;
      }
    } while (u8g2.nextPage());
    ali_narisem = 0;
  }
}

void krmiljenje_ventilov(ogrevalni_krog_t *krog) {
  if (krog->termostat_vklop) {
    digitalWrite(krog->crpalka_pin, HIGH);
    int8_t ventil_smer = pid_zanka(krog);
    krog->ventil_smer = ventil_smer;

    if (ventil_smer == -1) {  // hladna voda
      digitalWrite(krog->mes_vent_hlad_pin, HIGH);
      digitalWrite(krog->mes_vent_topl_pin, LOW);
    }
    if (ventil_smer == 0) {  // pusti ventil v trenutni poziciji
      digitalWrite(krog->mes_vent_hlad_pin, LOW);
      digitalWrite(krog->mes_vent_topl_pin, LOW);
    }
    if (ventil_smer == 1) {  // topla voda
      digitalWrite(krog->mes_vent_hlad_pin, LOW);
      digitalWrite(krog->mes_vent_topl_pin, HIGH);
    }
  }
  else {
    // TODO
    // Najprej se mora zapreti mešalni ventil, šele potem se ugasne črpalka.
    // Mešalni ventil se zapira, črpalka pa deluje še 5 min po signalu za izklop.
    //digitalWrite(krog->mes_vent_hlad_pin, LOW);
    
    digitalWrite(krog->crpalka_pin, LOW);
  }
}

String log_name;

void shrani_dnevnik(ogrevalni_krog_t *krog) {
  if (krog->termostat_vklop) {
    String dataString;
    dataString += String(krog->temp_zelena);
    dataString += ";";
    dataString += String(krog->temp_kroga);
    dataString += ";";
    dataString += String(krog->ventil_smer);
    if (SD.begin(SD_CS)) {
      //Serial.println("SD kartica pripravljena.");
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.println(dataString);
        dataFile.close();
        //Serial.println(dataString);
      }
    }
    else {
      //Serial.println("Ni SD Kartice.");
    }
  }
  else {
    log_name = "test1.txt";
  }

}

void setup() {
  // put your setup code here, to run once:

  /* Pin setup */

  pinMode(DISPLAY_LED, OUTPUT);

  krog1.crpalka_pin = CRPAKLA_KROG_1_DOUT;
  digitalWrite(CRPAKLA_KROG_1_DOUT, LOW);
  
  krog1.mes_vent_hlad_pin = MES_VENT_HLAD_KROG_1_DOUT;
  digitalWrite(MES_VENT_HLAD_KROG_1_DOUT, LOW);
  
  krog1.mes_vent_topl_pin = MES_VENT_TOPL_KROG_1_DOUT;
  digitalWrite(MES_VENT_TOPL_KROG_1_DOUT, LOW);
  
  pinMode(CRPAKLA_KROG_1_DOUT, OUTPUT);
  pinMode(MES_VENT_HLAD_KROG_1_DOUT, OUTPUT);
  pinMode(MES_VENT_TOPL_KROG_1_DOUT, OUTPUT);

  krog2.crpalka_pin = CRPAKLA_KROG_2_DOUT;
  digitalWrite(CRPAKLA_KROG_2_DOUT, LOW);

  krog2.mes_vent_hlad_pin = MES_VENT_HLAD_KROG_2_DOUT;
  digitalWrite(MES_VENT_HLAD_KROG_2_DOUT, LOW);

  krog2.mes_vent_topl_pin = MES_VENT_TOPL_KROG_2_DOUT;
  digitalWrite(MES_VENT_TOPL_KROG_2_DOUT, LOW);
  
  pinMode(CRPAKLA_KROG_2_DOUT, OUTPUT);
  pinMode(MES_VENT_HLAD_KROG_2_DOUT, OUTPUT);
  pinMode(MES_VENT_TOPL_KROG_2_DOUT, OUTPUT);

  krog1.temp_kroga_pin = TEMP_KROG_1_AIN;
  pinMode(TEMP_KROG_1_AIN, INPUT);
  krog2.temp_kroga_pin = TEMP_KROG_2_AIN;
  pinMode(TEMP_KROG_2_AIN, INPUT);
  pinMode(TEMP_ZALOG_AIN, INPUT);

  pinMode(TERMOSTAT_VKLOP_KROG1_DIN, INPUT_PULLDOWN);
  pinMode(TERMOSTAT_VKLOP_KROG2_DIN, INPUT_PULLDOWN);
  /* Pin setup end*/

  /* Display setup */
  u8g2.begin(ENC_BT, ENC_A, ENC_B);
  u8g2.setContrast(40);
  mui.begin(u8g2, fds_data, muif_list, sizeof(muif_list) / sizeof(muif_t));
  u8g2.enableUTF8Print();
  /* Display setup end */

  /* Encoder setup */
  encoder.setHandleRotate(obravnavaj_vrtenje);
  encoder.setHandlePressRelease(obravnavaj_gumb);
  /* Encoder setup end */

  state_machine = STATE_IDLE;
  krog1.Kp = 3;
  krog1.Ki = 10;
  krog1.Kd = 2;
  krog1.temp_zelena = 45;
  krog1.mrtvi_hod = 2;

  krog2.Kp = 1;
  krog2.Ki = 3;
  krog2.Kd = 1;
  krog2.temp_zelena = 28;
  krog2.mrtvi_hod = 2;

  Serial.begin(115200);
  Serial.println("Začetek programa.");
}

void loop() {
  // put your main code here, to run repeatedly:
  static unsigned long int pid_zanka_cas = 0;
  zatemnitev_zaslona();
  preberi_vhodne_signale();
  handle_state_machine();
  dobi_temperaturo(); //TODO

  if ((millis() - pid_zanka_cas) > 1000) {
    pid_zanka_cas = millis();
    krmiljenje_ventilov(&krog1);
    krmiljenje_ventilov(&krog2);
    shrani_dnevnik(&krog1);
    //shrani_dnevnik(&krog2);
    ali_narisem = 1;
  }

  narisi_zaslon();
}
