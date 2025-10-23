/* Includes start */
#include <Arduino.h>
#include <U8g2lib.h>
#include <MUIU8g2.h>
#include <SPI.h>
#include <Versatile_RotaryEncoder.h>

#include "sistem.h"
/* Includes end */

/* Defines start */
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

#define DISPLAY_LED A2

#define STATE_IDLE 0
#define STATE_MAIN_SCREEN 1
#define STATE_MENU_SCREEN 2
#define STATE_LEAVE_MENU 3
/* Defines end */

/* Typedefs start */
typedef struct
{
  uint8_t mes_vent_hlad;
  uint8_t mes_vent_topl;
  uint8_t mes_vent_hlad_pin;
  uint8_t mes_vent_topl_pin;
  uint8_t crpalka;
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
uint8_t cas_zakasnitve = 1;
uint8_t cas_vzorcenja = 1;
int8_t ki_omejitev = 0;

uint8_t state_machine;
unsigned long int prejsnji_cas;
uint8_t ali_narisem = 1;
unsigned long gumb_cas;
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
void stanje_gumbov_zaslona();
void dobi_temperaturo();
uint8_t pid_zanka(ogrevalni_krog_t *krog);
void narisi_zaslon();
/* Functions prototypes end */

/* MUI definition */
MUIU8G2 mui;

muif_t muif_list[] = {
  MUIF_U8G2_FONT_STYLE(0, u8g2_font_helvR08_tr),
  MUIF_U8G2_FONT_STYLE(1, u8g2_font_helvB08_tr),

  MUIF_RO("HR", mui_hrule),
  MUIF_U8G2_LABEL(),
  MUIF_RO("GP", mui_u8g2_goto_data),
  MUIF_BUTTON("GC", mui_u8g2_goto_form_w1_pi),

  MUIF_U8G2_U8_MIN_MAX("P1", &krog1.Kp, 0, 10, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("I1", &krog1.Ki, 0, 10, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("D1", &krog1.Kd, 0, 10, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("T1", &krog1.temp_zelena, 15, 80, mui_u8g2_u8_min_max_wm_mud_pi),

  MUIF_U8G2_U8_MIN_MAX("P2", &krog2.Kp, 0, 10, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("I2", &krog2.Ki, 0, 10, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("D2", &krog2.Kd, 0, 10, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("T2", &krog2.temp_zelena, 15, 80, mui_u8g2_u8_min_max_wm_mud_pi),

  MUIF_U8G2_U8_MIN_MAX("TZ", &cas_zakasnitve, 0, 60, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_S8_MIN_MAX("TH", &temp_hranilnika, 20, 80, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_S8_MIN_MAX("KO", &ki_omejitev, 1, 20, mui_u8g2_u8_min_max_wm_mud_pi),

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
  MUI_LABEL(5, 23, "Cas zaslona:")
  MUI_LABEL(5, 35, "Temp. hranilnika:")
  MUI_LABEL(5, 47, "Ki omejitev:")
  MUI_XY("TZ", 70, 23)
  MUI_XY("TH", 90, 35)
  MUI_XY("KO", 80, 47)
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
  enkoder_smer = rotation;
  gumb_cas = millis();
  digitalWrite(DISPLAY_LED, 1);
}

void obravnavaj_gumb() {
  enkoder_gumb_pritisnjen = 1;
  gumb_cas = millis();
  digitalWrite(DISPLAY_LED, 1);
}

void narisi_glaven_zaslon() {
  u8g2.drawXBMP(0, 0, LCD_BITMAP_WIDTH, LCD_BITMAP_HEIGHT, lcd_shema);
  u8g2.setCursor(0, 8);
  u8g2.print("T3= ");
  u8g2.print(40);
  u8g2.print("C");
  u8g2.print("  T4= ");
  u8g2.print(35);
  u8g2.print("C");
  u8g2.print("  T6= ");
  u8g2.print(28);
  u8g2.print("C");
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
  if ((millis() - gumb_cas) > cas_zakasnitve * 1000) {
    digitalWrite(DISPLAY_LED, 0);
  }
}

void handle_state_machine() {
  switch (state_machine) {
    case STATE_IDLE:
      break;
    
    case STATE_MAIN_SCREEN:
      state_machine = STATE_MENU_SCREEN;
      Serial.println("STATE: STATE_MAIN_SCREEN");
      mui.gotoForm(1, 0);
      break;

    case STATE_MENU_SCREEN:
      Serial.println("STATE: STATE_MENU_SCREEN");
      if (!mui.isFormActive()) {
        Serial.println("Forma ni aktivna");
        state_machine = STATE_MAIN_SCREEN;
      } else {
        Serial.println("Forma je aktivna");
      }
      break;

    case STATE_LEAVE_MENU:
      Serial.println("STATE: STATE_LEAVE_MENU");
      mui.saveForm();
      mui.leaveForm();
      state_machine = STATE_MAIN_SCREEN;
      break;

    default:
      break;
  }
}

void stanje_gumbov_zaslona() {
  encoder.ReadEncoder();

  if (enkoder_smer == ENKODER_ROTACIJA_KONT_URA) {
    Serial.println("Vrtim v levo.");
    mui.prevField();
    ali_narisem = 1;
    enkoder_smer = ENKODER_ROTACIJA_BREZ;
  }
  if (enkoder_smer == ENKODER_ROTACIJA_URA) {
    Serial.println("Vrtim v desno.");
    mui.nextField();
    ali_narisem = 1;
    enkoder_smer = ENKODER_ROTACIJA_BREZ;
  }

  if (enkoder_gumb_pritisnjen) {
    Serial.println("Gumb je pritisnjen.");
    mui.sendSelect();

    ali_narisem = 1;
    enkoder_gumb_pritisnjen = 0;
    handle_state_machine();
  }
}

void dobi_temperaturo() {

}

uint8_t pid_zanka(ogrevalni_krog_t *krog) {
  uint8_t ukaz = 0;
  float napaka = (float)krog->temp_zelena - (float)krog->temp_kroga;

  krog->integral += napaka;

  if (krog->integral > ki_omejitev) {
    krog->integral = ki_omejitev;
  }
  if (krog->integral < -ki_omejitev) {
    krog->integral = -ki_omejitev;
  }

  //der = der + 0.05 *(Kd * (napaka - napaka_prej) - der)
  krog->odvod = napaka - krog->napaka_prej;

  float pid_temp = napaka * krog->Kp + krog->integral * krog->Ki + krog->odvod * krog->Kd;

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
  if (mui.isFormActive()) {

    /* menu is active: draw the menu */
    encoder.ReadEncoder();
    if (ali_narisem) {
      u8g2.firstPage();
      do {
        mui.draw();
      } while (u8g2.nextPage());
      ali_narisem = 0;
    }
  } else {
    /* menu not active: show something */
    izrisi_stran();
  }
}

void setup() {
  // put your setup code here, to run once:

  /* Pin setup */

  pinMode(DISPLAY_LED, OUTPUT);

  digitalWrite(CRPAKLA_KROG_1_DOUT, LOW);
  digitalWrite(MES_VENT_HLAD_KROG_1_DOUT, LOW);
  digitalWrite(MES_VENT_TOPL_KROG_1_DOUT, LOW);
  pinMode(CRPAKLA_KROG_1_DOUT, OUTPUT);
  pinMode(MES_VENT_HLAD_KROG_1_DOUT, OUTPUT);
  pinMode(MES_VENT_TOPL_KROG_1_DOUT, OUTPUT);

  digitalWrite(CRPAKLA_KROG_2_DOUT, LOW);
  digitalWrite(MES_VENT_HLAD_KROG_2_DOUT, LOW);
  digitalWrite(MES_VENT_TOPL_KROG_2_DOUT, LOW);
  pinMode(CRPAKLA_KROG_2_DOUT, OUTPUT);
  pinMode(MES_VENT_HLAD_KROG_2_DOUT, OUTPUT);
  pinMode(MES_VENT_TOPL_KROG_2_DOUT, OUTPUT);

  pinMode(TEMP_KROG_1_AIN, INPUT);
  pinMode(TEMP_KROG_2_AIN, INPUT);
  pinMode(TEMP_ZALOG_AIN, INPUT);
  /* Pin setup end*/

  /* Display setup */
  u8g2.begin(ENC_BT, ENC_A, ENC_B);
  u8g2.setFont(u8g2_font_profont10_tf);
  u8g2.setContrast(40);
  mui.begin(u8g2, fds_data, muif_list, sizeof(muif_list) / sizeof(muif_t));
  /* Display setup end */

  /* Encoder setup */
  encoder.setHandleRotate(obravnavaj_vrtenje);
  encoder.setHandlePressRelease(obravnavaj_gumb);
  /* Encoder setup end */

  state_machine = STATE_MAIN_SCREEN;

  Serial.begin(9600);
  Serial.println("Začetek programa.");
}

void loop() {
  // put your main code here, to run repeatedly:
  zatemnitev_zaslona();
  stanje_gumbov_zaslona();
  dobi_temperaturo(); //TODO
  pid_zanka(&krog1); //TODO
  narisi_zaslon();
}
