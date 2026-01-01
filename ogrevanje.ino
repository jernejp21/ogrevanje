/* Includes start */
#include <Arduino.h>
#include <U8g2lib.h>
#include <MUIU8g2.h>
#include <SPI.h>
#include <Versatile_RotaryEncoder.h>
#include <SD.h>

#include "sistem.h"
/* Includes end */



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
  //uint8_t termostat_vklop_pin;
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
  char ime_kroga[8];
  uint8_t st_dnevnika;
  uint8_t povecam_dnevnik;
  unsigned long int cas;
  uint8_t prezracevanje;
} ogrevalni_krog_t;
/* Typedefs end */

/* Global variables declarations start */
// Orientation, CS, DC(A0), Reset
U8G2_ST7567_OS12864_1_4W_HW_SPI u8g2(U8G2_R2, LCD_CS, LCD_DC, LCD_RES);

// Pin A, Pin B, Button Pin
Versatile_RotaryEncoder encoder(ENC_A, ENC_B, ENC_BT);

int8_t enkoder_smer;
int8_t enkoder_gumb_pritisnjen;

ogrevalni_krog_t krog1 = {.ime_kroga="krog1", .povecam_dnevnik=1, .cas=0x7FFFFFFF};
ogrevalni_krog_t krog2 = {.ime_kroga="krog2", .povecam_dnevnik=1, .cas=0x7FFFFFFF};
float temp_hranilnika;
int8_t temp_hranilnika_zelena;
uint8_t cas_zakasnitve;  // v minutah
uint8_t cas_vzorcenja;  // v sekundah
float ki_omejitev;

uint8_t state_machine;
uint8_t ali_narisem;
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
float dobi_temperaturo(uint32_t adc_meritev);
uint8_t pid_zanka(ogrevalni_krog_t *krog);
void narisi_zaslon();
void krmiljenje_ventilov(ogrevalni_krog_t *krog);
void shrani_dnevnik(ogrevalni_krog_t *krog);
void prezracevanje(ogrevalni_krog_t *krog);
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
  MUIF_U8G2_S8_MIN_MAX("TH", &temp_hranilnika_zelena, 20, 80, mui_u8g2_u8_min_max_wm_mud_pi),
  MUIF_U8G2_U8_MIN_MAX("MH", &krog1.mrtvi_hod, 0, 5, mui_u8g2_u8_min_max_wm_mud_pi),

  MUIF_VARIABLE("C1", &krog1.prezracevanje, mui_u8g2_u8_chkbox_wm_pi),
  MUIF_VARIABLE("C2", &krog2.prezracevanje, mui_u8g2_u8_chkbox_wm_pi),

  /* a button for the menu... */
  MUIF_BUTTON("GT", mui_u8g2_btn_goto_wm_fi),
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
    MUI_40 "Info|"
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
  MUI_LABEL(78, 59, "°C")
  MUI_XY("P1", 70, 23)
  MUI_XY("I1", 70, 35)
  MUI_XY("D1", 70, 47)
  MUI_XY("T1", 65, 59)
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
  MUI_LABEL(78, 59, "°C")
  MUI_XY("P2", 70, 23)
  MUI_XY("I2", 70, 35)
  MUI_XY("D2", 70, 47)
  MUI_XY("T2", 65, 59)
  MUI_XYT("BK", 110, 59, " Nazaj ")
  
  MUI_FORM(30)
  MUI_STYLE(1)
  MUI_LABEL(5, 8, "Nastavitve 1/2")
  MUI_STYLE(0)
  MUI_XY("HR", 0, 11)
  MUI_LABEL(5, 23, "Čas zaslona:")
  MUI_LABEL(84, 23, "min")
  MUI_LABEL(5, 35, "Temp. hranilnika:")
  MUI_LABEL(98, 35, "°C")
  MUI_LABEL(5, 47, "Mrtvi hod:")
  MUI_LABEL(71, 47, "°C")
  MUI_XY("TZ", 70, 23)
  MUI_XY("TH", 85, 35)
  MUI_XY("MH", 58, 47)
  MUI_XYAT("GT", 20, 60, 1, " Nazaj ")
  MUI_XYAT("GT", 100, 60, 31, " Naprej ")

  MUI_FORM(31)
  MUI_STYLE(1)
  MUI_LABEL(5, 8, "Nastavitve 2/2")
  MUI_STYLE(0)
  MUI_XY("HR", 0, 11)
  MUI_LABEL(5, 23, "Prezračevanje K1:")
  MUI_LABEL(5, 35, "Prezračevanje K2:")
  MUI_XY("C1", 90, 23)
  MUI_XY("C2", 90, 35)
  MUI_XYAT("GT", 20, 60, 30, " Nazaj ")
  MUI_XYAT("GT", 100, 60, 1, " Naprej ")


  MUI_FORM(40)
  MUI_STYLE(1)
  MUI_LABEL(5, 8, "Info")
  MUI_STYLE(0)
  MUI_XY("HR", 0, 11)
  MUI_LABEL(5, 23, "Verzija:")
  MUI_LABEL(70, 23, VERZIJA)
  MUI_XYT("BK", 110, 60, " Nazaj ")
  
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
  krog2.mrtvi_hod = krog1.mrtvi_hod;
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
  u8g2.print((int)temp_hranilnika);
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
    digitalWrite(LCD_LED, !IZKLOP_IZHOD);
  }
}

void handle_state_machine() {
  switch (state_machine) {
    case STATE_IDLE:
      g_prikazi_stran = EKRAN_GLAVNI_ZASLON;
      digitalWrite(LCD_LED, !IZKLOP_IZHOD);
      break;
    
    case STATE_ACTIVE_SCREEN:
      g_prikazi_stran = EKRAN_GLAVNI_ZASLON;
      digitalWrite(LCD_LED, !VKLOP_IZHOD);
      break;

    case STATE_MENU_SCREEN:
      g_prikazi_stran = EKRAN_MENI_ZASLON;
      digitalWrite(LCD_LED, !VKLOP_IZHOD);
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

float upor;
float dobi_temperaturo(uint32_t adc_meritev) {
  float temp = 0;
  upor = (float)(adc_meritev * UPOR) / (float)(ADC_MAX_VREDNOST - adc_meritev);
  temp = (upor - 1000) * 100 / 385;
  return temp;
}

float pid_temp;
uint8_t pid_zanka(ogrevalni_krog_t *krog) {
  uint8_t ukaz = 0;
  float napaka = (float)krog->temp_zelena - (float)krog->temp_kroga;

  if (abs(napaka) > krog->mrtvi_hod) {
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

    if (pid_temp > krog->temp_kroga) {
      ukaz = 1;
    }

    if (pid_temp < krog->temp_kroga) {
      ukaz = -1;
    }
    
    krog->napaka_prej = napaka;
  }
  else {
    ukaz = 0;
  }

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
  if (krog->termostat_vklop == VKLOP_VHOD) {
    digitalWrite(krog->crpalka_pin, VKLOP_IZHOD);
    int8_t ventil_smer = pid_zanka(krog);
    krog->ventil_smer = ventil_smer;

    if (ventil_smer == -1) {  // hladna voda
      digitalWrite(krog->mes_vent_topl_pin, IZKLOP_IZHOD);
      digitalWrite(krog->mes_vent_hlad_pin, VKLOP_IZHOD);
    }
    if (ventil_smer == 0) {  // pusti ventil v trenutni poziciji
      digitalWrite(krog->mes_vent_hlad_pin, IZKLOP_IZHOD);
      digitalWrite(krog->mes_vent_topl_pin, IZKLOP_IZHOD);
    }
    if (ventil_smer == 1) {  // topla voda
      digitalWrite(krog->mes_vent_hlad_pin, IZKLOP_IZHOD);
      digitalWrite(krog->mes_vent_topl_pin, VKLOP_IZHOD);
    }
    krog->cas = millis();
  }
  else {
    // Najprej se mora zapreti mešalni ventil, šele potem se ugasne črpalka.
    // Mešalni ventil se zapira, črpalka pa deluje še 5 min po signalu za izklop.
    if ((millis() - krog->cas) < CAS_IZKLOPA_CRPALKE) {
      digitalWrite(krog->mes_vent_topl_pin, IZKLOP_IZHOD);
      digitalWrite(krog->mes_vent_hlad_pin, VKLOP_IZHOD);
      digitalWrite(krog->crpalka_pin, VKLOP_IZHOD);
    }
    else {
      digitalWrite(krog->mes_vent_hlad_pin, IZKLOP_IZHOD);
      digitalWrite(krog->mes_vent_topl_pin, IZKLOP_IZHOD);
      digitalWrite(krog->crpalka_pin, IZKLOP_IZHOD);
    }
    
  }
}

void zapisi_na_kartico(String dnevnik, String zapis) {
  uint8_t kartica_vstavljena = digitalRead(CD);
  if (SD.begin(SD_CS) && (kartica_vstavljena == SD_VSTAVLJENA)) {
      File dataFile = SD.open(dnevnik, FILE_WRITE);
      if (dataFile) {
        dataFile.println(zapis);
        dataFile.close();
      }
      //Serial.println("Pišem na kartico");
    }
}

void shrani_dnevnik(ogrevalni_krog_t *krog) {
  uint8_t kartica_vstavljena = digitalRead(CD);
  if ((krog->termostat_vklop == VKLOP_VHOD) && (kartica_vstavljena == SD_VSTAVLJENA)) {
    String log_name;
    
    String dataString;
    dataString += String(krog->temp_zelena);
    dataString += ",";
    dataString += String(krog->temp_kroga);
    dataString += ",";
    dataString += String(krog->ventil_smer);

    if (krog->povecam_dnevnik) {
      krog->st_dnevnika++;
      krog->povecam_dnevnik = 0;
      String csv_glava = "T_zelena,T_tren,ventil";
      log_name = krog->ime_kroga;
      log_name += "_" + String(krog->st_dnevnika) + ".csv";
      zapisi_na_kartico(log_name, csv_glava);
    }
    
    log_name = krog->ime_kroga;
    log_name += "_" + String(krog->st_dnevnika) + ".csv";
    zapisi_na_kartico(log_name, dataString);
  }
  else {
    krog->povecam_dnevnik = 1;
    krog->napaka_prej = 0;
    krog->integral = 0;
    krog->odvod = 0;
  }

}

void prezracevanje(ogrevalni_krog_t *krog)
{
  if(krog->prezracevanje) {
    digitalWrite(krog->crpalka_pin, VKLOP_IZHOD);
  }
}

void setup() {
  // put your setup code here, to run once:

  /* Pin setup */

  pinMode(LCD_LED, OUTPUT);
  pinMode(CD, INPUT_PULLUP);

  pinMode(CRPAKLA_KROG_1_DOUT, OUTPUT);
  pinMode(MES_VENT_HLAD_KROG_1_DOUT, OUTPUT);
  pinMode(MES_VENT_TOPL_KROG_1_DOUT, OUTPUT);

  krog1.crpalka_pin = CRPAKLA_KROG_1_DOUT;
  digitalWrite(CRPAKLA_KROG_1_DOUT, IZKLOP_IZHOD);
  
  krog1.mes_vent_hlad_pin = MES_VENT_HLAD_KROG_1_DOUT;
  digitalWrite(MES_VENT_HLAD_KROG_1_DOUT, IZKLOP_IZHOD);
  
  krog1.mes_vent_topl_pin = MES_VENT_TOPL_KROG_1_DOUT;
  digitalWrite(MES_VENT_TOPL_KROG_1_DOUT, IZKLOP_IZHOD);

  pinMode(CRPAKLA_KROG_2_DOUT, OUTPUT);
  pinMode(MES_VENT_HLAD_KROG_2_DOUT, OUTPUT);
  pinMode(MES_VENT_TOPL_KROG_2_DOUT, OUTPUT);

  krog2.crpalka_pin = CRPAKLA_KROG_2_DOUT;
  digitalWrite(CRPAKLA_KROG_2_DOUT, IZKLOP_IZHOD);

  krog2.mes_vent_hlad_pin = MES_VENT_HLAD_KROG_2_DOUT;
  digitalWrite(MES_VENT_HLAD_KROG_2_DOUT, IZKLOP_IZHOD);

  krog2.mes_vent_topl_pin = MES_VENT_TOPL_KROG_2_DOUT;
  digitalWrite(MES_VENT_TOPL_KROG_2_DOUT, IZKLOP_IZHOD);

  pinMode(TEMP_KROG_1_AIN, INPUT_ANALOG);
  pinMode(TEMP_KROG_2_AIN, INPUT_ANALOG);
  pinMode(TEMP_HRANIL_AIN, INPUT_ANALOG);

  pinMode(TERMOSTAT_VKLOP_KROG1_DIN, INPUT);
  pinMode(TERMOSTAT_VKLOP_KROG2_DIN, INPUT);

  krog1.temp_kroga_pin = TEMP_KROG_1_AIN;
  krog2.temp_kroga_pin = TEMP_KROG_2_AIN;
  
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
  krog1.temp_zelena = 40;
  krog1.mrtvi_hod = 2;

  krog2.Kp = 1;
  krog2.Ki = 3;
  krog2.Kd = 1;
  krog2.temp_zelena = 32;
  krog2.mrtvi_hod = 2;

  temp_hranilnika_zelena = 40;
  cas_zakasnitve = 5;  // v minutah
  cas_vzorcenja = 1;  // v sekundah

  ali_narisem = 1;

  Serial.begin(115200);
  Serial.println("Začetek programa.");
}

void loop() {
  // put your main code here, to run repeatedly:
  static unsigned long int pid_zanka_cas = 0;
  static unsigned long int temp_zanka_cas = 0;
  static uint32_t povp_krog1 = 0;
  static uint32_t povp_krog2 = 0;
  static uint32_t povp_hranil = 0;
  static uint32_t st_meritev = 0;
  static uint8_t temp_je_pripravljena = 0;

  zatemnitev_zaslona();
  preberi_vhodne_signale();
  handle_state_machine();

  if ((millis() - temp_zanka_cas) > 10) {
    temp_zanka_cas = millis();
    analogReadResolution(ADC_RESOLUCIJA);
    povp_krog1 += analogRead(krog1.temp_kroga_pin);
    povp_krog2 += analogRead(krog2.temp_kroga_pin);
    povp_hranil += analogRead(TEMP_HRANIL_AIN);

    st_meritev++;
    if (st_meritev > 10) {
      temp_je_pripravljena = 1;
    }
  }

  if (((millis() - pid_zanka_cas) > cas_vzorcenja * 1000) && temp_je_pripravljena) {
    pid_zanka_cas = millis();

    povp_krog1 = povp_krog1 / st_meritev;
    povp_krog2 = povp_krog2 / st_meritev;
    povp_hranil = povp_hranil / st_meritev;
    krog1.temp_kroga = dobi_temperaturo(povp_krog1);
    krog2.temp_kroga = dobi_temperaturo(povp_krog2);
    temp_hranilnika = dobi_temperaturo(povp_hranil);
    Serial.print("Krog 1: ");
    Serial.println(krog1.temp_kroga);
    Serial.print("Krog 2: ");
    Serial.println(krog2.temp_kroga);
    Serial.print("Hranilnik: ");
    Serial.println(temp_hranilnika);
    st_meritev = 0;
    povp_krog1 = 0;
    povp_krog2 = 0;
    povp_hranil = 0;

    krmiljenje_ventilov(&krog1);
    krmiljenje_ventilov(&krog2);
    prezracevanje(&krog1);
    prezracevanje(&krog2);
    shrani_dnevnik(&krog1);
    shrani_dnevnik(&krog2);
    ali_narisem = 1;
    temp_je_pripravljena = 0;
  }

  narisi_zaslon();
}
