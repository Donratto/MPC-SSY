# LWM->MQTT Gateway

## Popis projektu
Cílem projektu je vytvořit gateway pro protokol LWM, která bude:
1. Odesílat obsah (payload) z LWM zpráv pomocí protokolu MQTT.
2. Přijímat MQTT zprávy pro konfiguraci.

Projekt využívá ovladače pro Ethernet a MQTT stack dostupné na:  
[https://github.com/maxxir/m1284p_wiz5500](https://github.com/maxxir/m1284p_wiz5500). Dále využívá knihovnu potokolu LWM [](../../docs/Atmel-42029-Lightweight-Mesh-Getting-Started-Guide_Application-Note_AVR2131.pdf)

## Nutná nastavení
Pro správnou funkčnost projektu je nutné provést následující kroky:
1. **Nastavení oscilátoru**  
   - Nastavte takt oscilátoru pomocí konstanty `F_CPU` v projektu.
   - Pro ATmega256RFR2 je doporučeno nastavit `F_CPU` na hodnotu 8 MHz.
   - Jako další parametry je třeba definovat `PHY_ATMEGARFR2` a `HAL_ATMEGA256RFR2`

2. **Konfigurace LWM**  
   - V souboru `config.h` nastavte následující proměnné:
     - `APP_ADDR`: Adresa aplikace.
     - `APP_PANID`: Identifikátor PAN sítě.
     - `APP_ENDPOINT`: Koncový bod aplikace.

3. **Konfigurace SPI**  
   - Porty SPI byly nastaveny na rozšiřující rozhraní číslo 2:
     - **PB1**: SPI CLK
     - **PB2**: SPI MOSI
     - **PB3**: SPI MISO
     - **PD4**: Chip Select (SCS)
   - V souboru `spi.h` upravte definice pinů a portů pro SPI.
   - Funkci `spi_init(void)` v souboru `spi.c` přizpůsobte výše uvedeným nastavením, respektive je potřeba nastavit piny jako výstupní v registrech DDRx.

4. **MQTT Konfigurace**  
   - V souboru `main.c` nastavte:
     - `MQTT_targetIP`: IP adresa MQTT brokeru.
     - `ClientID`, `ClientUsername`, `ClientPassword`: Identifikační údaje MQTT klienta.
     - `SUBSCRIBE`: Téma pro odběr zpráv.
     - `PUBLISH`: Téma pro odesílání zpráv.

## Struktura projektu
Projekt je rozdělen do několika adresářů:

### 1. **stack**
- Obsahuje implementace jednotlivých vrstev a služeb protokolu LWM.
  
### 2. **Internet**
- Obsahuje adresáře `DNS` a `MQTT`:
  - `DNS`: Implementace funkcí pro DNS dotazy.
  - `MQTT`: Implementace MQTT klienta.

### 3. **Ethernet**
- Obsahuje ovladače pro ethernetový modul W5500.
- Používá se pro odesílání a příjem zpráv protokolů ICMP (PING), MQTT a DNS.

### 4. **Applications**
- Obsahuje:
  - `PING`: Funkce pro testování PING.
  - `loopback`: Funkce pro testování loopbacku.

### 5. **Hlavní adresář**
Obsahuje následující soubory:
- **`spi.h` a `spi.c`**  
  - Implementace funkcí pro SPI komunikaci.
- **`config.h`**  
  - Definice nutné pro funkčnost LWM.
- **`makra.h`**  
  - Obsahuje makra a definice používané v projektu.

## Funkce

### 1. `void HAL_UARTWriteString(char *text)`
- Byla přidána do hlavního souboru `./main.c` v sekci LWM pro výpis textového řetězce.
- Používá funkci `void HAL_UartWriteByte(uint8_t byte)` pro výpis textového řetězce přes UART, jak samotný název napovídá.
- **Parametr**: Ukazatel na textový řetězec.

### 2. `static bool appDataInd(NWK_DataInd_t *ind)`
- Zpracovává příchozí data pro registrovaný endpoint.
- Publikuje payload LWM zprávy pomocí MQTT.
- **Parametr**: Ukazatel na strukturu `NWK_DataInd_t`.
- **Návratová hodnota**: Pravdivostní hodnota (pro rozhodnutí odeslání potvrzujícího rámce).

### 3. `void messageArrived(MessageData *md)`
- Zpracovává příchozí MQTT zprávy.
- Používá se pro konfiguraci pomocí MQTT.
- **Parametr**: Ukazatel na strukturu `MessageData`.

### 4. `int main()`
- Inicializuje MCU, LWM stack a MQTT klienta.
- Provádí DNS dotaz na IP adresu MQTT brokeru.
- Připojuje se k MQTT brokeru a přihlašuje se k odběru témat.
- Obsahuje nekonečnou smyčku, která:
  - Volá LWM funkce: `SYS_TaskHandler()`, `HAL_UartTaskHandler()`, `APP_TaskHandler()`.
  - Zpracovává MQTT zprávy.
