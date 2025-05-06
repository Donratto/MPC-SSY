# LWM->MQTT Gateway

## Popis
Tento projekt implementuje gateway pro protokol LWM, která přenáší payload z LWM zpráv pomocí protokolu MQTT. Gateway také umožňuje přijímat konfigurační data prostřednictvím MQTT zpráv. Projekt využívá ovladače dostupné na [GitHubu](https://github.com/maxxir/m1284p_wiz5500) a vychází z LWM projektu.

## Nutná nastavení
1. **Takt oscilátoru**: Nastavte konstantu `F_CPU` podle použitého oscilátoru.
2. **Definice pro MCU**: Definujte konstanty specifické pro dané MCU, jak je uvedeno v dokumentaci na [Microchip](../../docs/Atmel-42029-Lightweight-Mesh-Getting-Started-Guide_Application-Note_AVR2131.pdf).
3. **Konfigurace LWM**:
   - Nastavte proměnné `APP_ADDR`, `APP_PANID` a `APP_ENDPOINT` v souboru `config.h`.
4. **Konfigurace SPI**:
   - Upravte piny a porty v souboru `spi.h`.
   - Podle použitých portů upravte funkci `spi_init()` v souboru `spi.c`.

## Adresáře obsažené v projektu

### `stack`
Obsahuje implementace jednotlivých vrstev a služeb protokolu LWM.  
- **Změny**: V souboru `stack/hal/drivers/atmega256rfr2/src/halUart.c` byla přidána funkce `void HAL_UARTWriteString(char *text)` pro výpis textového řetězce.

### `Internet`
Obsahuje adresáře `DNS` a `MQTT`.  
- `DNS`: Implementace DNS klienta pro překlad doménových jmen.  
- `MQTT`: Implementace MQTT klienta pro publikování a odběr zpráv.  
- **Změny**: Nebyly provedeny žádné změny.

### `Ethernet`
Obsahuje ovladače pro ethernetový modul W5500.  
- Používá se pro ICMP (ping), MQTT a DNS komunikaci.  
- **Změny**: Nebyly provedeny žádné změny.

### `Applications`
Obsahuje:
- `PING`: Funkce pro ICMP ping.
- `loopback`: Funkce pro testování TCP/UDP loopbacku.  
- **Změny**: Nebyly provedeny žádné změny.

## Další soubory
- **`spi.h` a `spi.c`**: Implementace SPI sběrnice pro komunikaci s ethernetovým modulem.  
- **`config.h`**: Definice potřebné pro funkčnost LWM (např. adresa, endpoint).  
- **`uart_extd.c` a `uart_extd.h`**: Obsahují funkce pro UART, ale nejsou potřeba, protože projekt využívá LWM UART.  

---

## Funkce

### `void HAL_UARTWriteString(char *text)`
- Používá funkci `HAL_UartWriteByte()` pro výpis textového řetězce.  
- **Vstup**: Ukazatel na textový řetězec.

### `static bool appDataInd(NWK_DataInd_t *ind)`
- Přijímá data pro registrovaný endpoint.  
- Publikuje payload LWM zprávy pomocí MQTT.  
- **Vstup**: Ukazatel na strukturu `NWK_DataInd_t`.  
- **Výstup**: Pravdivostní hodnota (pro rozhodnutí odeslání potvrzujícího rámce).

### `void messageArrived(MessageData *md)`
- Zpracovává přijaté MQTT zprávy.  
- Umožňuje konfiguraci gateway pomocí MQTT.  
- **Vstup**: Ukazatel na strukturu `MessageData`.

### `int main()`
- Inicializuje systém, provádí DNS dotaz na IP adresu MQTT brokeru a připojuje se k němu.  
- Nastavuje MQTT klienta a přihlašuje se k odběru témat.  
- V nekonečné smyčce volá funkce `SYS_TaskHandler()`, `HAL_UartTaskHandler()` a `APP_TaskHandler()`.  
- Publikuje zprávy na téma definované v `PUBLISH` a přijímá zprávy z tématu `SUBSCRIBE`.

---

## Závěr
Projekt implementuje plně funkční gateway mezi LWM a MQTT. Využívá dostupné ovladače a knihovny, přičemž byly provedeny minimální úpravy pro přizpůsobení konkrétnímu hardwaru a požadavkům projektu.