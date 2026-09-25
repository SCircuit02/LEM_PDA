# Changelog - QC LemBot / LemPDA

Convenciones:
- Cada versión va en su propia carpeta `CODE/Vx.y.z/`.
- La serie 0.x es de desarrollo. La 1.0 queda reservada para el firmware listo para terreno.
- El encabezado de cada `.ino` indica el MCU para el que está hecho.
- Nombres de archivo del S3: `esp-s3-v0.2.x`.

## 0.1.6 - ESP32-C3 sin pantalla ni botones (solo web)
Carpeta `CODE/V0.1.6/tester_esp32_v0_1_6/`, con `tester_esp32_v0_1_6.ino` e `index_html.h` en la misma carpeta.
- La misma GUI web y la misma red que la 0.2.2 (`LemPDA-XXXX`, clave `12345678`, `http://192.168.4.1`), sin OLED, encoder ni botones.
- **I2C movido a SDA = GP0 y SCL = GP1** (antes GP8/GP9). GP0 y GP1 eran el encoder y no son pines de strapping. GP8 queda para el LED RGB.
- LED RGB de estado en GP8 (`USE_RGB_LED`):
  - rojo fijo: el INA228 no responde;
  - rojo fuerte: sobrecorriente;
  - rojo parpadeando: grabando corriente;
  - azul: un test, la descarga o el bus SDI-12 en curso;
  - verde tenue: listo.
- Libres: GP2, GP3, GP4, GP5, GP9 y GP21.
- La lógica de medición y de los tests es la de la 0.1.5, con sus arreglos.
- En el C3, GP7 es a la vez SDI-12 TX, Pulse2 y el dato del SHT10, y GP6 es SDI-12 RX y Pulse1:
  - La web no usa el bus SDI-12 mientras corre Sense-QC o Weather-QC, y esos tests no arrancan mientras el bus está en uso.
  - El SHT10 se lee solo con el botón "Leer SHT10", nunca en segundo plano.
  - Sense-QC vuelve a poner GP6 y GP7 como entradas antes de empezar.
- El INA228 se lee cada 50 ms, porque con el promedio de 128 su dato cambia cada ~0,4 s.
- La batería se mide por el ADS1115 (A3). Si falta el ADS1115, la web muestra "Sin dato de batería".
- La potencia WiFi está en 8,5 dBm, porque muchos C3 SuperMini fallan a potencia máxima.
- Sin INA228, la web sigue funcionando: muestra el error y el gestor SDI-12 sigue disponible.
- Arreglado: en la primera subida de la 0.1.6, el JSON de `/api/data` salía inválido por un comentario mal ubicado, y la página no recibía datos.

## 0.2.2 - ESP32-S3 LOLIN Mini (portal web)
Carpeta `CODE/V0.2.2/esp-s3-v0.2.2/`, con `esp-s3-v0.2.2.ino` e `index_html.h` en la misma carpeta.
- Red WiFi propia `LemPDA-XXXX` (últimos 4 dígitos hex de la MAC), clave `12345678` (`AP_PASS`). WPA2 no acepta claves de menos de 8 caracteres.
- Portal cautivo: al conectarse, el teléfono abre `http://192.168.4.1`. Cualquier otra URL redirige a la página.
- GUI web con 5 pestañas, para usar el equipo sin pantalla, encoder ni botones:
  - **Corriente:** actual, mediana, máxima y mínima; voltaje, carga, promedio y uso estimado; estado de la grabación; gráfico; descarga en CSV (`;` y coma decimal); reinicio de la medición.
  - **Sensores:** batería, BME280/SHT30/SHT10 con la diferencia contra el BME280, ADS1115, GPIO, Soil (DFM) y monitor UART.
  - **SDI-12:** escaneo, identificación aI!, mediciones aC! y aM!, y cambio de ID.
  - **Tests:** Sense-QC, Weather-QC y descarga de batería.
  - **Equipo:** configuración (se guarda en EEPROM), valores por defecto e información del sistema.
- El monitor de corriente, los tests, el UART y el Soil corren en segundo plano, aunque la pantalla muestre otro menú. Se corre una prueba a la vez, desde la pantalla o desde la web.
- Durante la descarga de batería el monitor de corriente queda en pausa, porque la descarga usa el acumulador del INA228.
- El bus SDI-12 se turna entre la pantalla y la web.
- Sin INA228, el equipo queda en modo web: la página muestra el error y el gestor SDI-12 sigue funcionando.
- Los sensores ausentes al arrancar no se leen. Un ADS1115 ausente dejaba colgada su lectura.
- ADS1115, BME280, SHT30 y SHT10 se leen para la web solo mientras la pestaña Sensores está abierta.

## 0.2.1 - ESP32-S3 LOLIN Mini
- Página 1 de corriente en orden actual, mediana, máxima, mínima.
- La vista del sensor SDI-12 muestra el fabricante, el modelo, la versión, la serie y la versión SDI-12 (aI!) junto a los valores.
- Arreglos:
  - Sense-QC ya no arrastra los pulsos de la placa anterior (daba un PASS falso en P1/P2).
  - Cambiar "Samples" ya no deja leer fuera de los arreglos.
  - El SHT10 ausente ya no se lee como -40,1 °C, y el Weather-QC pasa al SHT30.
  - La tabla muestra 4 filas.
  - TX SDI-12 queda en reposo al salir del módulo.

## 0.1.5 - ESP32-C3
- Los mismos cambios y arreglos que la 0.2.1. En el C3, TX SDI-12 no se toca, porque sus pines son compartidos con Pulse y SHT10.

## 0.2.0 - ESP32-C6 Super Mini y ESP32-S3 LOLIN Mini
- Port al C6 con pines seguros (`tester_esp32c6_v0_2_0.ino`). LED RGB para debug visual y menú Zigbee (work in progress).
- Port al S3 (`esp-s3-v0.2.0.ino`). Batería por ADC interno en GP1 (ADC1, sigue funcionando con WiFi) y SDI-12 en GP13/GP10.

## 0.1.4 - ESP32-C3
- SDI-12 mide con aC! por defecto. El click del encoder vuelve a pedir la medición con aM!.

## 0.1.3 - ESP32-C3
- La vista SDI-12 tiene scroll y muestra todos los valores (aD0!..aD9!) con el formato "- valor".

## 0.1.2 - ESP32-C3
- Front-end SDI-12 de hardware con 2 pines por bit-bang: TX GPIO7 → SN74LVC1G3157 → SN74LVC1G240 y RX GPIO6 ← SN74AHC1G14.

## 0.1.1 y 0.1.0.2 - ESP32-C3 (archivos perdidos, no están en el repositorio)
- 0.1.0.2: visor y editor SDI-12.
- 0.1.1: cambio de ID con varios sensores, ícono de batería de 0 a 100 %, corriente en 2 páginas, dirección del encoder en RAW y pantallas sin textos superpuestos.
- Estos cambios están incluidos desde la 0.1.2.

## 0.1.0.1 - ESP32-C3
- Firmware original: Corriente, Sense-QC, Weather-QC, Soil-QC, RAW, Temp & Hum, UART, Descarga, Config y Pinout.

## Extras
- `CODE/EXTRAS/nano_medidor_corriente/`: medidor de corriente con Arduino Nano + INA228 + OLED.
  - El botón en D5 cambia de vista; mantenerlo apretado reinicia la medición.
  - Por Serial manda `I:x,Imed:y,Imax:z,Imin:w`.
- `CODE/EXTRAS/esp32s2_sdi12_web/`: gestor SDI-12 por WiFi para ESP32-S2 (TX GPIO10, RX GPIO11). Usa la misma red y el mismo portal que la 0.2.2.
- `CODE/TEST/test_i2c_oled_c3/`: prueba de I2C y OLED para el ESP32-C3.

## Documentos
- `DOCS/QC_LemBot_Manual_v0.1.4.docx`: manual de usuario.
- `DOCS/QC_LemBot_Pinout_v0.2.0_ESP32-S3.docx`: pinout del S3.
