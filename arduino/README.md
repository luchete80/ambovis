Requisitos para usar el script de deploy

Linux/Unix/MacOS

Instalar arduino-cli https://arduino.github.io/arduino-cli/0.32/installation/
Tener instaladas las librerias externas que require el proyecto, tales como:

Adafruit ADS1X15     2.3.0
Adafruit BusIO       1.7.3
LiquidCrystal        1.0.7
Adafruit GFX Library 1.10.9
Adafruit ILI9341     1.5.6

Otras librerias seran obtenidas del core de avr, que debe ser instalado usando:

arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli core list

Listar los boards aceptados por arduino cli
arduino-cli board listall mega 

Listar los puertos detectados
arduino-cli board list

Luego se puede ejecutar:

sh deploy.sh

Donde se solicitara la password para desencriptar los datos privados, compilara y subirá el codigo a la placa en el puerto indicado. Para evitar que se solicite la password siempre que se ejecuta el script, se puede exportar la variable:
export OPENSSL_PWD=<password>

Si se desea usar el monitor serie, se puede acceder a él a traves de la interfaz grafica de Arduino CC una vez que el codigo esta subido.

Windows

Instalar arduino-cli https://arduino.github.io/arduino-cli/0.32/installation/

TBD

