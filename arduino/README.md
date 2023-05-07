##Requisitos para usar el script de deploy

Linux/OS/Windows:
Instalar arduino-cli https://arduino.github.io/arduino-cli/0.32/installation/

(Windows) Instalar OpenSSL 
Win64: https://slproweb.com/download/Win64OpenSSL_Light-3_1_0.msi
Win32: https://slproweb.com/download/Win32OpenSSL_Light-3_1_0.msi

Si se tiene instalado Arduino CC, la carpeta de librerias ya existe y no hay que hacer nada en particular. Puede chequear la existencia de la ruta `~/Documents/Arduino/libraries` o creala a mano.

Tener instaladas las librerias externas que require el proyecto, tales como:

Adafruit ADS1X15     2.3.0
Adafruit BusIO       1.7.3
LiquidCrystal        1.0.7
Adafruit GFX Library 1.10.9
Adafruit ILI9341     1.5.6

Instalar por unica vez el core de avr, que debe ser instalado usando:
```
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli core list
```
Listar los boards aceptados por arduino cli
`arduino-cli board listall mega`

Listar los puertos detectados:
`arduino-cli board list`

###Ejecución del script

Linux/OS: `sh deploy.sh`
Windows: `powershell deploy.ps1` en modo administrador

Al comenzar el script por primera vez, se solicitara la password para desencriptar ciertos datos y compilara el codigo.
Si la compilacion fue exitosa, se pedira el nombre del puerto donde esta conectada la placa y subirá el codigo. 

Para evitar que se solicite la password siempre que se ejecuta el script se puede exportar la variable OPENSSL_PWD.
Linux/OS: `export OPENSSL_PWD=<password>`
Windows: no es necesario exportar la variable, el script lo hace automaticamente despues de la primera vez.
Si la password es incorrecta, el codigo no compilará, deteniendo la ejecucion del script. 

Si se desea usar el monitor serie, se puede acceder a él a traves de la interfaz grafica de Arduino CC una vez que el codigo esta subidoa la placa.

###Referencias:

https://arduino.github.io/arduino-cli/0.32/
https://github.com/arduino/arduino-cli/issues/38
https://slproweb.com/products/Win32OpenSSL.html
https://learn.microsoft.com/en-us/training/modules/introduction-to-powershell/2-what-is-powershell
