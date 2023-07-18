# Despliegue en un dispositivo físico

La implementación de este software requiere información que no puede compartirse libremente y ha sido encriptada para respetar la voluntad de su propietario.

Es por ello que **el despliegue debe realizarse usando el script correspondiente y NO el botón de subida de Arduino IDE** para su correcta compilación.

## Requisitos para usar el script de despliegue

Linux/OS/Windows:
Instalar arduino-cli https://arduino.github.io/arduino-cli/0.32/installation/

(Sólo Windows) Instalar OpenSSL

Win64: https://slproweb.com/download/Win64OpenSSL_Light-3_1_0.msi

Win32: https://slproweb.com/download/Win32OpenSSL_Light-3_1_0.msi

Chequear la existencia de la ruta `~/Documents/Arduino/libraries` o crearla a mano. Si se tiene instalado Arduino IDE, la carpeta de librerías ya existe.

Tener instaladas o instalar las librerias externas que requiere el proyecto, tales como:

- Adafruit ADS1X15     2.3.0
- Adafruit BusIO       1.7.3
- LiquidCrystal        1.0.7
- Adafruit GFX Library 1.10.9
- Adafruit ILI9341     1.5.6
- Adafruit STMPE610    1.1.3

Instalar por única vez el core de avr:
```
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli core list
```

### Ejecución del script

Linux/OS: `sh deploy.sh`

Windows: `powershell deploy.ps1` en modo administrador.

1. Al comenzar el script por primera vez, se solicitará la contraseña para desencriptar los datos y compilará el código.

2. Si la compilación fue exitosa, se pedirá el nombre del puerto donde esta conectada la placa y subirá el código. El script mostrará las opciones de puertos disponibles.

#### Guardado de la contraseña
En Linux/OS se puede exportar la variable  `export OPENSSL_PWD=<password>`

Windows: no es necesario exportar la variable, el script lo hace automáticamente despues de la primera vez.

Si la password es incorrecta, el código no compilará, deteniendo la ejecución del script.

Si se desea usar el monitor serie, se puede acceder a él a través de la interfaz gráfica de Arduino IDE una vez que el código esta subido a la placa.

### Algunos comandos útiles

Listar los boards aceptados por arduino cli

`arduino-cli board listall mega`

Listar los puertos detectados

`arduino-cli board list`

### Referencias:

https://arduino.github.io/arduino-cli/0.32/
https://github.com/arduino/arduino-cli/issues/38
https://slproweb.com/products/Win32OpenSSL.html
https://learn.microsoft.com/en-us/training/modules/introduction-to-powershell/2-what-is-powershell
