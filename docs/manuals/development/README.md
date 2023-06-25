# Preparación de entorno

### Requisitos
- Sistema Operativo: Windows, Linux/MacOS
- Arduino ATMega 2560

Ver configuración de la placa asociada al Arduino [aquí](../../resources/schematics).

### Frameworks de Desarrollo
Para comenzar con el desarrollo de este proyecto, se recomienda instalar:
- IDE: [Arduino IDE](https://www.arduino.cc/en/software)
  - Opcional: [CLion](https://www.jetbrains.com/es-es/clion/download/)
- Git

### Librerías arduino

Las librerías pueden ser instaladas usando Arduino IDE en Herramientas -> Administrar bibliotecas. En el buscador colocar el nombre de la libreria buscada y elegir la version. Luego presionar en el botón "Instalar"
- Adafruit_ADS1X15 2.3.0
- AdafruitBusIO 1.7.3
- Adafruit GFX Library 1.10.9
- Adafruit_ILI9341 1.5.6
- LiquidCrystal 1.0.7

### Despliegue

El procedimiento de despliegue es a través de un script. 

Hay opciones en bash para Linux/MacOS, en powershell para Windows, y python para cualquier sistema operativo con python3 (previamente instalando los requisitos listados en `requirements.txt`.
